#include <ion.h>
#include <ion/led.h>
#include <ion/timing.h>
#include <regs/regs.h>
#include <drivers/config/console.h>
#include <drivers/config/display.h>
#include "rpi.h"
#include "display.h"

using namespace Ion::Device::Regs;
using namespace Ion::Rpi::Device;

/* Display bridge, SPI1 slave -> DMA -> LCD.
 *
 * Legacy protocol: every CS-low frame is a full 320x240 landscape frame.
 *
 * Protocol 2 (enabled by the Pi with "DSP:ON" over UART): each update is a
 * pair of CS-low frames, an 8-word header then the pixels. The header gives
 * the panel's memory access mode and the window, so the Pi can send only
 * the damaged rectangle, in the panel's native scan order. SPI stays enabled
 * (SSI low) the whole time; DMA is pointed at the header buffer or at the
 * LCD between frames, from the CS-rising-edge interrupt.
 *
 * Tearing: the ST7789 refreshes its native rows (pages) top to bottom at a
 * rate set by FRCTRL2 (RTNA). The LCD's TE line is timestamped by TIM2 input
 * capture, so for every update we know where the scan is. Before starting
 * the pixel frame we pick an RTNA so that the scan never crosses the write
 * inside the window: each refresh pass then shows either the whole old or
 * the whole new rectangle. */

namespace {

bool s_powered = false;

/* ---- Registers not covered by regs/ ---- */

inline volatile uint32_t & reg32(uint32_t address) { return *reinterpret_cast<volatile uint32_t *>(address); }

constexpr uint32_t RCCAPB1ENR = 0x40023840;
constexpr uint32_t RCCAPB2RSTR = 0x40023824;
constexpr uint32_t SPI1RST = 1u << 12;
constexpr uint32_t TIM2Base = 0x40000000;
constexpr uint32_t TIM2CR1 = TIM2Base + 0x00;
constexpr uint32_t TIM2EGR = TIM2Base + 0x14;
constexpr uint32_t TIM2CCMR2 = TIM2Base + 0x1C;
constexpr uint32_t TIM2CCER = TIM2Base + 0x20;
constexpr uint32_t TIM2CNT = TIM2Base + 0x24;
constexpr uint32_t TIM2PSC = TIM2Base + 0x28;
constexpr uint32_t TIM2ARR = TIM2Base + 0x2C;
constexpr uint32_t TIM2SR = TIM2Base + 0x10;

constexpr uint32_t DMA2Base = 0x40026400;
constexpr uint32_t DMALIFCR = DMA2Base + 0x08;
constexpr uint32_t DMASxCR = DMA2Base + 0x10 + 0x18 * DMAStream;
constexpr uint32_t DMASxNDTR = DMASxCR + 0x04;
constexpr uint32_t DMASxM0AR = DMASxCR + 0x0C;
static_assert(DMAStream == 2, "DMA flag mask below is for stream 2");
constexpr uint32_t DMAStream2Flags = 0x003D0000;
// Channel 3 (SPI1_RX), half-word memory and peripheral, peripheral to memory
constexpr uint32_t DMABaseConfig = (3u << 25) | (1u << 13) | (1u << 11);
constexpr uint32_t DMAMemoryIncrement = 1u << 10;
constexpr uint32_t DMACircular = 1u << 8;

constexpr uint32_t SPI1Base = 0x40013000;
constexpr uint32_t SPI1SR = SPI1Base + 0x08;
constexpr uint32_t SPI1DR = SPI1Base + 0x0C;

/* ---- Timing: TIM2 free-running at 1 MHz, TE captured on falling edge ---- */

constexpr int TimerMHz = 96; // APB1 is 48 MHz with a /2 prescaler, so timers run at 96 MHz
// TE: PB10 on N0100 (TIM2_CH3), PB11 on N0110 (TIM2_CH4), both AF1
inline bool TEOnChannel3() { return Ion::Device::Display::Config::TearingEffectPin == GPIOPin(GPIOB, 10); }
inline bool HasTECapture() { return TEOnChannel3() || Ion::Device::Display::Config::TearingEffectPin == GPIOPin(GPIOB, 11); }
inline uint32_t TIM2CCRTE() { return TIM2Base + (TEOnChannel3() ? 0x3C : 0x40); }
inline uint32_t TECaptureFlag() { return TEOnChannel3() ? (1u << 3) : (1u << 4); }

inline uint32_t now() { return reg32(TIM2CNT); }

void initTimer() {
  reg32(RCCAPB1ENR) |= 1; // TIM2EN
  reg32(TIM2CR1) = 0;
  reg32(TIM2PSC) = TimerMHz - 1;
  reg32(TIM2ARR) = 0xFFFFFFFF;
  if (HasTECapture()) {
    GPIOPin te = Ion::Device::Display::Config::TearingEffectPin;
    te.group().AFR()->setAlternateFunction(te.pin(), GPIO::AFR::AlternateFunction::AF1);
    te.group().MODER()->setMode(te.pin(), GPIO::MODER::Mode::AlternateFunction);
    // Input capture on TIx, filter 4 samples, falling edge (end of vertical blanking)
    if (TEOnChannel3()) {
      reg32(TIM2CCMR2) = (reg32(TIM2CCMR2) & ~0xFFu) | 0x21;
      reg32(TIM2CCER) = (reg32(TIM2CCER) & ~(0xFu << 8)) | (0x3u << 8);
    } else {
      reg32(TIM2CCMR2) = (reg32(TIM2CCMR2) & ~0xFF00u) | 0x2100;
      reg32(TIM2CCER) = (reg32(TIM2CCER) & ~(0xFu << 12)) | (0x3u << 12);
    }
  }
  reg32(TIM2EGR) = 1;
  reg32(TIM2CR1) = 1;
}

void releaseTE() {
  if (HasTECapture()) {
    GPIOPin te = Ion::Device::Display::Config::TearingEffectPin;
    te.group().MODER()->setMode(te.pin(), GPIO::MODER::Mode::Input);
  }
}

/* ---- LCD ---- */

namespace Display = Ion::Device::Display;

inline void lcdCommand(Display::Command c) { *Display::CommandAddress = c; }
inline void lcdData(uint8_t d) { *Display::DataAddress = d; }
inline void lcdWindow(Display::Command c, uint16_t start, uint16_t end) {
  lcdCommand(c);
  lcdData(start >> 8);
  lcdData(start & 0xFF);
  lcdData(end >> 8);
  lcdData(end & 0xFF);
}

constexpr uint8_t CalculatorRTNA = 0x1E; // 40 Hz, what initPanel() sets
constexpr int ActiveLines = 320;
constexpr int TotalLines = 344; // + default porches (PORCTRL 0x0C / 0x0C)
constexpr int MinRTNA = 4;      // ~90 Hz
constexpr int MaxRTNA = 30;     // ~40 Hz

/* ---- Protocol 2 state (ISR-owned once enabled) ---- */

constexpr uint16_t HeaderMagic = 0x4E57; // "NW"
constexpr uint16_t HeaderVersion = 2;
constexpr int HeaderWords = 8;
volatile uint16_t s_header[HeaderWords];

volatile bool s_v2 = false;
bool s_expectPixels = false;
uint32_t s_fallTime;       // last CS falling edge
uint32_t s_headerRiseTime; // end of last valid header
uint32_t s_pixels;         // pixels announced by the last header

// Panel and link model, in microseconds
float s_oscillator = 1.0f; // measured / nominal line time
uint8_t s_rtna = CalculatorRTNA;
uint8_t s_rtnaBefore = CalculatorRTNA;
uint32_t s_rtnaChangeTime;
float s_pixelUs = 16.0f / 66.67f; // wire time per pixel, measured from full frames
float s_gapUs = 60.0f;             // header end to pixel start, measured

struct Stats {
  uint32_t frames, badHeaders, rtnaChanges, unavoidable, maxIsrUs, late;
};
Stats s_stats;
uint16_t s_badHeader[HeaderWords + 1]; // last rejected header, then its DMA count left

inline float lineUs(int rtna) { return s_oscillator * 0.1f * (250 + 16 * rtna); }

void setRTNA(int rtna) {
  if (rtna == s_rtna) {
    return;
  }
  lcdCommand(Display::Command::FrameRateControl);
  lcdData(rtna);
  s_rtnaBefore = s_rtna;
  s_rtna = rtna;
  s_rtnaChangeTime = now();
  s_stats.rtnaChanges++;
}

/* Does refresh pass starting at @start (line time @line) show a mix of old
 * and new pages of the window being written? Page p of [p0, p1] is new in
 * that pass if it is scanned after its write ends. The sign of
 * start + line*p - writeEnd(p) is linear in p, so the ends decide. */
inline bool passTears(float start, float line, int p0, int p1, float writeStart, float pageUs) {
  float writeEnd = writeStart + (p1 - p0 + 1) * pageUs;
  if (start + ActiveLines * line < writeStart || start > writeEnd) {
    return false;
  }
  float first = start + line * p0 - (writeStart + pageUs);
  float last = start + line * p1 - writeEnd;
  return (first > 0) != (last > 0);
}

/* Would some refresh pass tear if FRCTRL2 were set to @rtna now? (It takes
 * effect from the next refresh; the current one keeps its line time.) */
bool tears(int rtna, uint32_t teTime, int p0, int p1, float writeStart, float pageUs) {
  int frameRTNA = static_cast<int32_t>(s_rtnaChangeTime - teTime) > 0 ? s_rtnaBefore : s_rtna;
  float line = lineUs(frameRTNA);
  float writeEnd = writeStart + (p1 - p0 + 1) * pageUs;
  if (passTears(0, line, p0, p1, writeStart, pageUs)) {
    return true;
  }
  float start = line * TotalLines;
  line = lineUs(rtna);
  for (int i = 0; i < 4 && start <= writeEnd; i++) {
    if (passTears(start, line, p0, p1, writeStart, pageUs)) {
      return true;
    }
    start += line * TotalLines;
  }
  return false;
}

/* This runs between the header and the pixels, and every rate tried costs
 * about a microsecond: search only this far from the preferred rate. */
constexpr int SearchSteps = 6;

/* Pick the refresh rate for a window of native pages [p0, p1] that is
 * @columns wide. A pass tears when it starts inside a window of length
 * (pages - 1) * |line time - page write time|: with the scan moving exactly
 * as fast as the write, it never can. So prefer the rate whose line time is
 * nearest this window's page write time, then the current rate, then the
 * nearest tear-free rate. */
void chooseRefreshRate(int p0, int p1, int columns, uint32_t time) {
  if (!HasTECapture()) {
    return;
  }
  uint32_t teTime = reg32(TIM2CCRTE());
  float sinceTE = static_cast<int32_t>(time - teTime);
  if (sinceTE < 0 || sinceTE > 2 * lineUs(MaxRTNA) * TotalLines) {
    return; // no recent TE edge, nothing to sync to
  }
  float writeStart = sinceTE + s_gapUs;
  float pageUs = columns * s_pixelUs;

  int preferred = static_cast<int>((pageUs / (0.1f * s_oscillator) - 250) / 16 + 0.5f);
  preferred = preferred < MinRTNA ? MinRTNA : (preferred > MaxRTNA ? MaxRTNA : preferred);

  if (!tears(preferred, teTime, p0, p1, writeStart, pageUs)) {
    setRTNA(preferred);
    return;
  }
  if (!tears(s_rtna, teTime, p0, p1, writeStart, pageUs)) {
    return;
  }
  for (int d = 1; d <= SearchSteps; d++) {
    for (int sign = -1; sign <= 1; sign += 2) {
      int r = preferred + sign * d;
      if (r >= MinRTNA && r <= MaxRTNA && !tears(r, teTime, p0, p1, writeStart, pageUs)) {
        setRTNA(r);
        return;
      }
    }
  }
  s_stats.unavoidable++;
  setRTNA(preferred);
}

/* Measure the panel's real line time once: oscillator tolerance is ±10%. */
void calibratePanel() {
  if (!HasTECapture()) {
    return;
  }
  constexpr int rtna = 20;
  lcdCommand(Display::Command::FrameRateControl);
  lcdData(rtna);
  s_rtnaBefore = s_rtna = rtna;
  uint32_t edges[3];
  for (int i = 0; i < 3; i++) {
    reg32(TIM2SR) = ~TECaptureFlag();
    uint64_t deadline = Ion::Timing::millis() + 50;
    while (!(reg32(TIM2SR) & TECaptureFlag())) {
      if (Ion::Timing::millis() > deadline) {
        return;
      }
    }
    edges[i] = reg32(TIM2CCRTE());
  }
  // The first period may straddle the RTNA change
  float period = edges[2] - edges[1];
  float oscillator = period / (0.1f * (250 + 16 * rtna) * TotalLines);
  if (oscillator > 0.7f && oscillator < 1.4f) {
    s_oscillator = oscillator;
  }
  s_rtnaChangeTime = edges[0];
}

/* ---- DMA / SPI ---- */

void armDMA(uint32_t memory, uint32_t count, uint32_t mode) {
  reg32(DMASxCR) &= ~1u;
  while (reg32(DMASxCR) & 1u) {
  }
  reg32(DMALIFCR) = DMAStream2Flags;
  reg32(DMASxM0AR) = memory;
  reg32(DMASxNDTR) = count;
  reg32(DMASxCR) = DMABaseConfig | mode;
  reg32(DMASxCR) |= 1u;
}

void armHeader() { armDMA(reinterpret_cast<uint32_t>(s_header), HeaderWords, DMAMemoryIncrement); }
void armLCD() { armDMA(reinterpret_cast<uint32_t>(Display::DataAddress), 1, DMACircular); }

/* Between frames: restart the SPI so a glitch can never leave it out of
 * half-word alignment, and drop any overrun. */
void resetSPI() {
  /* Only a peripheral reset restarts the slave's bit count; SPE and SSI do
   * not. Without it, one stray clock edge (the Pi booting, or SSI dropped
   * in the middle of a frame) leaves every later frame misaligned. */
  reg32(RCCAPB2RSTR) |= SPI1RST;
  reg32(RCCAPB2RSTR) &= ~SPI1RST;
  SPI1.CR1()->setRXONLY(true);
  SPI1.CR1()->setSSM(true);
  SPI1.CR1()->setDFF(true);
  SPI1.CR1()->setSPE(true);
  SPI1.CR2()->setRXDMAEN(true);
}

bool takeHeader(uint8_t * madctl, uint16_t * c0, uint16_t * c1, uint16_t * p0, uint16_t * p1) {
  if (reg32(DMASxNDTR) != 0) {
    return false;
  }
  uint16_t check = 0xFFFF;
  for (int i = 0; i < HeaderWords - 1; i++) {
    check ^= s_header[i];
  }
  if (s_header[0] != HeaderMagic || s_header[1] != HeaderVersion || s_header[7] != check) {
    return false;
  }
  *madctl = s_header[2];
  *c0 = s_header[3];
  *c1 = s_header[4];
  *p0 = s_header[5];
  *p1 = s_header[6];
  return *c0 <= *c1 && *c1 < 320 && *p0 <= *p1 && *p1 < 320;
}

void v2Rise(uint32_t time) {
  if (s_expectPixels) {
    /* A header frame is ~2 us long, so its falling-edge interrupt can run
     * after CS is already high and handle it as the rise; the real rise
     * then lands here. The Pi waits header_gap_us (40 us by default) before
     * the pixel frame. */
    if (static_cast<int32_t>(time - s_headerRiseTime) < 20) {
      return;
    }
    s_expectPixels = false;
    float duration = static_cast<int32_t>(time - s_fallTime);
    if (s_pixels >= 20000 && duration > 0) {
      s_pixelUs += 0.1f * (duration / s_pixels - s_pixelUs);
    }
    float gap = static_cast<int32_t>(s_fallTime - s_headerRiseTime);
    if (gap > 0 && gap < 2000) {
      s_gapUs += 0.1f * (gap - s_gapUs);
    }
    s_stats.frames++;
    resetSPI();
    armHeader();
    return;
  }

  uint8_t madctl;
  uint16_t c0, c1, p0, p1;
  if (reg32(DMASxNDTR) == HeaderWords) {
    return; // duplicate edge, nothing received
  }
  resetSPI();
  if (!takeHeader(&madctl, &c0, &c1, &p0, &p1)) {
    s_stats.badHeaders++;
    for (int i = 0; i < HeaderWords; i++) {
      s_badHeader[i] = s_header[i];
    }
    s_badHeader[HeaderWords] = reg32(DMASxNDTR);
    armHeader();
    return;
  }
  s_headerRiseTime = time;
  s_pixels = (c1 - c0 + 1) * (p1 - p0 + 1);
  // Pages run along the scan only without row/column exchange
  if (!(madctl & 0x20)) {
    chooseRefreshRate(p0, p1, c1 - c0 + 1, time);
  }
  lcdCommand(Display::Command::MemoryAccessControl);
  lcdData(madctl);
  lcdWindow(Display::Command::ColumnAddressSet, c0, c1);
  lcdWindow(Display::Command::PageAddressSet, p0, p1);
  lcdCommand(Display::Command::MemoryWrite);
  // Pixels overran the SPI before the DMA was ready: the header gap is too short
  if (reg32(SPI1SR) & (1u << 6)) {
    s_stats.late++;
  }
  armLCD();
  s_expectPixels = true;
}

/* Switch protocol between frames, with the CS interrupt masked. */
void setProtocol(bool v2) {
  uint64_t deadline = Ion::Timing::millis() + 50;
  while (!GPIOA.IDR()->get(ChipSelectPin) && Ion::Timing::millis() < deadline) {
  }
  EXTI.IMR()->set(ChipSelectPin, false);
  if (v2) {
    if (!s_v2) {
      calibratePanel();
    }
    s_expectPixels = false;
    resetSPI();
    armHeader();
    SPI1.CR1()->setSSI(false);
  } else {
    armLCD();
    SPI1.CR1()->setSSI(true);
    lcdCommand(Display::Command::FrameRateControl);
    lcdData(CalculatorRTNA);
    s_rtna = CalculatorRTNA;
  }
  s_v2 = v2;
  EXTI.PR()->set(ChipSelectPin, true);
  EXTI.IMR()->set(ChipSelectPin, true);
}

/* ---- UART commands from the Pi: "DSP:?", "DSP:ON", "DSP:OFF", "DSP:STAT",
 * "DSP:PX:x,y", "SYS:DFU" ---- */

char s_line[24];
int s_lineLength = 0;
bool s_dfuRequested = false;

void appendNumber(char * & p, const char * label, uint32_t v) {
  while (*label) {
    *p++ = *label++;
  }
  char digits[10];
  int n = 0;
  do {
    digits[n++] = '0' + v % 10;
    v /= 10;
  } while (v);
  while (n) {
    *p++ = digits[--n];
  }
}

void sendStats() {
  char buffer[192];
  char * p = buffer;
  appendNumber(p, "DSP:STAT frames=", s_stats.frames);
  appendNumber(p, " bad=", s_stats.badHeaders);
  appendNumber(p, " rtna=", s_rtna);
  appendNumber(p, " switches=", s_stats.rtnaChanges);
  appendNumber(p, " torn=", s_stats.unavoidable);
  appendNumber(p, " isr_us=", s_stats.maxIsrUs);
  appendNumber(p, " px_ns=", static_cast<uint32_t>(s_pixelUs * 1000));
  appendNumber(p, " gap_us=", static_cast<uint32_t>(s_gapUs));
  appendNumber(p, " osc_pm=", static_cast<uint32_t>(s_oscillator * 1000));
  appendNumber(p, " late=", s_stats.late);
  // Last rejected header words and the DMA count left: what the wire delivered
  *p++ = ' ';
  for (int i = 0; i <= HeaderWords; i++) {
    for (int shift = 12; shift >= 0; shift -= 4) {
      int d = (s_badHeader[i] >> shift) & 0xF;
      *p++ = d < 10 ? '0' + d : 'A' + d - 10;
    }
    *p++ = i < HeaderWords ? '.' : 0;
  }
  p--;
  *p = 0;
  Ion::Console::writeLine(buffer);
}

int parseNumber(const char * & p) {
  int v = 0;
  while (*p >= '0' && *p <= '9') {
    v = 10 * v + (*p++ - '0');
  }
  return v;
}

/* "DSP:PX:x,y": read 4 landscape pixels back from the panel's GRAM, so the
 * Pi can check what actually reached the screen. Only between protocol 2
 * frames, when the DMA targets the header buffer and the LCD bus is idle;
 * interrupts stay off for the few microseconds of the read. */
void readPixels(const char * arg) {
  int x = parseNumber(arg);
  if (*arg++ != ',') {
    return;
  }
  int y = parseNumber(arg);
  if (!s_v2 || x > 316 || y > 239) {
    Ion::Console::writeLine("DSP:PX ERR");
    return;
  }
  KDColor pixels[4];
  bool done = false;
  for (int tries = 0; tries < 200 && !done; tries++) {
    asm volatile("cpsid i" ::: "memory");
    if (!s_expectPixels) {
      Ion::Display::pullRect(KDRect(x, y, 4, 1), pixels);
      done = true;
    }
    asm volatile("cpsie i" ::: "memory");
    if (!done) {
      Ion::Timing::usleep(100);
    }
  }
  if (!done) {
    Ion::Console::writeLine("DSP:PX BUSY");
    return;
  }
  char buffer[64];
  char * p = buffer;
  appendNumber(p, "DSP:PX ", x);
  appendNumber(p, ",", y);
  *p++ = '=';
  for (int i = 0; i < 4; i++) {
    uint16_t v = static_cast<uint16_t>(pixels[i]);
    for (int shift = 12; shift >= 0; shift -= 4) {
      int d = (v >> shift) & 0xF;
      *p++ = d < 10 ? '0' + d : 'A' + d - 10;
    }
    *p++ = i < 3 ? ',' : 0;
  }
  Ion::Console::writeLine(buffer);
}

void handleLine(const char * line) {
  if (line[0] == 'S' && line[1] == 'Y' && line[2] == 'S' && line[3] == ':' &&
      line[4] == 'D' && line[5] == 'F' && line[6] == 'U' && line[7] == 0) {
    s_dfuRequested = true;
    return;
  }
  if (line[0] != 'D' || line[1] != 'S' || line[2] != 'P' || line[3] != ':') {
    return;
  }
  const char * arg = line + 4;
  if (arg[0] == '?' && arg[1] == 0) {
    Ion::Console::writeLine("DSP:2");
  } else if (arg[0] == 'O' && arg[1] == 'N' && arg[2] == 0) {
    setProtocol(true);
    Ion::Console::writeLine("DSP:OK");
  } else if (arg[0] == 'O' && arg[1] == 'F' && arg[2] == 'F' && arg[3] == 0) {
    setProtocol(false);
    Ion::Console::writeLine("DSP:OK");
  } else if (arg[0] == 'S' && arg[1] == 'T') {
    sendStats();
  } else if (arg[0] == 'P' && arg[1] == 'X' && arg[2] == ':') {
    readPixels(arg + 3);
  }
}

void pollUART() {
  using namespace Ion::Device::Console;
  while (Config::Port.SR()->getRXNE()) {
    char c = Config::Port.RDR()->get();
    if (c == '\n' || c == '\r') {
      s_line[s_lineLength] = 0;
      if (s_lineLength > 0) {
        handleLine(s_line);
      }
      s_lineLength = 0;
    } else if (s_lineLength < static_cast<int>(sizeof(s_line)) - 1) {
      s_line[s_lineLength++] = c;
    }
  }
}

}

bool Ion::Rpi::isPowered() {
  return s_powered;
}

/* Pins, SPI1 slave, its RX DMA stream and the CS interrupt, from scratch.
 * Also run on every Pi app entry: a glitch on the link (the Pi reloading
 * its display driver) can leave the receiver in a state the per-frame
 * reset does not clear, and only this (or a reset) recovered it. */
static void initBridge() {
  // SPI GPIO
  GPIOA.MODER()->setMode(5, GPIO::MODER::Mode::AlternateFunction);
  GPIOA.AFR()->setAlternateFunction(5, GPIO::AFR::AlternateFunction::AF5);
  GPIOA.MODER()->setMode(6, GPIO::MODER::Mode::Input);
  GPIOA.PUPDR()->setPull(6, GPIO::PUPDR::Pull::Up);
  GPIOA.MODER()->setMode(7, GPIO::MODER::Mode::AlternateFunction);
  GPIOA.AFR()->setAlternateFunction(7, GPIO::AFR::AlternateFunction::AF5);

  // SPI, from reset
  reg32(RCCAPB2RSTR) |= SPI1RST;
  reg32(RCCAPB2RSTR) &= ~SPI1RST;
  SPI1.CR1()->setRXONLY(true);
  SPI1.CR1()->setSSI(true); // Software chip select
  SPI1.CR1()->setSSM(true); // Software chip select mode
  SPI1.CR1()->setDFF(true); // 16 bits
  SPI1.CR1()->setSPE(true); // enable
  SPI1.CR2()->setRXDMAEN(true); // enable DMA requests

  // DMA: SPI1 RX into the LCD, circular (armLCD sets everything else)
  DMAEngine.SPAR(DMAStream)->set((uint32_t)SPI1.DR());
  armLCD();

  // ISR
  SYSCFG.EXTICR2()->setEXTI(Ion::Rpi::Device::ChipSelectPin, Ion::Rpi::Device::ChipSelectGPIO);
  EXTI.RTSR()->set(Ion::Rpi::Device::ChipSelectPin, true);
  EXTI.FTSR()->set(Ion::Rpi::Device::ChipSelectPin, true);
}

static void enableDisplay() {
  initBridge();
  initTimer();
  if (s_v2) {
    // The calculator UI ran at its own rate meanwhile; re-sync and re-arm
    s_v2 = false;
    setProtocol(true);
  } else {
    EXTI.IMR()->set(Ion::Rpi::Device::ChipSelectPin, true);
  }
}

static void setOn() {
  PowerPin.group().MODER()->setMode(PowerPin.pin(), GPIO::MODER::Mode::Output);
  PowerPin.group().ODR()->set(PowerPin.pin(), true);  // HIGH enables SX1308 boost
  s_powered = true;
  enableDisplay();
  Ion::LED::setColor(KDColor::RGB24(0x1A0005)); // Dim pink while Pi is on
}

static void disableDisplay() {
  EXTI.IMR()->set(Ion::Rpi::Device::ChipSelectPin, false);
  SPI1.CR1()->setSSI(true);
  if (s_v2) {
    // Give the panel back to the calculator UI in the state it expects
    armLCD();
    lcdCommand(Display::Command::FrameRateControl);
    lcdData(CalculatorRTNA);
    s_rtna = CalculatorRTNA;
  }
  releaseTE();
  reg32(TIM2CR1) = 0;
}

static void setOff() {
  disableDisplay();
  s_powered = false;
  // Drive LOW to disable SX1308 boost (PCB has 10k pull-down)
  PowerPin.group().MODER()->setMode(PowerPin.pin(), GPIO::MODER::Mode::Output);
  PowerPin.group().ODR()->set(PowerPin.pin(), false);
  Ion::LED::setColor(KDColorBlack);
}

extern "C" void rpi_isr() {
  EXTI.PR()->set(Ion::Rpi::Device::ChipSelectPin, true);
  bool high = GPIOA.IDR()->get(6);

  if (s_v2) {
    uint32_t start = now();
    if (high) {
      v2Rise(start);
      uint32_t isrUs = now() - start;
      if (isrUs > s_stats.maxIsrUs) {
        s_stats.maxIsrUs = isrUs;
      }
    } else {
      s_fallTime = start;
    }
    return;
  }

  if (high) {
    SPI1.CR1()->setSSI(true);
  } else {
    Ion::Device::Display::setDrawingArea(KDRect(0,0,320,240), Ion::Device::Display::Orientation::Landscape);
    *Ion::Device::Display::CommandAddress = Ion::Device::Display::Command::MemoryWrite;
    SPI1.CR1()->setSSI(false);
  }
}

bool Ion::Rpi::transferControl() {
  Ion::Display::pushRectUniform(KDRect(0,0,320,240), KDColor::RGB24(0x808080));
  setOn();
  // Tell the Pi this firmware speaks display protocol 2 (it answers DSP:ON)
  Ion::Console::writeLine("DSP:2");

  #define TO_HEX(i) (i <= 9 ? '0' + i : 'A' - 10 + i)
  uint64_t scan, lastScan = 0;
  char buf[18];
  while((scan = Ion::Keyboard::scan()) != 0x40) { // HOME
    pollUART();
    if (s_dfuRequested) {
      /* "SYS:DFU" from the Pi: become flashable over USB without anyone
       * touching the calculator. A flash ends with a reset; otherwise
       * (timeout, Back key) we resume bridging. */
      s_dfuRequested = false;
      uint64_t requestTime = Ion::Timing::millis();
      Ion::Console::writeLine("SYS:DFU");
      disableDisplay();
      Ion::USB::enable();
      uint64_t deadline = Ion::Timing::millis() + 10000;
      while (!Ion::USB::isEnumerated() && Ion::Timing::millis() < deadline) {
      }
      if (Ion::USB::isEnumerated()) {
        Ion::USB::DFU(true);
      }
      Ion::USB::disable();
      /* No flash happened (Back key, unplugged, timeout). The Pi powers off
       * when it sees our SYS:DFU, so let it finish, then power-cycle it. */
      while (Ion::Timing::millis() < requestTime + 20000) {
      }
      setOff();
      Ion::Timing::msleep(1000);
      setOn();
      continue;
    }
    if(scan != lastScan) {
      buf[17] = '\0';
      buf[16] = TO_HEX(((scan >> 0) & 0x0f));
      buf[15] = TO_HEX(((scan >> 4) & 0x0f));
      buf[14] = TO_HEX(((scan >> 8) & 0x0f));
      buf[13] = TO_HEX(((scan >> 12) & 0x0f));
      buf[12] = TO_HEX(((scan >> 16) & 0x0f));
      buf[11] = TO_HEX(((scan >> 20) & 0x0f));
      buf[10] = TO_HEX(((scan >> 24) & 0x0f));
      buf[9] = TO_HEX(((scan >> 28) & 0x0f));
      buf[8] = TO_HEX(((scan >> 32) & 0x0f));
      buf[7] = TO_HEX(((scan >> 36) & 0x0f));
      buf[6] = TO_HEX(((scan >> 40) & 0x0f));
      buf[5] = TO_HEX(((scan >> 44) & 0x0f));
      buf[4] = TO_HEX(((scan >> 48) & 0x0f));
      buf[3] = TO_HEX(((scan >> 52) & 0x0f));
      buf[2] = TO_HEX(((scan >> 56) & 0x0f));
      buf[1] = TO_HEX(((scan >> 60) & 0x0f));
      buf[0] = ':';
      Ion::Console::writeLine(buf);
    }
    lastScan = scan;
  }
  #undef TO_HEX
  disableDisplay();
  return true;
}

void Ion::Rpi::Device::init() {
  setOff();
  initBridge();
  NVIC.NVIC_ISER0()->set(23, true);
}

/* Also linked into the RAM-resident DFU code, which has no .bss/.data:
 * touch registers only, no driver state. */
void Ion::Rpi::Device::shutdown() {
  EXTI.IMR()->set(Ion::Rpi::Device::ChipSelectPin, false);
  SPI1.CR1()->setSSI(true);
  PowerPin.group().MODER()->setMode(PowerPin.pin(), GPIO::MODER::Mode::Output);
  PowerPin.group().ODR()->set(PowerPin.pin(), false);
  Ion::LED::setColor(KDColorBlack);

  NVIC.NVIC_ISER0()->set(23, false);
  SPI1.CR2()->setRXDMAEN(false);
  DMAEngine.SCR(DMAStream)->setEN(false);

  // SPI GPIO
  GPIOA.MODER()->setMode(5, GPIO::MODER::Mode::Analog);
  GPIOA.PUPDR()->setPull(5, GPIO::PUPDR::Pull::None);
  GPIOA.MODER()->setMode(6, GPIO::MODER::Mode::Analog);
  GPIOA.PUPDR()->setPull(6, GPIO::PUPDR::Pull::None);
  GPIOA.MODER()->setMode(7, GPIO::MODER::Mode::Analog);
  GPIOA.PUPDR()->setPull(7, GPIO::PUPDR::Pull::None);
}
