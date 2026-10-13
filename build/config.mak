# You can override those settings on the command line

PLATFORM ?= device
DEBUG ?= 0

HOME_DISPLAY_EXTERNALS ?= 1
EPSILON_VERSION ?= 15.5.0
OMEGA_VERSION ?= 2.0.2
UPSILON_VERSION ?= 1.1.2
# OMEGA_USERNAME ?= N/A
OMEGA_STATE ?= dev
EPSILON_APPS ?= calculation graph rpn code statistics probability solver atomic sequence regression reader settings external
SUBMODULES_APPS = atomic rpn
EPSILON_I18N ?= en fr nl pt it de es hu
EPSILON_COUNTRIES ?= WW CA DE ES FR GB IT NL PT US
EPSILON_GETOPT ?= 0
ESCHER_LOG_EVENTS_BINARY ?= 0
THEME_NAME ?= upsilon_light
THEME_REPO ?= local
INCLUDE_ULAB ?= 1
ENABLE_RPI ?= 0

ifeq ($(ENABLE_RPI),1)
SFLAGS += -DENABLE_RPI=1
EPSILON_APPS += rpi
endif

# Development: open the Pi app at boot, so the Pi powers up after a reflash
RPI_AUTOSTART ?= 0
ifeq ($(RPI_AUTOSTART),1)
SFLAGS += -DRPI_AUTOSTART=1
endif
