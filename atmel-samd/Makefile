# Select the board to build for: if not given on the command line,
# then default to PYBV10.
BOARD ?= samd51_xplained
ifeq ($(wildcard boards/$(BOARD)/.),)
$(error Invalid BOARD specified)
endif


# If the build directory is not given, make it reflect the board name.
BUILD ?= build-$(BOARD)

include ../py/mkenv.mk
-include mpconfigport.mk
include boards/$(BOARD)/mpconfigboard.mk

# qstr definitions (must come before including py.mk)
QSTR_DEFS = qstrdefsport.h

# include py core make definitions
include ../py/py.mk

CROSS_COMPILE = arm-none-eabi-

BOSSAC := tools/bossac_osx

HAL_DIR=hal/$(MCU_SERIES)

INC += -I. \
       -I.. \
       -I../lib/mp-readline \
       -I../lib/timeutils \
       -Iasf4/$(CHIP_FAMILY) \
       -Iasf4/$(CHIP_FAMILY)/hal/include \
       -Iasf4/$(CHIP_FAMILY)/hal/utils/include \
       -Iasf4/$(CHIP_FAMILY)/hri \
       -Iasf4/$(CHIP_FAMILY)/hpl/port \
       -Iasf4/$(CHIP_FAMILY)/include \
       -Iasf4/$(CHIP_FAMILY)/CMSIS/Include \
       -Iasf4/$(CHIP_FAMILY)/usb \
       -Iasf4/$(CHIP_FAMILY)/usb/class/cdc \
       -Iasf4/$(CHIP_FAMILY)/usb/class/hid \
       -Iasf4/$(CHIP_FAMILY)/usb/device \
       -Iboards/$(BOARD) \
       -Ifreetouch \
       -I$(BUILD)

BASE_CFLAGS = \
	-fsingle-precision-constant \
	-fno-strict-aliasing \
	-Wdouble-promotion \
	-Wno-endif-labels \
	-Wstrict-prototypes \
	-Werror-implicit-function-declaration \
	-Wpointer-arith \
	-Wfloat-equal \
	-Wundef \
	-Wshadow \
	-Wwrite-strings \
	-Wsign-compare \
	-Wmissing-format-attribute \
	-Wno-deprecated-declarations \
	-Wpacked \
	-Wnested-externs \
	-Wunreachable-code \
	-Wcast-align \
	-D__$(CHIP_VARIANT)__ \
	-DUSB_DEVICE_PRODUCT_ID=$(USB_PID) \
	-DUSB_DEVICE_VENDOR_ID=$(USB_VID) \
	-DBOARD=USER_BOARD \
	-ffunction-sections \
	-fdata-sections \
	-fshort-enums \
	--param max-inline-insns-single=500


CFLAGS = $(INC) -Wall -Werror -std=gnu11 -nostdlib $(BASE_CFLAGS) $(COPT)

ifeq ($(CHIP_VARIANT), samd21)
CFLAGS += \
	-mthumb \
	-mabi=aapcs-linux \
	-mcpu=cortex-m0plus \
	-msoft-float \
	-mfloat-abi=soft
else ifeq ($(CHIP_VARIANT), samd51)
CFLAGS += \
	-mthumb \
	-mabi=aapcs-linux \
	-mlong-calls \
	-mcpu=cortex-m4 \
	-mfloat-abi=softfp \
	-mfpu=fpv4-sp-d16
endif

#Debugging/Optimization
ifeq ($(DEBUG), 1)
# NDEBUG disables assert() statements. This reduces code size pretty dramatically, per tannewt.
# Turn on Python modules useful for debugging (e.g. uheap, ustack).
CFLAGS += -Os -ggdb -DNDEBUG
ifeq ($(CHIP_VARIANT), SAMD21)
CFLAGS += -DENABLE_MICRO_TRACE_BUFFER
endif
else
CFLAGS += -Os -DNDEBUG -flto
endif

ifneq ($(FROZEN_DIR),)
# To use frozen source modules, put your .py files in a subdirectory (eg scripts/)
# and then invoke make with FROZEN_DIR=scripts (be sure to build from scratch).
CFLAGS += -DMICROPY_MODULE_FROZEN_STR
CFLAGS += -Wno-error=lto-type-mismatch
endif

ifneq ($(FROZEN_MPY_DIR),)
# To use frozen bytecode, put your .py files in a subdirectory (eg frozen/) and
# then invoke make with FROZEN_MPY_DIR=frozen (be sure to build from scratch).
CFLAGS += -DMICROPY_QSTR_EXTRA_POOL=mp_qstr_frozen_const_pool
CFLAGS += -DMICROPY_MODULE_FROZEN_MPY
CFLAGS += -Wno-error=lto-type-mismatch
endif

#LIBM_FILE_NAME   = $(shell $(CC) $(CFLAGS) -print-file-name=libm.a)
LDFLAGS = $(CFLAGS) -nostartfiles -fshort-enums -Wl,-nostdlib -Wl,-T,$(LD_FILE) -Wl,-Map=$@.map -Wl,-cref -Wl,-gc-sections -specs=nano.specs
LIBS = -lgcc -lc


ifeq ($(CHIP_VARIANT), samd21)
LDFLAGS += -mthumb -mcpu=cortex-m0plus -Lasf/thirdparty/CMSIS/Lib/GCC/
LIBS = -larm_cortexM0l_math -lm $(LIBS)
else  ifeq ($(CHIP_VARIANT), samd51)
LDFLAGS += -mthumb -mcpu=cortex-m4
endif

SRC_ASF = $(addprefix asf4/$(CHIP_FAMILY)/,\
	)

# Skip this source for now.
#	access_vfs.c \
	$(FLASH_IMPL) \

SRC_C = \
	access_vfs.c \
	autoreload.c \
	background.c \
	fatfs_port.c \
	flash_api.c \
	main.c \
	mphalport.c \
	reset.c \
	$(CHIP_FAMILY)_pins.c \
	shared_dma.c \
	rgb_led_status.c \
	tick.c \
	usb.c \
	bindings/samd/__init__.c \
	asf4/$(CHIP_FAMILY)/gcc/gcc/startup_$(CHIP_FAMILY).c \
	asf4/$(CHIP_FAMILY)/gcc/system_$(CHIP_FAMILY).c \
	boards/$(BOARD)/board.c \
	boards/$(BOARD)/pins.c \
	freetouch/adafruit_ptc.c \
	lib/oofatfs/ff.c \
	lib/oofatfs/option/ccsbcs.c \
	lib/timeutils/timeutils.c \
	lib/utils/buffer_helper.c \
	lib/utils/context_manager_helpers.c \
	lib/utils/interrupt_char.c \
	lib/utils/pyexec.c \
	lib/utils/stdout_helpers.c \
	lib/libc/string0.c \
	lib/mp-readline/readline.c

STM_SRC_C = $(addprefix stmhal/,\
	pybstdio.c \
	)

SRC_COMMON_HAL = \
	board/__init__.c \
	microcontroller/__init__.c \
	microcontroller/Pin.c \
	digitalio/__init__.c \
	digitalio/DigitalInOut.c
#	analogio/__init__.c \
	analogio/AnalogIn.c \
	analogio/AnalogOut.c \
	audiobusio/__init__.c \
	audiobusio/PDMIn.c \
	audioio/__init__.c \
	audioio/AudioOut.c \
	busio/__init__.c \
	busio/I2C.c \
	busio/SPI.c \
	busio/UART.c \
	neopixel_write/__init__.c \
	os/__init__.c \
	pulseio/__init__.c \
	pulseio/PulseIn.c \
	pulseio/PulseOut.c \
	pulseio/PWMOut.c \
	storage/__init__.c \
	time/__init__.c \
	touchio/__init__.c \
	touchio/TouchIn.c \
	usb_hid/__init__.c \
	usb_hid/Device.c

# These don't have corresponding files in each port but are still located in
# shared-bindings to make it clear what the contents of the modules are.
SRC_BINDINGS_ENUMS = \
	digitalio/Direction.c \
	digitalio/DriveMode.c \
	digitalio/Pull.c \
	help.c

SRC_COMMON_HAL_EXPANDED = $(addprefix shared-bindings/, $(SRC_COMMON_HAL)) \
                          $(addprefix shared-bindings/, $(SRC_BINDINGS_ENUMS)) \
                          $(addprefix common-hal/, $(SRC_COMMON_HAL))

SRC_SHARED_MODULE = \
	bitbangio/__init__.c \
	bitbangio/I2C.c \
	bitbangio/OneWire.c \
	bitbangio/SPI.c \
	busio/OneWire.c \
	os/__init__.c \
	random/__init__.c \
	storage/__init__.c \
	uheap/__init__.c \
	ustack/__init__.c

SRC_SHARED_MODULE_EXPANDED = $(addprefix shared-bindings/, $(SRC_SHARED_MODULE)) \
                             $(addprefix shared-module/, $(SRC_SHARED_MODULE))

OBJ = $(PY_O) $(addprefix $(BUILD)/, $(SRC_C:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(SRC_ASF:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(STM_SRC_C:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(SRC_COMMON_HAL_EXPANDED:.c=.o))
OBJ += $(addprefix $(BUILD)/, $(SRC_SHARED_MODULE_EXPANDED:.c=.o))

SRC_QSTR += $(SRC_C) $(SRC_COMMON_HAL_EXPANDED) $(SRC_SHARED_MODULE_EXPANDED) $(STM_SRC_C)

all: $(BUILD)/firmware.bin $(BUILD)/firmware.uf2

$(BUILD)/firmware.elf: $(OBJ)
	$(STEPECHO) "LINK $@"
	$(Q)$(CC) -o $@ $(LDFLAGS) $^ -Wl,--start-group $(LIBS) -Wl,--end-group
	$(Q)$(SIZE) $@ | python3 ../tools/build_memory_info.py $(LD_FILE)

$(BUILD)/firmware.bin: $(BUILD)/firmware.elf
	$(ECHO) "Create $@"
	$(Q)$(OBJCOPY) -O binary -j .vectors -j .text -j .data $^ $@

$(BUILD)/firmware.uf2: $(BUILD)/firmware.bin
	$(ECHO) "Create $@"
	python2 ../tools/uf2/utils/uf2conv.py -c -o $@ $^

deploy: $(BUILD)/firmware.bin
	$(ECHO) "Writing $< to the board"
	$(BOSSAC) -u $<

# Run emulation build on a POSIX system with suitable terminal settings
run:
	stty raw opost -echo
	build/firmware.elf
	@echo Resetting terminal...
# This sleep is useful to spot segfaults
	sleep 1
	reset

test: $(BUILD)/firmware.elf
	$(Q)/bin/echo -e "print('hello world!', list(x+1 for x in range(10)), end='eol\\\\n')\\r\\n\\x04" | $(BUILD)/firmware.elf | tail -n2 | grep "^hello world! \\[1, 2, 3, 4, 5, 6, 7, 8, 9, 10\\]eol"

include ../py/mkrules.mk
