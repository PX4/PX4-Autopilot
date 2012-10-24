#
# Top-level Makefile for building PX4 firmware images.
#
#
# Note that this is a transitional process; the eventual goal is for this
# project to slim down and simply generate PX4 link kits via the NuttX
# 'make export' mechanism.
#
#

#
# Some useful paths.
#
export PX4BASE		 = $(realpath $(dir $(lastword $(MAKEFILE_LIST))))
export NUTTX_SRC	 = $(PX4BASE)/nuttx
export NUTTX_APPS	 = $(PX4BASE)/apps
export MAVLINK_SRC	 = $(PX4BASE)/mavlink
export ROMFS_SRC	 = $(PX4BASE)/ROMFS
export IMAGE_DIR	 = $(PX4BASE)/Images

#
# Tools
#
MKFW			 = $(PX4BASE)/Tools/px_mkfw.py
UPLOADER		 = $(PX4BASE)/Tools/px_uploader.py

#
# What are we currently configured for?
#
CONFIGURED		 = $(PX4BASE)/.configured
ifeq ($(wildcard $(CONFIGURED)),)
# the $(CONFIGURED) target will make this a reality before building
export TARGET		 = px4fmu
$(shell echo $(TARGET) > $(CONFIGURED))
else
export TARGET		 = $(shell cat $(CONFIGURED))
endif

#
# What we will build
#
FIRMWARE_BUNDLE		 = $(IMAGE_DIR)/$(TARGET).px4
FIRMWARE_BINARY		 = $(IMAGE_DIR)/$(TARGET).bin
FIRMWARE_PROTOTYPE	 = $(IMAGE_DIR)/$(TARGET).prototype

#
# Debugging
#
MQUIET			 = --no-print-directory
#MQUIET			 = --print-directory

all:			$(FIRMWARE_BUNDLE)

#
# Generate a wrapped .px4 file from the built binary
#
$(FIRMWARE_BUNDLE):	$(FIRMWARE_BINARY) $(MKFW) $(FIRMWARE_PROTOTYPE)
	@echo Generating $@
	@$(MKFW) --prototype $(FIRMWARE_PROTOTYPE) \
		--git_identity $(PX4BASE) \
		--image $(FIRMWARE_BINARY) > $@
#
# Build the firmware binary.
#
.PHONY:			$(FIRMWARE_BINARY)
$(FIRMWARE_BINARY):	configure_$(TARGET) setup_$(TARGET)
	@echo Building $@
	@make -C $(NUTTX_SRC) -r $(MQUIET) all
	@cp $(NUTTX_SRC)/nuttx.bin $@

#
# The 'configure' targets select one particular firmware configuration
# and makes it current.
#
configure_px4fmu:
ifneq ($(TARGET),px4fmu)
	@make -C $(PX4BASE) distclean
endif
	@cd $(NUTTX_SRC)/tools && /bin/sh configure.sh px4fmu/nsh
	@echo px4fmu > $(CONFIGURED)

configure_px4io:
ifneq ($(TARGET),px4io)
	@make -C $(PX4BASE) distclean
endif
	@cd $(NUTTX_SRC)/tools && /bin/sh configure.sh px4io/io
	@echo px4io > $(CONFIGURED)

#
# Per-configuration additional targets
#
.PHONY:			px4fmu_setup
setup_px4fmu:
	@echo Generating ROMFS
	@make -C $(ROMFS_SRC) all

setup_px4io:

#
# Firmware uploading.
#

# serial port defaults by operating system.
SYSTYPE			 = $(shell uname)
ifeq ($(SYSTYPE),Darwin)
SERIAL_PORTS		?= "/dev/tty.usbmodemPX1,/dev/tty.usbmodemPX2,/dev/tty.usbmodemPX3,/dev/tty.usbmodemPX4,/dev/tty.usbmodem1,/dev/tty.usbmodem2,/dev/tty.usbmodem3,/dev/tty.usbmodem4"
endif
ifeq ($(SYSTYPE),Linux)
SERIAL_PORTS		?= "/dev/ttyACM5,/dev/ttyACM4,/dev/ttyACM3,/dev/ttyACM2,/dev/ttyACM1,/dev/ttyACM0"
endif
ifeq ($(SERIAL_PORTS),)
SERIAL_PORTS		 = "\\\\.\\COM18,\\\\.\\COM17,\\\\.\\COM16,\\\\.\\COM15,\\\\.\\COM14,\\\\.\\COM13,\\\\.\\COM12,\\\\.\\COM11,\\\\.\\COM10,\\\\.\\COM9,\\\\.\\COM8,\\\\.\\COM7,\\\\.\\COM6,\\\\.\\COM5,\\\\.\\COM4,\\\\.\\COM3,\\\\.\\COM2,\\\\.\\COM1,\\\\.\\COM0"
endif

upload:		$(FIRMWARE_BUNDLE) $(UPLOADER)
	@python -u $(UPLOADER) --port $(SERIAL_PORTS) $(FIRMWARE_BUNDLE)
	
upload-jtag-px4fmu:
	@echo Attempting to flash PX4FMU board via JTAG
	@openocd -f interface/olimex-jtag-tiny.cfg -f ../Bootloader/stm32f4x.cfg -c init -c "reset halt" -c "flash write_image erase nuttx/nuttx" -c "flash write_image erase ../Bootloader/px4fmu_bl.elf" -c "reset run" -c shutdown

#
# Hacks and fixups
#

ifeq ($(SYSTYPE),Darwin)
# PATH inherited by Eclipse may not include toolchain install location 
export PATH			 := $(PATH):/usr/local/bin
endif

#
# Cleanup targets.  'clean' should remove all built products and force
# a complete re-compilation, 'distclean' should remove everything 
# that's generated leaving only files that are in source control.
#
.PHONY:	clean upload-jtag-px4fmu
clean:
	@make -C $(NUTTX_SRC) -r $(MQUIET) distclean
	@make -C $(ROMFS_SRC) -r $(MQUIET) clean

.PHONY:	distclean
distclean:
	@rm -f $(CONFIGURED)
	@make -C $(NUTTX_SRC) -r $(MQUIET) distclean
	@make -C $(ROMFS_SRC) -r $(MQUIET) distclean

