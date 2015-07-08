#
# Rules and tools for uploading firmware to various PX4 boards.
#

UPLOADER		 = $(PX4_BASE)/Tools/px_uploader.py

SYSTYPE			:= $(shell uname -s)

#
# Serial port defaults.
#
# XXX The uploader should be smarter than this.
#
ifeq ($(SYSTYPE),Darwin)
SERIAL_PORTS		?= "/dev/tty.usbmodemPX*,/dev/tty.usbmodem*"
endif
ifeq ($(SYSTYPE),Linux)
SERIAL_PORTS		?= "/dev/serial/by-id/usb-3D_Robotics*,/dev/serial/by-id/pci-3D_Robotics*"
endif
ifeq ($(SERIAL_PORTS),)
SERIAL_PORTS		 = "COM32,COM31,COM30,COM29,COM28,COM27,COM26,COM25,COM24,COM23,COM22,COM21,COM20,COM19,COM18,COM17,COM16,COM15,COM14,COM13,COM12,COM11,COM10,COM9,COM8,COM7,COM6,COM5,COM4,COM3,COM2,COM1,COM0"
endif

.PHONY:	all upload-$(METHOD)-$(BOARD)
all:	upload-$(METHOD)-$(BOARD)

upload-serial-px4fmu-v1:	$(BUNDLE) $(UPLOADER)
	$(Q) $(PYTHON) -u $(UPLOADER) --port $(SERIAL_PORTS) $(BUNDLE)

upload-serial-px4fmu-v2:	$(BUNDLE) $(UPLOADER)
	$(Q) $(PYTHON) -u $(UPLOADER) --port $(SERIAL_PORTS) $(BUNDLE)

upload-serial-aerocore:
	openocd -f $(PX4_BASE)/makefiles/gumstix-aerocore.cfg -c 'init; reset halt; flash write_image erase $(PX4_BASE)/../Bootloader/px4aerocore_bl.bin 0x08000000; flash write_image erase $(PX4_BASE)/Build/aerocore_default.build/firmware.bin 0x08004000; reset run; exit'

upload-serial-px4-stm32f4discovery:	$(BUNDLE) $(UPLOADER)
	$(Q) $(PYTHON) -u $(UPLOADER) --port $(SERIAL_PORTS) $(BUNDLE)

#
# JTAG firmware uploading with OpenOCD
#
JTAGCONFIG		?= interface/olimex-jtag-tiny.cfg

upload-jtag-px4fmu: all
	@$(ECHO) Attempting to flash PX4FMU board via JTAG
	$(Q) $(OPENOCD) -f $(JTAGCONFIG) -f ../Bootloader/stm32f4x.cfg -c init -c "reset halt" -c "flash write_image erase nuttx/nuttx" -c "flash write_image erase ../Bootloader/px4fmu_bl.elf" -c "reset run" -c shutdown

upload-jtag-px4io: all
	@$(ECHO) Attempting to flash PX4IO board via JTAG
	$(Q) $(OPENOCD) -f $(JTAGCONFIG) -f ../Bootloader/stm32f1x.cfg -c init -c "reset halt" -c "flash write_image erase nuttx/nuttx" -c "flash write_image erase ../Bootloader/px4io_bl.elf" -c "reset run" -c shutdown
