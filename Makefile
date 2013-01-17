#
# Top-level Makefile for building PX4 firmware images.
#


#
# Some useful paths.
#
export PX4_BASE		 = $(realpath $(dir $(lastword $(MAKEFILE_LIST))))
export NUTTX_SRC	 = $(PX4_BASE)/nuttx
export NUTTX_APPS	 = $(PX4_BASE)/apps
export MAVLINK_SRC	 = $(PX4_BASE)/mavlink
export ROMFS_SRC	 = $(PX4_BASE)/ROMFS
export IMAGE_DIR	 = $(PX4_BASE)/Images
export BUILD_DIR	 = $(PX4_BASE)/Build
export ARCHIVE_DIR	 = $(PX4_BASE)/Archives

#
# Tools
#
MKFW			 = $(PX4_BASE)/Tools/px_mkfw.py
UPLOADER		 = $(PX4_BASE)/Tools/px_uploader.py
COPY			 = cp
REMOVE			 = rm -f
RMDIR			 = rm -rf

#
# Canned firmware configurations that we build.
#
CONFIGS			?= px4fmu_default px4io_default

#
# If the user has listed a config as a target, strip it out and override CONFIGS
#
EXPLICIT_CONFIGS	:= $(filter $(CONFIGS),$(MAKECMDGOALS))
ifneq ($(EXPLICIT_CONFIGS),)
CONFIGS			:= $(EXPLICIT_CONFIGS)
.PHONY:			$(EXPLICIT_CONFIGS)
$(EXPLICIT_CONFIGS):	all
endif

#
# Platforms (boards) that we build prelink kits for.
#
PLATFORMS		 = px4fmu px4io

#
# Some handy macros
#
PLATFORM_FROM_CONFIG	 = $(word 1,$(subst _, ,$1))

#
# Built products
#
FIRMWARES		 = $(foreach config,$(CONFIGS),$(IMAGE_DIR)/$(config).px4)
BINARIES		 = $(foreach config,$(CONFIGS),$(BUILD_DIR)/$(config).build/firmware.bin)

#
# Debugging
#
MQUIET			 = --no-print-directory
#MQUIET			 = --print-directory
ifeq ($(V),)
Q			 = @
else
Q			 =
endif

all:			$(FIRMWARES)
#all:			$(SYMBOLS)


#
# Generate FIRMWARES from BINARIES
#
$(IMAGE_DIR)/%.px4:			basepath = $(basename $(@))
$(IMAGE_DIR)/%.px4:			config = $(notdir $(basepath))
$(IMAGE_DIR)/%.px4:			platform = $(call PLATFORM_FROM_CONFIG,$(config))
$(FIRMWARES): $(IMAGE_DIR)/%.px4:	$(BUILD_DIR)/%.build/firmware.bin
	@echo %% Generating $@ for $(config)
	$(Q) $(MKFW) --prototype $(IMAGE_DIR)/$(platform).prototype \
		--git_identity $(PX4_BASE) \
		--image $< > $@

#
# Generate the firmware executable
#
# XXX pick the right build tool for the host OS - force it on the generation side too.
#
$(BUILD_DIR)/%.build/firmware.bin: config = $(patsubst $(BUILD_DIR)/%.build/firmware.bin,%,$@)
$(BUILD_DIR)/%.build/firmware.bin: work_dir = $(BUILD_DIR)/$(config).build
$(BINARIES): $(BUILD_DIR)/%.build/firmware.bin:	$(BUILD_DIR)/$(word 1,$(subst _, ,%)).build
	@echo %% Building $(config) in $(work_dir)
	$(Q) make -C $(work_dir) \
		-f $(PX4_BASE)/makefiles/$(config).mk \
		CONFIG=$(config) \
		PLATFORM=$(call PLATFORM_FROM_CONFIG,$(config)) \
		WORK_DIR=$(work_dir)

#
# Generate the config build directory.
#
BUILDAREAS		 = $(foreach config,$(CONFIGS),$(BUILD_DIR)/$(config).build)
.PHONY:			buildareas
buildareas:		$(BUILDAREAS)

$(BUILD_DIR)/%.build:	config = $(notdir $(basename $@))
$(BUILD_DIR)/%.build:	platform = $(call PLATFORM_FROM_CONFIG,$(config))
$(BUILDAREAS): $(BUILD_DIR)/%.build:
	@echo %% Setting up build environment for $(config)
	$(Q) mkdir -p $@
	$(Q) (cd $@ && $(RMDIR) nuttx-export && unzip -q $(ARCHIVE_DIR)/$(platform).export)

#
# Build the NuttX export archives.
#
# Note that there are no explicit dependencies extended from these
# archives. If NuttX is updated, the user is expected to rebuild the 
# archives/build area manually.
#
# XXX Should support fetching/unpacking from a separate directory to permit
#     downloads of the prebuilt archives as well...
#
# XXX PX4IO config name is bad - we should just call them all "px4"
#
NUTTX_ARCHIVES		 = $(foreach platform,$(PLATFORMS),$(ARCHIVE_DIR)/$(platform).export)
.PHONY:			archives
archives:		$(NUTTX_ARCHIVES)

$(ARCHIVE_DIR)/%.export:	platform = $(notdir $(basename $@))
$(ARCHIVE_DIR)/%.export:	config = $(if $(filter $(platform),px4io),io,nsh)
$(NUTTX_ARCHIVES): $(ARCHIVE_DIR)/%.export: $(NUTTX_SRC) $(NUTTX_APPS)
	@echo %% Configuring NuttX for $(platform)
	$(Q) (cd $(NUTTX_SRC) && $(RMDIR) nuttx-export)
	$(Q) make -C $(NUTTX_SRC) -r $(MQUIET) distclean
	$(Q) (cd $(NUTTX_SRC)/tools && ./configure.sh $(platform)/$(config))
	@echo Generating ROMFS for $(platform) XXX move this!
	$(Q) make -C $(ROMFS_SRC) all
	@echo %% Exporting NuttX for $(platform)
	$(Q) make -C $(NUTTX_SRC) -r $(MQUIET) export
	$(Q) mkdir -p $(dir $@)
	$(Q) $(COPY) $(NUTTX_SRC)/nuttx-export.zip $@

setup_px4io:

#
# Firmware upload.
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
SERIAL_PORTS		 = "\\\\.\\COM32,\\\\.\\COM31,\\\\.\\COM30,\\\\.\\COM29,\\\\.\\COM28,\\\\.\\COM27,\\\\.\\COM26,\\\\.\\COM25,\\\\.\\COM24,\\\\.\\COM23,\\\\.\\COM22,\\\\.\\COM21,\\\\.\\COM20,\\\\.\\COM19,\\\\.\\COM18,\\\\.\\COM17,\\\\.\\COM16,\\\\.\\COM15,\\\\.\\COM14,\\\\.\\COM13,\\\\.\\COM12,\\\\.\\COM11,\\\\.\\COM10,\\\\.\\COM9,\\\\.\\COM8,\\\\.\\COM7,\\\\.\\COM6,\\\\.\\COM5,\\\\.\\COM4,\\\\.\\COM3,\\\\.\\COM2,\\\\.\\COM1,\\\\.\\COM0"
endif

upload:		$(FIRMWARE_BUNDLE) $(UPLOADER)
	@python -u $(UPLOADER) --port $(SERIAL_PORTS) $(FIRMWARE_BUNDLE)

#
# JTAG firmware uploading with OpenOCD
#
ifeq ($(JTAGCONFIG),)
JTAGCONFIG=interface/olimex-jtag-tiny.cfg
endif

.PHONY: upload-jtag-px4fmu
upload-jtag-px4fmu: all
	@echo Attempting to flash PX4FMU board via JTAG
	@openocd -f $(JTAGCONFIG) -f ../Bootloader/stm32f4x.cfg -c init -c "reset halt" -c "flash write_image erase nuttx/nuttx" -c "flash write_image erase ../Bootloader/px4fmu_bl.elf" -c "reset run" -c shutdown

.PHONY: upload-jtag-px4io
upload-jtag-px4io: all
	@echo Attempting to flash PX4IO board via JTAG
	@openocd -f $(JTAGCONFIG) -f ../Bootloader/stm32f1x.cfg -c init -c "reset halt" -c "flash write_image erase nuttx/nuttx" -c "flash write_image erase ../Bootloader/px4io_bl.elf" -c "reset run" -c shutdown

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
.PHONY:	clean
clean:
	$(Q) $(RMDIR) $(BUILD_DIR)/*.build
	$(Q) $(REMOVE) -f $(IMAGE_DIR)/*.bin
	$(Q) $(REMOVE) -f $(IMAGE_DIR)/*.sym
	$(Q) $(REMOVE) -f $(IMAGE_DIR)/*.px4
	$(Q) make -C $(ROMFS_SRC) -r $(MQUIET) clean

.PHONY:	distclean
distclean: clean
	$(Q) $(REMOVE) -f $(ARCHIVE_DIR)/*.export
	$(Q) make -C $(NUTTX_SRC) -r $(MQUIET) distclean
	$(Q) make -C $(ROMFS_SRC) -r $(MQUIET) distclean

