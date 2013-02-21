#
# Top-level Makefile for building PX4 firmware images.
#

#
# Get path and tool configuration
#
export PX4_BASE		 := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))
include $(PX4_BASE)/makefiles/setup.mk

#
# Canned firmware configurations that we build.
#
CONFIGS			?= px4fmu_default px4io_default

#
# Boards that we build NuttX export kits for.
#
BOARDS			 = px4fmu px4io

#
# Debugging
#
MQUIET			 = --no-print-directory
#MQUIET			 = --print-directory

################################################################################
# No user-serviceable parts below
################################################################################

#
# If the user has listed a config as a target, strip it out and override CONFIGS.
#
EXPLICIT_CONFIGS	:= $(filter $(CONFIGS),$(MAKECMDGOALS))
ifneq ($(EXPLICIT_CONFIGS),)
CONFIGS			:= $(EXPLICIT_CONFIGS)
.PHONY:			$(EXPLICIT_CONFIGS)
$(EXPLICIT_CONFIGS):	all
endif

#
# Built products
#
STAGED_FIRMWARES	 = $(foreach config,$(CONFIGS),$(IMAGE_DIR)/$(config).px4)
FIRMWARES		 = $(foreach config,$(CONFIGS),$(BUILD_DIR)/$(config).build/firmware.px4)

all:			$(STAGED_FIRMWARES)

#
# Copy FIRMWARES into the image directory.
#
$(STAGED_FIRMWARES): $(IMAGE_DIR)/%.px4: $(BUILD_DIR)/%.build/firmware.px4
	@echo %% Copying $@
	$(Q) $(COPY) $< $@

#
# Generate FIRMWARES.
#
.PHONY: $(FIRMWARES)
$(BUILD_DIR)/%.build/firmware.px4: config   = $(patsubst $(BUILD_DIR)/%.build/firmware.px4,%,$@)
$(BUILD_DIR)/%.build/firmware.px4: work_dir = $(BUILD_DIR)/$(config).build
$(FIRMWARES): $(BUILD_DIR)/%.build/firmware.px4:
	@echo %%%% Building $(config) in $(work_dir)
	$(Q) mkdir -p $(work_dir)
	$(Q) make -C $(work_dir) \
		-f $(PX4_BASE)/makefiles/config_$(config).mk \
		WORK_DIR=$(work_dir) \
		firmware

#
# Build the NuttX export archives.
#
# Note that there are no explicit dependencies extended from these
# archives. If NuttX is updated, the user is expected to rebuild the 
# archives/build area manually. Likewise, when the 'archives' target is
# invoked, all archives are always rebuilt.
#
# XXX Should support fetching/unpacking from a separate directory to permit
#     downloads of the prebuilt archives as well...
#
# XXX PX4IO configuration name is bad - NuttX configs should probably all be "px4"
#
NUTTX_ARCHIVES		 = $(foreach board,$(BOARDS),$(ARCHIVE_DIR)/$(board).export)
.PHONY:			archives
archives:		$(NUTTX_ARCHIVES)

$(ARCHIVE_DIR)/%.export:	board = $(notdir $(basename $@))
$(ARCHIVE_DIR)/%.export:	configuration = $(if $(filter $(board),px4io),io,nsh)
$(NUTTX_ARCHIVES): $(ARCHIVE_DIR)/%.export: $(NUTTX_SRC) $(NUTTX_APPS)
	@echo %% Configuring NuttX for $(board)
	$(Q) (cd $(NUTTX_SRC) && $(RMDIR) nuttx-export)
	$(Q) make -C $(NUTTX_SRC) -r $(MQUIET) distclean
	$(Q) (cd $(NUTTX_SRC)/tools && ./configure.sh $(board)/$(configuration))
	@echo %% Exporting NuttX for $(board)
	$(Q) make -C $(NUTTX_SRC) -r $(MQUIET) export
	$(Q) mkdir -p $(dir $@)
	$(Q) $(COPY) $(NUTTX_SRC)/nuttx-export.zip $@

#
# Cleanup targets.  'clean' should remove all built products and force
# a complete re-compilation, 'distclean' should remove everything 
# that's generated leaving only files that are in source control.
#
.PHONY:	clean
clean:
	$(Q) $(RMDIR) $(BUILD_DIR)/*.build
	$(Q) $(REMOVE) -f $(IMAGE_DIR)/*.px4

.PHONY:	distclean
distclean: clean
	$(Q) $(REMOVE) -f $(ARCHIVE_DIR)/*.export
	$(Q) make -C $(NUTTX_SRC) -r $(MQUIET) distclean

