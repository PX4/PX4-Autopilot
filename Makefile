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
STAGED_FIRMWARES	 = $(foreach config,$(CONFIGS),$(IMAGE_DIR)/$(config).px4)
FIRMWARES		 = $(foreach config,$(CONFIGS),$(BUILD_DIR)/$(config).build/firmware.px4)

#
# Debugging
#
MQUIET			 = --no-print-directory
#MQUIET			 = --print-directory
Q			:= $(if $(V),,@)

all:			$(STAGED_FIRMWARES)

#
# Copy FIRMWARES into the image directory.
#
$(STAGED_FIRMWARES): $(IMAGE_DIR)/%.px4: $(BUILD_DIR)/%.build/firmware.px4
	@echo %% Copying $@
	$(Q) $(COPY) $< $@

#
# Generate FIRMWARES
#
$(BUILD_DIR)/%.build/firmware.px4: config   = $(patsubst $(BUILD_DIR)/%.build/firmware.px4,%,$@)
$(BUILD_DIR)/%.build/firmware.px4: work_dir = $(BUILD_DIR)/$(config).build
$(FIRMWARES): $(BUILD_DIR)/%.build/firmware.px4:
	@echo %%%% Building $(config) in $(work_dir)
	$(Q) mkdir -p $(work_dir)
	$(Q) make -C $(work_dir) \
		-f $(PX4_BASE)/makefiles/$(config).mk \
		CONFIG=$(config) \
		PLATFORM=$(call PLATFORM_FROM_CONFIG,$(config)) \
		WORK_DIR=$(work_dir)

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

#
# Cleanup targets.  'clean' should remove all built products and force
# a complete re-compilation, 'distclean' should remove everything 
# that's generated leaving only files that are in source control.
#
.PHONY:	clean
clean:
	$(Q) $(RMDIR) $(BUILD_DIR)/*.build
	$(Q) $(REMOVE) -f $(IMAGE_DIR)/*.px4
	$(Q) make -C $(ROMFS_SRC) -r $(MQUIET) clean

.PHONY:	distclean
distclean: clean
	$(Q) $(REMOVE) -f $(ARCHIVE_DIR)/*.export
	$(Q) make -C $(NUTTX_SRC) -r $(MQUIET) distclean
	$(Q) make -C $(ROMFS_SRC) -r $(MQUIET) distclean

