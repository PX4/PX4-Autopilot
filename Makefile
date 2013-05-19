#
#   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

#
# Top-level Makefile for building PX4 firmware images.
#

#
# Get path and tool configuration
#
export PX4_BASE		 := $(realpath $(dir $(lastword $(MAKEFILE_LIST))))/
include $(PX4_BASE)makefiles/setup.mk

#
# Canned firmware configurations that we build.
#
CONFIGS			?= $(subst config_,,$(basename $(notdir $(wildcard $(PX4_MK_DIR)config_*.mk))))

#
# Boards that we build NuttX export kits for.
#
BOARDS			:= $(subst board_,,$(basename $(notdir $(wildcard $(PX4_MK_DIR)board_*.mk))))

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
FIRMWARE_GOAL		 = firmware
EXPLICIT_CONFIGS	:= $(filter $(CONFIGS),$(MAKECMDGOALS))
ifneq ($(EXPLICIT_CONFIGS),)
CONFIGS			:= $(EXPLICIT_CONFIGS)
.PHONY:			$(EXPLICIT_CONFIGS)
$(EXPLICIT_CONFIGS):	all

#
# If the user has asked to upload, they must have also specified exactly one
# config.
#
ifneq ($(filter upload,$(MAKECMDGOALS)),)
ifneq ($(words $(EXPLICIT_CONFIGS)),1)
$(error In order to upload, exactly one board config must be specified)
endif
FIRMWARE_GOAL		 = upload
.PHONY: upload
upload:
	@:
endif
endif

#
# Built products
#
STAGED_FIRMWARES	 = $(foreach config,$(CONFIGS),$(IMAGE_DIR)$(config).px4)
FIRMWARES		 = $(foreach config,$(CONFIGS),$(BUILD_DIR)$(config).build/firmware.px4)

all:			$(STAGED_FIRMWARES)

#
# Copy FIRMWARES into the image directory.
#
# XXX copying the .bin files is a hack to work around the PX4IO uploader 
#     not supporting .px4 files, and it should be deprecated onced that 
#     is taken care of.
#
$(STAGED_FIRMWARES): $(IMAGE_DIR)%.px4: $(BUILD_DIR)%.build/firmware.px4
	@echo %% Copying $@
	$(Q) $(COPY) $< $@
	$(Q) $(COPY) $(patsubst %.px4,%.bin,$<) $(patsubst %.px4,%.bin,$@)

#
# Generate FIRMWARES.
#
.PHONY: $(FIRMWARES)
$(BUILD_DIR)%.build/firmware.px4: config   = $(patsubst $(BUILD_DIR)%.build/firmware.px4,%,$@)
$(BUILD_DIR)%.build/firmware.px4: work_dir = $(BUILD_DIR)$(config).build/
$(FIRMWARES): $(BUILD_DIR)%.build/firmware.px4:
	@echo %%%%
	@echo %%%% Building $(config) in $(work_dir)
	@echo %%%%
	$(Q) mkdir -p $(work_dir)
	$(Q) make -r -C $(work_dir) \
		-f $(PX4_MK_DIR)firmware.mk \
		CONFIG=$(config) \
		WORK_DIR=$(work_dir) \
		$(FIRMWARE_GOAL)

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
NUTTX_ARCHIVES		 = $(foreach board,$(BOARDS),$(ARCHIVE_DIR)$(board).export)
.PHONY:			archives
archives:		$(NUTTX_ARCHIVES)

# We cannot build these parallel; note that we also force -j1 for the
# sub-make invocations.
ifneq ($(filter archives,$(MAKECMDGOALS)),)
.NOTPARALLEL:
endif

$(ARCHIVE_DIR)%.export:	board = $(notdir $(basename $@))
$(ARCHIVE_DIR)%.export:	configuration = $(if $(filter $(board),px4io),io,nsh)
$(NUTTX_ARCHIVES): $(ARCHIVE_DIR)%.export: $(NUTTX_SRC) $(NUTTX_APPS)
	@echo %% Configuring NuttX for $(board)
	$(Q) (cd $(NUTTX_SRC) && $(RMDIR) nuttx-export)
	$(Q) make -r -j1 -C $(NUTTX_SRC) -r $(MQUIET) distclean
	$(Q) (cd $(NUTTX_SRC)tools && ./configure.sh $(board)/$(configuration))
	@echo %% Exporting NuttX for $(board)
	$(Q) make -r -j1 -C $(NUTTX_SRC) -r $(MQUIET) export
	$(Q) mkdir -p $(dir $@)
	$(Q) $(COPY) $(NUTTX_SRC)nuttx-export.zip $@

#
# Cleanup targets.  'clean' should remove all built products and force
# a complete re-compilation, 'distclean' should remove everything 
# that's generated leaving only files that are in source control.
#
.PHONY:	clean
clean:
	$(Q) $(RMDIR) $(BUILD_DIR)*.build
	$(Q) $(REMOVE) $(IMAGE_DIR)*.px4

.PHONY:	distclean
distclean: clean
	$(Q) $(REMOVE) $(ARCHIVE_DIR)*.export
	$(Q) make -C $(NUTTX_SRC) -r $(MQUIET) distclean

#
# Print some help text
#
.PHONY: help
help:
	@echo ""
	@echo " PX4 firmware builder"
	@echo " ===================="
	@echo ""
	@echo "  Available targets:"
	@echo "  ------------------"
	@echo ""
	@echo "  archives"
	@echo "    Build the NuttX RTOS archives that are used by the firmware build."
	@echo ""
	@echo "  all"
	@echo "    Build all firmware configs: $(CONFIGS)"
	@echo "    A limited set of configs can be built with CONFIGS=<list-of-configs>"
	@echo ""
	@for config in $(CONFIGS); do \
		echo "  $$config"; \
		echo "    Build just the $$config firmware configuration."; \
		echo ""; \
	done
	@echo "  clean"
	@echo "    Remove all firmware build pieces."
	@echo ""
	@echo "  distclean"
	@echo "    Remove all compilation products, including NuttX RTOS archives."
	@echo ""
	@echo "  upload"
	@echo "    When exactly one config is being built, add this target to upload the"
	@echo "    firmware to the board when the build is complete. Not supported for"
	@echo "    all configurations."
	@echo ""
	@echo "  Common options:"
	@echo "  ---------------"
	@echo ""
	@echo "  V=1"
	@echo "    If V is set, more verbose output is printed during the build. This can"
	@echo "    help when diagnosing issues with the build or toolchain."
	@echo ""
