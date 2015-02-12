#
#   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
# Get a version string provided by git
# This assumes that git command is available and that
# the directory holding this file also contains .git directory
#
GIT_DESC := $(shell git log -1 --pretty=format:%H)
ifneq ($(words $(GIT_DESC)),1)
    GIT_DESC := "unknown_git_version"
endif
export GIT_DESC

#
# Canned firmware configurations that we (know how to) build.
#
KNOWN_CONFIGS		:= $(subst config_,,$(basename $(notdir $(wildcard $(PX4_MK_DIR)config_*.mk))))
CONFIGS			?= $(KNOWN_CONFIGS)

#
# Boards that we (know how to) build NuttX export kits for.
#
KNOWN_BOARDS		:= $(subst board_,,$(basename $(notdir $(wildcard $(PX4_MK_DIR)board_*.mk))))
BOARDS			?= $(KNOWN_BOARDS)

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
DESIRED_FIRMWARES 	 = $(foreach config,$(CONFIGS),$(IMAGE_DIR)$(config).px4)
STAGED_FIRMWARES	 = $(foreach config,$(KNOWN_CONFIGS),$(IMAGE_DIR)$(config).px4)
FIRMWARES		 = $(foreach config,$(KNOWN_CONFIGS),$(BUILD_DIR)$(config).build/firmware.px4)

all:	$(DESIRED_FIRMWARES)

#
# Copy FIRMWARES into the image directory.
#
# XXX copying the .bin files is a hack to work around the PX4IO uploader
#     not supporting .px4 files, and it should be deprecated onced that
#     is taken care of.
#
$(STAGED_FIRMWARES): $(IMAGE_DIR)%.px4: $(BUILD_DIR)%.build/firmware.px4
	@$(ECHO) %% Copying $@
	$(Q) $(COPY) $< $@
	$(Q) $(COPY) $(patsubst %.px4,%.bin,$<) $(patsubst %.px4,%.bin,$@)

#
# Generate FIRMWARES.
#
.PHONY: $(FIRMWARES)
$(BUILD_DIR)%.build/firmware.px4: config   = $(patsubst $(BUILD_DIR)%.build/firmware.px4,%,$@)
$(BUILD_DIR)%.build/firmware.px4: work_dir = $(BUILD_DIR)$(config).build/
$(FIRMWARES): $(BUILD_DIR)%.build/firmware.px4:	checksubmodules generateuorbtopicheaders
	@$(ECHO) %%%%
	@$(ECHO) %%%% Building $(config) in $(work_dir)
	@$(ECHO) %%%%
	$(Q) $(MKDIR) -p $(work_dir)
	$(Q) $(MAKE) -r -C $(work_dir) \
		-f $(PX4_MK_DIR)firmware.mk \
		CONFIG=$(config) \
		WORK_DIR=$(work_dir) \
		$(FIRMWARE_GOAL)

#
# Make FMU firmwares depend on the corresponding IO firmware.
#
# This is a pretty vile hack, since it hard-codes knowledge of the FMU->IO dependency
# and forces the _default config in all cases. There has to be a better way to do this...
#
FMU_VERSION		 = $(patsubst px4fmu-%,%,$(word 1, $(subst _, ,$(1))))
define FMU_DEP
$(BUILD_DIR)$(1).build/firmware.px4: $(IMAGE_DIR)px4io-$(call FMU_VERSION,$(1))_default.px4
endef
FMU_CONFIGS		:= $(filter px4fmu%,$(CONFIGS))
$(foreach config,$(FMU_CONFIGS),$(eval $(call FMU_DEP,$(config))))

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
NUTTX_ARCHIVES		 = $(foreach board,$(BOARDS),$(ARCHIVE_DIR)$(board).export)
.PHONY:			archives
archives:		checksubmodules $(NUTTX_ARCHIVES)

# We cannot build these parallel; note that we also force -j1 for the
# sub-make invocations.
ifneq ($(filter archives,$(MAKECMDGOALS)),)
.NOTPARALLEL:
endif

J?=1

$(ARCHIVE_DIR)%.export:	board = $(notdir $(basename $@))
$(ARCHIVE_DIR)%.export:	configuration = nsh
$(NUTTX_ARCHIVES): $(ARCHIVE_DIR)%.export: $(NUTTX_SRC)
	@$(ECHO) %% Configuring NuttX for $(board)
	$(Q) (cd $(NUTTX_SRC) && $(RMDIR) nuttx-export)
	$(Q) $(MAKE) -r -j$(J) -C $(NUTTX_SRC) -r $(MQUIET) distclean
	$(Q) (cd $(NUTTX_SRC)/configs && $(COPYDIR) $(PX4_BASE)nuttx-configs/$(board) .)
	$(Q) (cd $(NUTTX_SRC)tools && ./configure.sh $(board)/$(configuration))
	@$(ECHO) %% Exporting NuttX for $(board)
	$(Q) $(MAKE) -r -j$(J) -C $(NUTTX_SRC) -r $(MQUIET) CONFIG_ARCH_BOARD=$(board) export
	$(Q) $(MKDIR) -p $(dir $@)
	$(Q) $(COPY) $(NUTTX_SRC)nuttx-export.zip $@
	$(Q) (cd $(NUTTX_SRC)/configs && $(RMDIR) $(board))

#
# The user can run the NuttX 'menuconfig' tool for a single board configuration with
# make BOARDS=<boardname> menuconfig
#
ifeq ($(MAKECMDGOALS),menuconfig)
ifneq ($(words $(BOARDS)),1)
$(error BOARDS must specify exactly one board for the menuconfig goal)
endif
BOARD			 = $(BOARDS)
menuconfig: $(NUTTX_SRC)
	@$(ECHO) %% Configuring NuttX for $(BOARD)
	$(Q) (cd $(NUTTX_SRC) && $(RMDIR) nuttx-export)
	$(Q) $(MAKE) -r -j$(J) -C $(NUTTX_SRC) -r $(MQUIET) distclean
	$(Q) (cd $(NUTTX_SRC)/configs && $(COPYDIR) $(PX4_BASE)nuttx-configs/$(BOARD) .)
	$(Q) (cd $(NUTTX_SRC)tools && ./configure.sh $(BOARD)/nsh)
	@$(ECHO) %% Running menuconfig for $(BOARD)
	$(Q) $(MAKE) -r -j$(J) -C $(NUTTX_SRC) -r $(MQUIET) menuconfig
	@$(ECHO) %% Saving configuration file
	$(Q)$(COPY) $(NUTTX_SRC).config $(PX4_BASE)nuttx-configs/$(BOARD)/nsh/defconfig
else
menuconfig:
	@$(ECHO) ""
	@$(ECHO) "The menuconfig goal must be invoked without any other goal being specified"
	@$(ECHO) ""
endif

$(NUTTX_SRC): checksubmodules

$(UAVCAN_DIR):
	$(Q) (./Tools/check_submodules.sh)

.PHONY: checksubmodules
checksubmodules:
	$(Q) ($(PX4_BASE)/Tools/check_submodules.sh)

.PHONY: updatesubmodules
updatesubmodules:
	$(Q) (git submodule init)
	$(Q) (git submodule update)

MSG_DIR = $(PX4_BASE)msg
UORB_TEMPLATE_DIR = $(PX4_BASE)msg/templates/uorb
MULTIPLATFORM_TEMPLATE_DIR = $(PX4_BASE)msg/templates/px4/uorb
TOPICS_DIR = $(PX4_BASE)src/modules/uORB/topics
MULTIPLATFORM_HEADER_DIR = $(PX4_BASE)src/platforms/nuttx/px4_messages
MULTIPLATFORM_PREFIX = px4_
TOPICHEADER_TEMP_DIR = $(BUILD_DIR)topics_temporary
GENMSG_PYTHONPATH = $(PX4_BASE)Tools/genmsg/src
GENCPP_PYTHONPATH = $(PX4_BASE)Tools/gencpp/src

.PHONY: generateuorbtopicheaders
generateuorbtopicheaders:
	@$(ECHO) "Generating uORB topic headers"
	$(Q) (PYTHONPATH=$(GENMSG_PYTHONPATH):$(GENCPP_PYTHONPATH) $(PYTHON) \
		$(PX4_BASE)Tools/px_generate_uorb_topic_headers.py \
		-d $(MSG_DIR) -o $(TOPICS_DIR) -e $(UORB_TEMPLATE_DIR) -t $(TOPICHEADER_TEMP_DIR))
	@$(ECHO) "Generating multiplatform uORB topic wrapper headers"
	$(Q) (PYTHONPATH=$(GENMSG_PYTHONPATH):$(GENCPP_PYTHONPATH) $(PYTHON) \
		$(PX4_BASE)Tools/px_generate_uorb_topic_headers.py \
		-d $(MSG_DIR) -o $(MULTIPLATFORM_HEADER_DIR) -e $(MULTIPLATFORM_TEMPLATE_DIR) -t $(TOPICHEADER_TEMP_DIR) -p $(MULTIPLATFORM_PREFIX))
# clean up temporary files
	$(Q) (rm -r $(TOPICHEADER_TEMP_DIR))

#
# Testing targets
#
testbuild:
	$(Q) (cd $(PX4_BASE) && $(MAKE) distclean && $(MAKE) archives && $(MAKE) -j8)
	$(Q) (zip -r Firmware.zip $(PX4_BASE)/Images)

#
# Unittest targets. Builds and runs the host-level
# unit tests.
.PHONY: tests
tests:	generateuorbtopicheaders
	$(Q) (mkdir -p $(PX4_BASE)/unittests/build && cd $(PX4_BASE)/unittests/build && cmake .. && $(MAKE) unittests)

#
# Cleanup targets.  'clean' should remove all built products and force
# a complete re-compilation, 'distclean' should remove everything
# that's generated leaving only files that are in source control.
#
.PHONY:	clean
clean:
	@echo > /dev/null
	$(Q) $(RMDIR) $(BUILD_DIR)*.build
	$(Q) $(REMOVE) $(IMAGE_DIR)*.px4

.PHONY:	distclean
distclean: clean
	@echo > /dev/null
	$(Q) $(REMOVE) $(ARCHIVE_DIR)*.export
	$(Q) $(MAKE) -C $(NUTTX_SRC) -r $(MQUIET) distclean
	$(Q) (cd $(NUTTX_SRC)/configs && $(FIND) . -maxdepth 1 -type l -delete)

#
# Print some help text
#
.PHONY: help
help:
	@$(ECHO) ""
	@$(ECHO) " PX4 firmware builder"
	@$(ECHO) " ===================="
	@$(ECHO) ""
	@$(ECHO) "  Available targets:"
	@$(ECHO) "  ------------------"
	@$(ECHO) ""
	@$(ECHO) "  archives"
	@$(ECHO) "    Build the NuttX RTOS archives that are used by the firmware build."
	@$(ECHO) ""
	@$(ECHO) "  all"
	@$(ECHO) "    Build all firmware configs: $(CONFIGS)"
	@$(ECHO) "    A limited set of configs can be built with CONFIGS=<list-of-configs>"
	@$(ECHO) ""
	@for config in $(CONFIGS); do \
		$(ECHO) "  $$config"; \
		$(ECHO) "    Build just the $$config firmware configuration."; \
		$(ECHO) ""; \
	done
	@$(ECHO) "  clean"
	@$(ECHO) "    Remove all firmware build pieces."
	@$(ECHO) ""
	@$(ECHO) "  distclean"
	@$(ECHO) "    Remove all compilation products, including NuttX RTOS archives."
	@$(ECHO) ""
	@$(ECHO) "  upload"
	@$(ECHO) "    When exactly one config is being built, add this target to upload the"
	@$(ECHO) "    firmware to the board when the build is complete. Not supported for"
	@$(ECHO) "    all configurations."
	@$(ECHO) ""
	@$(ECHO) "  testbuild"
	@$(ECHO) "    Perform a complete clean build of the entire tree."
	@$(ECHO) ""
	@$(ECHO) "  Common options:"
	@$(ECHO) "  ---------------"
	@$(ECHO) ""
	@$(ECHO) "  V=1"
	@$(ECHO) "    If V is set, more verbose output is printed during the build. This can"
	@$(ECHO) "    help when diagnosing issues with the build or toolchain."
	@$(ECHO) ""
