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

TARGETS	:= nuttx posix posix-arm qurt
EXPLICIT_TARGET	:= $(filter $(TARGETS),$(MAKECMDGOALS))
ifneq ($(EXPLICIT_TARGET),)
    export PX4_TARGET_OS=$(EXPLICIT_TARGET)
    export GOALS := $(wordlist 2,$(words $(MAKECMDGOALS)),$(MAKECMDGOALS))
endif

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

GIT_DESC_SHORT := $(shell echo $(GIT_DESC) | cut -c1-16)

#
# Canned firmware configurations that we (know how to) build.
#
KNOWN_CONFIGS		:= $(subst config_,,$(basename $(notdir $(wildcard $(PX4_MK_DIR)/$(PX4_TARGET_OS)/config_*.mk))))
CONFIGS			?= $(KNOWN_CONFIGS)

#
# Boards that we (know how to) build NuttX export kits for.
#
KNOWN_BOARDS		:= $(subst board_,,$(basename $(notdir $(wildcard $(PX4_MK_DIR)/$(PX4_TARGET_OS)/board_*.mk))))
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

ifeq ($(PX4_TARGET_OS),nuttx)
include $(PX4_BASE)makefiles/nuttx/firmware_nuttx.mk
endif
ifeq ($(PX4_TARGET_OS),posix)
include $(PX4_BASE)makefiles/posix/firmware_posix.mk
endif
ifeq ($(PX4_TARGET_OS),posix-arm)
include $(PX4_BASE)makefiles/posix/firmware_posix.mk
endif
ifeq ($(PX4_TARGET_OS),qurt)
include $(PX4_BASE)makefiles/qurt/firmware_qurt.mk
endif

qurt_fixup:
	makefiles/qurt/setup.sh $(PX4_BASE)

restore:
	cd src/lib/eigen && git checkout .
	git submodule update
	
#
# Versioning
#

GIT_VER_FILE = $(PX4_VERSIONING_DIR).build_git_ver
GIT_HEADER_FILE = $(PX4_VERSIONING_DIR)build_git_version.h

$(GIT_VER_FILE) :
	$(Q) if [ ! -f $(GIT_VER_FILE) ]; then \
		$(MKDIR) -p $(PX4_VERSIONING_DIR); \
		$(ECHO) "" > $(GIT_VER_FILE); \
	fi

.PHONY: checkgitversion
checkgitversion: $(GIT_VER_FILE)
	$(Q) if [ "$(GIT_DESC)" != "$(shell cat $(GIT_VER_FILE))" ]; then \
		$(ECHO) "/* Auto Magically Generated file */" > $(GIT_HEADER_FILE); \
		$(ECHO) "/* Do not edit! */" >> $(GIT_HEADER_FILE); \
		$(ECHO) "#define PX4_GIT_VERSION_STR  \"$(GIT_DESC)\"" >> $(GIT_HEADER_FILE); \
		$(ECHO) "#define PX4_GIT_VERSION_BINARY 0x$(GIT_DESC_SHORT)" >> $(GIT_HEADER_FILE); \
		$(ECHO) $(GIT_DESC) > $(GIT_VER_FILE); \
	fi
#
# Sizes
#

.PHONY: size
size:
	$(Q) for elfs in Build/*; do if [ -f  $$elfs/firmware.elf ]; then  $(SIZE) $$elfs/firmware.elf; fi done


#
# Submodule Checks
#

.PHONY: checksubmodules
checksubmodules:
	$(Q) ($(PX4_BASE)/Tools/check_submodules.sh $(PX4_TARGET_OS))

.PHONY: updatesubmodules
updatesubmodules:
	$(Q) (git submodule init)
	$(Q) (git submodule update)

MSG_DIR = $(PX4_BASE)msg
UORB_TEMPLATE_DIR = $(PX4_BASE)msg/templates/uorb
MULTIPLATFORM_TEMPLATE_DIR = $(PX4_BASE)msg/templates/px4/uorb
TOPICS_DIR = $(PX4_BASE)src/modules/uORB/topics
MULTIPLATFORM_HEADER_DIR = $(PX4_BASE)src/platforms/$(PX4_TARGET_OS)/px4_messages
MULTIPLATFORM_PREFIX = px4_
TOPICHEADER_TEMP_DIR = $(BUILD_DIR)topics_temporary
MULTI_TOPICHEADER_TEMP_DIR = $(BUILD_DIR)multi_topics_temporary
GENMSG_PYTHONPATH = $(PX4_BASE)Tools/genmsg/src
GENCPP_PYTHONPATH = $(PX4_BASE)Tools/gencpp/src

.PHONY: generateuorbtopicheaders
generateuorbtopicheaders: checksubmodules
	@$(ECHO) "Generating uORB topic headers"
	$(Q) (PYTHONPATH=$(GENMSG_PYTHONPATH):$(GENCPP_PYTHONPATH):$(PYTHONPATH) $(PYTHON) \
		$(PX4_BASE)Tools/px_generate_uorb_topic_headers.py \
		-d $(MSG_DIR) -o $(TOPICS_DIR) -e $(UORB_TEMPLATE_DIR) -t $(TOPICHEADER_TEMP_DIR))
	@$(ECHO) "Generating multiplatform uORB topic wrapper headers"
	$(Q) (PYTHONPATH=$(GENMSG_PYTHONPATH):$(GENCPP_PYTHONPATH):$(PYTHONPATH) $(PYTHON) \
		$(PX4_BASE)Tools/px_generate_uorb_topic_headers.py \
		-d $(MSG_DIR) -o $(MULTIPLATFORM_HEADER_DIR) -e $(MULTIPLATFORM_TEMPLATE_DIR) -t $(MULTI_TOPICHEADER_TEMP_DIR) -p $(MULTIPLATFORM_PREFIX))

#
# Testing targets
#
testbuild:
	$(Q) (cd $(PX4_BASE) && $(MAKE) distclean && $(MAKE) archives && $(MAKE) -j8)
	$(Q) (zip -r Firmware.zip $(PX4_BASE)/Images)

nuttx posix posix-arm qurt: 
ifeq ($(GOALS),)
	$(MAKE) PX4_TARGET_OS=$@ $(GOALS)
else
	export PX4_TARGET_OS=$@
endif

sitl_quad:
	$(Q) Tools/sitl_run.sh posix-configs/SITL/init/rcS
sitl_quad_gazebo:
	$(Q) Tools/sitl_run.sh posix-configs/SITL/init/rc_iris_ros
sitl_plane:
	$(Q) Tools/sitl_run.sh posix-configs/SITL/init/rc.fixed_wing

qurtrun:
	$(MAKE) PX4_TARGET_OS=qurt sim

#
# Unittest targets. Builds and runs the host-level
# unit tests.
.PHONY: tests
tests:	generateuorbtopicheaders
	$(Q) (mkdir -p $(PX4_BASE)/unittests/build && cd $(PX4_BASE)/unittests/build && cmake .. && $(MAKE) unittests)

.PHONY: format check_format
check_format:
	$(Q) (./Tools/check_code_style.sh | sort -n)

#
# Cleanup targets.  'clean' should remove all built products and force
# a complete re-compilation, 'distclean' should remove everything
# that's generated leaving only files that are in source control.
#
.PHONY:	clean
clean:
	@echo > /dev/null
	$(Q) $(RMDIR) $(BUILD_DIR)*.build
	$(Q) $(RMDIR) $(PX4_VERSIONING_DIR)
	$(Q) $(REMOVE) $(IMAGE_DIR)*.px4
	$(Q) $(RMDIR) $(TOPICHEADER_TEMP_DIR)
	$(Q) $(RMDIR) $(MULTI_TOPICHEADER_TEMP_DIR)

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
ifeq ($(PX4_TARGET_OS),nuttx)
	@$(ECHO) "  archives"
	@$(ECHO) "    Build the NuttX RTOS archives that are used by the firmware build."
	@$(ECHO) ""
endif
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
ifeq ($(PX4_TARGET_OS),nuttx)
	@$(ECHO) "  distclean"
	@$(ECHO) "    Remove all compilation products, including NuttX RTOS archives."
	@$(ECHO) ""
	@$(ECHO) "  upload"
	@$(ECHO) "    When exactly one config is being built, add this target to upload the"
	@$(ECHO) "    firmware to the board when the build is complete. Not supported for"
	@$(ECHO) "    all configurations."
	@$(ECHO) ""
endif
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
ifeq ($(PX4_TARGET_OS),nuttx)
	@$(ECHO) "  To see help for a specifix target use 'make <target> help' where target is"
	@$(ECHO) "  one of: "
	@$(ECHO) "     nuttx"
	@$(ECHO) "     posix"
	@$(ECHO) "     qurt"
	@$(ECHO) ""
endif

