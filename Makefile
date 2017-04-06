############################################################################
#
# Copyright (c) 2015 - 2017 PX4 Development Team. All rights reserved.
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
############################################################################

# Enforce the presence of the GIT repository
#
# We depend on our submodules, so we have to prevent attempts to
# compile without it being present.
ifeq ($(wildcard .git),)
    $(error YOU HAVE TO USE GIT TO DOWNLOAD THIS REPOSITORY. ABORTING.)
endif

# Help
# --------------------------------------------------------------------
# Don't be afraid of this makefile, it is just passing
# arguments to cmake to allow us to keep the wiki pages etc.
# that describe how to build the px4 firmware
# the same even when using cmake instead of make.
#
# Example usage:
#
# make px4fmu-v2_default 			(builds)
# make px4fmu-v2_default upload 	(builds and uploads)
# make px4fmu-v2_default test 		(builds and tests)
#
# This tells cmake to build the nuttx px4fmu-v2 default config in the
# directory build_nuttx_px4fmu-v2_default and then call make
# in that directory with the target upload.

#  explicity set default build target
all: posix_sitl_default

# Parsing
# --------------------------------------------------------------------
# assume 1st argument passed is the main target, the
# rest are arguments to pass to the makefile generated
# by cmake in the subdirectory
FIRST_ARG := $(firstword $(MAKECMDGOALS))
ARGS := $(wordlist 2,$(words $(MAKECMDGOALS)),$(MAKECMDGOALS))
j ?= 4

NINJA_BIN := ninja
ifndef NO_NINJA_BUILD
	NINJA_BUILD := $(shell $(NINJA_BIN) --version 2>/dev/null)

	ifndef NINJA_BUILD
		NINJA_BIN := ninja-build
		NINJA_BUILD := $(shell $(NINJA_BIN) --version 2>/dev/null)
	endif
endif

ifdef NINJA_BUILD
	PX4_CMAKE_GENERATOR := Ninja
	PX4_MAKE := $(NINJA_BIN)
	PX4_MAKE_ARGS :=
else
	ifdef SYSTEMROOT
		# Windows
		PX4_CMAKE_GENERATOR := "MSYS\ Makefiles"
	else
		PX4_CMAKE_GENERATOR := "Unix\ Makefiles"
	endif
	PX4_MAKE = $(MAKE)
	PX4_MAKE_ARGS = -j$(j) --no-print-directory
endif

SRC_DIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

# check if replay env variable is set & set build dir accordingly
ifdef replay
	BUILD_DIR_SUFFIX := _replay
else
	BUILD_DIR_SUFFIX :=
endif

# additional config parameters passed to cmake
CMAKE_ARGS :=
ifdef EXTERNAL_MODULES_LOCATION
	CMAKE_ARGS := -DEXTERNAL_MODULES_LOCATION:STRING=$(EXTERNAL_MODULES_LOCATION)
endif


# Functions
# --------------------------------------------------------------------
# describe how to build a cmake config
define cmake-build
+@$(eval BUILD_DIR = $(SRC_DIR)/build_$@$(BUILD_DIR_SUFFIX))
+@if [ $(PX4_CMAKE_GENERATOR) = "Ninja" ] && [ -e $(BUILD_DIR)/Makefile ]; then rm -rf $(BUILD_DIR); fi
+@if [ ! -e $(BUILD_DIR)/CMakeCache.txt ]; then mkdir -p $(BUILD_DIR) && cd $(BUILD_DIR) && cmake $(2) -G"$(PX4_CMAKE_GENERATOR)" -DCONFIG=$(1) $(CMAKE_ARGS) || (rm -rf $(BUILD_DIR)); fi
+@(cd $(BUILD_DIR) && $(PX4_MAKE) $(PX4_MAKE_ARGS) $(ARGS))
endef

define colorecho
+@tput setaf 6
+@echo $1
+@tput sgr0
endef

# Get a list of all config targets.
ALL_CONFIG_TARGETS := $(basename $(shell find "$(SRC_DIR)/cmake/configs" ! -name '*_common*' ! -name '*_sdflight_*' -name '*.cmake' -print | sed  -e 's:^.*/::' | sort))
# Strip off leading nuttx_
NUTTX_CONFIG_TARGETS := $(patsubst nuttx_%,%,$(filter nuttx_%,$(ALL_CONFIG_TARGETS)))

# ADD CONFIGS HERE
# --------------------------------------------------------------------
#  Do not put any spaces between function arguments.

# All targets.
$(ALL_CONFIG_TARGETS):
	$(call cmake-build,$@,$(SRC_DIR))

# Abbreviated config targets.

# nuttx_ is left off by default; provide a rule to allow that.
$(NUTTX_CONFIG_TARGETS):
	$(call cmake-build,nuttx_$@,$(SRC_DIR))

all_nuttx_targets: $(NUTTX_CONFIG_TARGETS)

posix: posix_sitl_default
broadcast: posix_sitl_broadcast

# Multi- config targets.
eagle_default: posix_eagle_default qurt_eagle_default
eagle_legacy_default: posix_eagle_legacy qurt_eagle_legacy
excelsior_default: posix_excelsior_default qurt_excelsior_default
excelsior_legacy_default: posix_excelsior_legacy qurt_excelsior_legacy


# All targets with just dependencies but no recipe must either be marked as phony (or have the special @: as recipe).
.PHONY: all posix broadcast eagle_default eagle_legacy_default excelsior_legacy_default excelsior_default all_nuttx_targets

# Other targets
# --------------------------------------------------------------------

.PHONY: qgc_firmware alt_firmware checks_bootloaders uavcan_firmware sizes check quick_check

# QGroundControl flashable NuttX firmware
qgc_firmware: \
	check_auav-x21_default \
	check_aerofc-v1_default \
	check_crazyflie_default \
	check_mindpx-v2_default \
	check_px4fmu-v1_default \
	check_px4fmu-v2_default \
	check_px4fmu-v2_lpe \
	check_px4fmu-v3_default \
	check_px4fmu-v4_default \
	check_tap-v1_default \
	check_sizes

# Other NuttX firmware
alt_firmware: \
	check_px4-stm32f4discovery_default \
	check_px4cannode-v1_default \
	check_px4esc-v1_default \
	check_px4fmu-v4pro_default \
	check_px4fmu-v5_default \
	check_px4nucleoF767ZI-v1_default \
	check_s2740vc-v1_default \
	check_sizes

checks_bootloaders: \
	check_esc35-v1_bootloader \
	check_px4cannode-v1_bootloader \
	check_px4esc-v1_bootloader \
	check_px4flow-v2_bootloader \
	check_s2740vc-v1_bootloader \
# not fitting in flash	check_zubaxgnss-v1_bootloader \
	check_sizes

uavcan_firmware:
	$(call colorecho,"Downloading and building Vector control (FOC) firmware for the S2740VC and PX4ESC 1.6")
	@rm -rf vectorcontrol
	@git clone --quiet --depth 1 https://github.com/thiemar/vectorcontrol.git && cd vectorcontrol
	@BOARD=s2740vc_1_0 make --silent --no-print-directory
	@BOARD=px4esc_1_6 make --silent --no-print-directory && $(SRC_DIR)/Tools/uavcan_copy.sh)


sizes:
	@-find build_* -name firmware_nuttx -type f | xargs size 2> /dev/null || :

# All default targets that don't require a special build environment
check: check_posix_sitl_default qgc_firmware alt_firmware checks_bootloaders tests check_format

# quick_check builds a single nuttx and posix target, runs testing, and checks the style
quick_check: check_posix_sitl_default check_px4fmu-v3_default tests check_format

check_%:
	@echo
	$(call colorecho,"Building" $(subst check_,,$@))
	@$(MAKE) --no-print-directory $(subst check_,,$@)
	@echo

# Documentation
# --------------------------------------------------------------------
.PHONY: parameters_markdown

parameters_markdown: posix_sitl_default
	@python $(SRC_DIR)/Tools/px_process_params.py -s $(SRC_DIR)/src --markdown

# S3 upload helpers
# --------------------------------------------------------------------
# s3cmd uses these ENV variables
#  AWS_ACCESS_KEY_ID
#  AWS_SECRET_ACCESS_KEY
#  AWS_S3_BUCKET
.PHONY: s3put_firmware s3put_qgc_firmware

Firmware.zip:
	@rm -rf Firmware.zip
	@zip --junk-paths Firmware.zip `find . -name \*.px4`

s3put_firmware: Firmware.zip
	$(SRC_DIR)/Tools/s3put.sh Firmware.zip

s3put_qgc_firmware: qgc_firmware
	@$(SRC_DIR)/Tools/s3put.sh $(SRC_DIR)/build_px4fmu-v3_default/airframes.xml
	@$(SRC_DIR)/Tools/s3put.sh $(SRC_DIR)/build_px4fmu-v3_default/parameters.xml
	@find $(SRC_DIR)/build_* -name "*.px4" -exec $(SRC_DIR)/Tools/s3put.sh "{}" \;

s3put_parameters_markdown: parameters_markdown
	@$(SRC_DIR)/Tools/s3put.sh parameters.md

# Astyle
# --------------------------------------------------------------------
.PHONY: check_format format

check_format:
	$(call colorecho,"Checking formatting with astyle")
	@$(SRC_DIR)/Tools/check_code_style_all.sh
	@git diff --check

format:
	$(call colorecho,"Formatting with astyle")
	@$(SRC_DIR)/Tools/check_code_style_all.sh --fix

# Testing
# --------------------------------------------------------------------
.PHONY: unittest run_tests_posix tests tests_coverage

unittest: posix_sitl_default
	$(call cmake-build,unittest,$(SRC_DIR)/unittests)
	@(cd build_unittest && ctest -j2 --output-on-failure)

run_tests_posix:
	$(MAKE) --no-print-directory posix_sitl_default test_results

tests: unittest run_tests_posix

tests_coverage:
	@lcov --zerocounters --directory $(SRC_DIR) --quiet
	@lcov --capture --initial --directory $(SRC_DIR) --quiet --output-file coverage.info
	@$(MAKE) --no-print-directory unittest PX4_CODE_COVERAGE=1 CCACHE_DISABLE=1
	@$(MAKE) --no-print-directory posix_sitl_default test_results PX4_CODE_COVERAGE=1 CCACHE_DISABLE=1
	@lcov --no-checksum --directory $(SRC_DIR) --capture --quiet --output-file coverage.info
	@lcov --remove coverage.info '/usr/*' 'unittests/googletest/*' --quiet --output-file coverage.info
	@genhtml --legend --show-details --function-coverage --quiet --output-directory coverage-html coverage.info
	@$(MAKE) --no-print-directory posix_sitl_default test_results_junit



# Clang analyzers
# --------------------------------------------------------------------
.PHONY: scan-build clang-check clang-tidy

scan-build:
	@export CCACHE_DISABLE=1
	@mkdir -p $(SRC_DIR)/build_posix_sitl_default_scan-build
	@cd $(SRC_DIR)/build_posix_sitl_default_scan-build && scan-build cmake .. -GNinja -DCONFIG=posix_sitl_default
	@scan-build cmake --build $(SRC_DIR)/build_posix_sitl_default_scan-build

clang-check:
	@CC=clang CXX=clang++ $(MAKE) --no-print-directory posix_sitl_default
	@$(SRC_DIR)/Tools/clang-tool.sh -b build_posix_sitl_default -t clang-check

clang-tidy:
	rm -rf $(SRC_DIR)/build_posix_sitl_default
	@CC=clang CXX=clang++ $(MAKE) --no-print-directory posix_sitl_default
	@$(SRC_DIR)/Tools/clang-tool.sh -b build_posix_sitl_default -t clang-tidy

clang-tidy-parallel:
	rm -rf $(SRC_DIR)/build_posix_sitl_default
	@CC=clang CXX=clang++ $(MAKE) --no-print-directory posix_sitl_default
	@$(SRC_DIR)/Tools/run-clang-tidy.py -j$(j) -p $(SRC_DIR)/build_posix_sitl_default

clang-tidy-fix:
	rm -rf $(SRC_DIR)/build_posix_sitl_default
	@CC=clang CXX=clang++ $(MAKE) --no-print-directory posix_sitl_default
	@run-clang-tidy.py -fix -j$(j) -p $(SRC_DIR)/build_posix_sitl_default

# Cleanup
# --------------------------------------------------------------------
.PHONY: clean submodulesclean distclean

clean:
	@rm -rf $(SRC_DIR)/build_*/
	-@$(MAKE) --no-print-directory -C NuttX/nuttx clean

submodulesclean:
	@git submodule sync --recursive
	@git submodule deinit -f .
	@git submodule update --init --recursive --force

gazeboclean:
	@rm -rf ~/.gazebo/*

distclean: submodulesclean gazeboclean
	@git clean -ff -x -d -e ".project" -e ".cproject" -e ".idea"

# --------------------------------------------------------------------

# All other targets are handled by PX4_MAKE. Add a rule here to avoid printing an error.
%:
	$(if $(filter $(FIRST_ARG),$@), \
		$(error "$@ cannot be the first argument. Use '$(MAKE) help|list_config_targets' to get a list of all possible [configuration] targets."),@#)

CONFIGS:=$(shell ls cmake/configs | sed -e "s~.*/~~" | sed -e "s~\..*~~")

#help:
#	@echo
#	@echo "Type 'make ' and hit the tab key twice to see a list of the available"
#	@echo "build configurations."
#	@echo

empty :=
space := $(empty) $(empty)

# Print a list of non-config targets (based on http://stackoverflow.com/a/26339924/1487069)
help:
	@echo "Usage: $(MAKE) <target>"
	@echo "Where <target> is one of:"
	@$(MAKE) -pRrq -f $(lastword $(MAKEFILE_LIST)) : 2>/dev/null | \
		awk -v RS= -F: '/^# File/,/^# Finished Make data base/ {if ($$1 !~ "^[#.]") {print $$1}}' | sort | \
		egrep -v -e '^[^[:alnum:]]' -e '^($(subst $(space),|,$(ALL_CONFIG_TARGETS) $(NUTTX_CONFIG_TARGETS)))$$' -e '_default$$' -e '^(posix|eagle|Makefile)'
	@echo
	@echo "Or, $(MAKE) <config_target> [<make_target(s)>]"
	@echo "Use '$(MAKE) list_config_targets' for a list of configuration targets."

# Print a list of all config targets.
list_config_targets:
	@for targ in $(patsubst nuttx_%,[nuttx_]%,$(ALL_CONFIG_TARGETS)); do echo $$targ; done
