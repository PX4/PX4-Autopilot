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
# directory build/nuttx_px4fmu-v2_default and then call make
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

	ifdef VERBOSE
		PX4_MAKE_ARGS := -v
	else
		PX4_MAKE_ARGS :=
	endif
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
ifdef EXTERNAL_MODULES_LOCATION
	CMAKE_ARGS += -DEXTERNAL_MODULES_LOCATION:STRING=$(EXTERNAL_MODULES_LOCATION)
endif

ifdef PX4_CMAKE_BUILD_TYPE
	CMAKE_ARGS += -DCMAKE_BUILD_TYPE=${PX4_CMAKE_BUILD_TYPE}
else

	# Address Sanitizer
	ifdef PX4_ASAN
		CMAKE_ARGS += -DCMAKE_BUILD_TYPE=AddressSanitizer
	endif

	# Memory Sanitizer
	ifdef PX4_MSAN
		CMAKE_ARGS += -DCMAKE_BUILD_TYPE=MemorySanitizer
	endif

	# Thread Sanitizer
	ifdef PX4_TSAN
		CMAKE_ARGS += -DCMAKE_BUILD_TYPE=ThreadSanitizer
	endif

	# Undefined Behavior Sanitizer
	ifdef PX4_UBSAN
		CMAKE_ARGS += -DCMAKE_BUILD_TYPE=UndefinedBehaviorSanitizer
	endif

endif

# Functions
# --------------------------------------------------------------------
# describe how to build a cmake config
define cmake-build
+@$(eval PX4_CONFIG = $(1))
+@$(eval BUILD_DIR = $(SRC_DIR)/build/$(PX4_CONFIG)$(BUILD_DIR_SUFFIX))
+@if [ $(PX4_CMAKE_GENERATOR) = "Ninja" ] && [ -e $(BUILD_DIR)/Makefile ]; then rm -rf $(BUILD_DIR); fi
+@if [ ! -e $(BUILD_DIR)/CMakeCache.txt ]; then mkdir -p $(BUILD_DIR) && cd $(BUILD_DIR) && cmake $(SRC_DIR) -G"$(PX4_CMAKE_GENERATOR)" $(CMAKE_ARGS) -DCONFIG=$(PX4_CONFIG) || (rm -rf $(BUILD_DIR)); fi
+@$(PX4_MAKE) -C $(BUILD_DIR) $(PX4_MAKE_ARGS) $(ARGS)
endef

COLOR_BLUE = \033[0;94m
NO_COLOR   = \033[m

define colorecho
+@echo -e '${COLOR_BLUE}${1} ${NO_COLOR}'
endef

# Get a list of all config targets cmake/configs/*.cmake
ALL_CONFIG_TARGETS := $(basename $(shell find "$(SRC_DIR)/cmake/configs" -maxdepth 1 ! -name '*_common*' ! -name '*_sdflight_*' -name '*.cmake' -print | sed  -e 's:^.*/::' | sort))
# Strip off leading nuttx_
NUTTX_CONFIG_TARGETS := $(patsubst nuttx_%,%,$(filter nuttx_%,$(ALL_CONFIG_TARGETS)))

# ADD CONFIGS HERE
# --------------------------------------------------------------------
#  Do not put any spaces between function arguments.

# All targets.
$(ALL_CONFIG_TARGETS):
	$(call cmake-build,$@)

# Abbreviated config targets.

# nuttx_ is left off by default; provide a rule to allow that.
$(NUTTX_CONFIG_TARGETS):
	$(call cmake-build,nuttx_$@)

all_nuttx_targets: $(NUTTX_CONFIG_TARGETS)

posix: posix_sitl_default
broadcast: posix_sitl_broadcast

# All targets with just dependencies but no recipe must either be marked as phony (or have the special @: as recipe).
.PHONY: all posix broadcast all_nuttx_targets

# Multi- config targets.
eagle_default: posix_eagle_default qurt_eagle_default
eagle_rtps: posix_eagle_rtps qurt_eagle_default
eagle_legacy_default: posix_eagle_legacy qurt_eagle_legacy
excelsior_default: posix_excelsior_default qurt_excelsior_default
excelsior_rtps: posix_excelsior_rtps qurt_excelsior_default
excelsior_legacy_default: posix_excelsior_legacy qurt_excelsior_legacy

.PHONY: eagle_default eagle_rtps eagle_legacy_default
.PHONY: excelsior_default excelsior_rtps excelsior_legacy_default

# Other targets
# --------------------------------------------------------------------

.PHONY: qgc_firmware px4fmu_firmware misc_qgc_extra_firmware alt_firmware check_rtps

# QGroundControl flashable NuttX firmware
qgc_firmware: px4fmu_firmware misc_qgc_extra_firmware

# px4fmu NuttX firmware
px4fmu_firmware: \
	check_px4io-v2_default \
	check_px4fmu-v2_default \
	check_px4fmu-v3_default \
	check_px4fmu-v4_default \
	check_px4fmu-v4pro_default \
	check_px4fmu-v5_default \
	sizes

misc_qgc_extra_firmware: \
	check_aerocore2_default \
	check_aerofc-v1_default \
	check_auav-x21_default \
	check_crazyflie_default \
	check_mindpx-v2_default \
	check_px4fmu-v2_lpe \
	check_tap-v1_default \
	sizes

# Other NuttX firmware
alt_firmware: \
	check_nxphlite-v3_default \
	check_px4-same70xplained-v1_default \
	check_px4-stm32f4discovery_default \
	check_px4cannode-v1_default \
	check_px4esc-v1_default \
	check_px4nucleoF767ZI-v1_default \
	check_s2740vc-v1_default \
	sizes

# builds with RTPS
check_rtps: \
	check_px4fmu-v3_rtps \
	check_px4fmu-v4_rtps \
	check_px4fmu-v4pro_rtps \
	check_posix_sitl_rtps \
	sizes

.PHONY: sizes check quick_check check_rtps uorb_graphs

sizes:
	@-find build -name *.elf -type f | xargs size 2> /dev/null || :

# All default targets that don't require a special build environment
check: check_posix_sitl_default px4fmu_firmware misc_qgc_extra_firmware alt_firmware tests check_format

# quick_check builds a single nuttx and posix target, runs testing, and checks the style
quick_check: check_posix_sitl_default check_px4fmu-v4pro_default tests check_format

check_%:
	@echo
	$(call colorecho,'Building' $(subst check_,,$@))
	@$(MAKE) --no-print-directory $(subst check_,,$@)
	@echo

uorb_graphs:
	@./Tools/uorb_graph/create_from_startupscript.sh
	@./Tools/uorb_graph/create.py --src-path src --exclude-path src/examples --file Tools/uorb_graph/graph_full
	@$(MAKE) --no-print-directory px4fmu-v2_default uorb_graph
	@$(MAKE) --no-print-directory px4fmu-v4_default uorb_graph
	@$(MAKE) --no-print-directory posix_sitl_default uorb_graph


.PHONY: coverity_scan

coverity_scan: posix_sitl_default

# Documentation
# --------------------------------------------------------------------
.PHONY: parameters_metadata airframe_metadata module_documentation px4_metadata doxygen

parameters_metadata:
	@$(MAKE) --no-print-directory posix_sitl_default metadata_parameters

airframe_metadata:
	@$(MAKE) --no-print-directory posix_sitl_default metadata_airframes

module_documentation:
	@$(MAKE) --no-print-directory posix_sitl_default metadata_module_documentation

px4_metadata: parameters_metadata airframe_metadata module_documentation

doxygen:
	@mkdir -p $(SRC_DIR)/build/doxygen
	@cd $(SRC_DIR)/build/doxygen && cmake $(SRC_DIR) $(CMAKE_ARGS) -G"$(PX4_CMAKE_GENERATOR)" -DCONFIG=posix_sitl_default -DBUILD_DOXYGEN=ON
	@$(PX4_MAKE) -C $(SRC_DIR)/build/doxygen
	@touch $(SRC_DIR)/build/doxygen/Documentation/.nojekyll

# Astyle
# --------------------------------------------------------------------
.PHONY: check_format format

check_format:
	$(call colorecho,'Checking formatting with astyle')
	@$(SRC_DIR)/Tools/astyle/check_code_style_all.sh
	@cd $(SRC_DIR) && git diff --check

format:
	$(call colorecho,'Formatting with astyle')
	@$(SRC_DIR)/Tools/astyle/check_code_style_all.sh --fix

# Testing
# --------------------------------------------------------------------
.PHONY: tests tests_coverage tests_mission tests_mission_coverage tests_offboard rostest python_coverage

tests:
	@$(MAKE) --no-print-directory posix_sitl_default test_results \
	ASAN_OPTIONS="color=always:check_initialization_order=1:detect_stack_use_after_return=1" \
	UBSAN_OPTIONS="color=always"

tests_coverage:
	@$(MAKE) clean
	@$(MAKE) --no-print-directory posix_sitl_default test_coverage_genhtml PX4_CMAKE_BUILD_TYPE=Coverage
	@echo "Open $(SRC_DIR)/build/posix_sitl_default/coverage-html/index.html to see coverage"

rostest: posix_sitl_default
	@$(MAKE) --no-print-directory posix_sitl_default sitl_gazebo

tests_mission: rostest
	@$(SRC_DIR)/test/rostest_px4_run.sh mavros_posix_tests_missions.test

tests_mission_coverage:
	@$(MAKE) clean
	@$(MAKE) --no-print-directory posix_sitl_default PX4_CMAKE_BUILD_TYPE=Coverage
	@$(MAKE) --no-print-directory posix_sitl_default sitl_gazebo PX4_CMAKE_BUILD_TYPE=Coverage
	@$(SRC_DIR)/test/rostest_px4_run.sh mavros_posix_test_mission.test mission:=VTOL_mission_1 vehicle:=standard_vtol
	@$(MAKE) --no-print-directory posix_sitl_default generate_coverage

tests_offboard: rostest
	@$(SRC_DIR)/test/rostest_px4_run.sh mavros_posix_tests_offboard_attctl.test
	@$(SRC_DIR)/test/rostest_px4_run.sh mavros_posix_tests_offboard_posctl.test

python_coverage:
	@mkdir -p $(SRC_DIR)/build/python_coverage
	@cd $(SRC_DIR)/build/python_coverage && cmake $(SRC_DIR) $(CMAKE_ARGS) -G"$(PX4_CMAKE_GENERATOR)" -DCONFIG=posix_sitl_default -DPYTHON_COVERAGE=ON
	@$(PX4_MAKE) -C $(SRC_DIR)/build/python_coverage
	@$(PX4_MAKE) -C $(SRC_DIR)/build/python_coverage metadata_airframes
	@$(PX4_MAKE) -C $(SRC_DIR)/build/python_coverage metadata_parameters
	#@$(PX4_MAKE) -C $(SRC_DIR)/build/python_coverage module_documentation # TODO: fix within coverage.py
	@coverage combine `find . -name .coverage\*`
	@coverage report -m

# static analyzers (scan-build, clang-tidy, cppcheck)
# --------------------------------------------------------------------
.PHONY: scan-build posix_sitl_default-clang clang-tidy clang-tidy-fix clang-tidy-quiet
.PHONY: cppcheck shellcheck_all validate_module_configs

scan-build:
	@export CCC_CC=clang
	@export CCC_CXX=clang++
	@rm -rf $(SRC_DIR)/build/posix_sitl_default-scan-build
	@rm -rf $(SRC_DIR)/build/scan-build/report_latest
	@mkdir -p $(SRC_DIR)/build/posix_sitl_default-scan-build
	@cd $(SRC_DIR)/build/posix_sitl_default-scan-build && scan-build cmake $(SRC_DIR) -GNinja -DCONFIG=posix_sitl_default
	@scan-build -o $(SRC_DIR)/build/scan-build cmake --build $(SRC_DIR)/build/posix_sitl_default-scan-build
	@find $(SRC_DIR)/build/scan-build -maxdepth 1 -mindepth 1 -type d -exec cp -r "{}" $(SRC_DIR)/build/scan-build/report_latest \;

posix_sitl_default-clang:
	@mkdir -p $(SRC_DIR)/build/posix_sitl_default-clang
	@cd $(SRC_DIR)/build/posix_sitl_default-clang && cmake $(SRC_DIR) $(CMAKE_ARGS) -G"$(PX4_CMAKE_GENERATOR)" -DCONFIG=posix_sitl_default -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++
	@$(PX4_MAKE) -C $(SRC_DIR)/build/posix_sitl_default-clang

clang-tidy: posix_sitl_default-clang
	@cd $(SRC_DIR)/build/posix_sitl_default-clang && $(SRC_DIR)/Tools/run-clang-tidy.py -header-filter=".*\.hpp" -j$(j) -p .

# to automatically fix a single check at a time, eg modernize-redundant-void-arg
#  % run-clang-tidy-4.0.py -fix -j4 -checks=-\*,modernize-redundant-void-arg -p .
clang-tidy-fix: posix_sitl_default-clang
	@cd $(SRC_DIR)/build/posix_sitl_default-clang && $(SRC_DIR)/Tools/run-clang-tidy.py -header-filter=".*\.hpp" -j$(j) -fix -p .

# modified version of run-clang-tidy.py to return error codes and only output relevant results
clang-tidy-quiet: posix_sitl_default-clang
	@cd $(SRC_DIR)/build/posix_sitl_default-clang && $(SRC_DIR)/Tools/run-clang-tidy.py -header-filter=".*\.hpp" -j$(j) -p .

# TODO: Fix cppcheck errors then try --enable=warning,performance,portability,style,unusedFunction or --enable=all
cppcheck: posix_sitl_default
	@mkdir -p $(SRC_DIR)/build/cppcheck
	@cppcheck -i$(SRC_DIR)/src/examples --enable=performance --std=c++11 --std=c99 --std=posix --project=$(SRC_DIR)/build/posix_sitl_default/compile_commands.json --xml-version=2 2> $(SRC_DIR)/build/cppcheck/cppcheck-result.xml > /dev/null
	@cppcheck-htmlreport --source-encoding=ascii --file=$(SRC_DIR)/build/cppcheck/cppcheck-result.xml --report-dir=$(SRC_DIR)/build/cppcheck --source-dir=$(SRC_DIR)/src/

shellcheck_all:
	@$(SRC_DIR)/Tools/run-shellcheck.sh $(SRC_DIR)/ROMFS/px4fmu_common/
	@make px4fmu-v2_default shellcheck

validate_module_configs:
	@find $(SRC_DIR)/src/modules $(SRC_DIR)/src/drivers $(SRC_DIR)/src/lib -name *.yaml -type f -print0 | xargs -0 $(SRC_DIR)/Tools/validate_yaml.py --schema-file $(SRC_DIR)/validation/module_schema.yaml

# Cleanup
# --------------------------------------------------------------------
.PHONY: clean submodulesclean submodulesupdate gazeboclean distclean

clean:
	@rm -rf $(SRC_DIR)/build

submodulesclean:
	@git submodule foreach --quiet --recursive git clean -ff -x -d
	@git submodule update --quiet --init --recursive --force || true
	@git submodule sync --recursive
	@git submodule update --init --recursive --force

submodulesupdate:
	@git submodule update --quiet --init --recursive || true
	@git submodule sync --recursive
	@git submodule update --init --recursive

gazeboclean:
	@rm -rf ~/.gazebo/*

distclean: gazeboclean
	@git submodule deinit -f .
	@git clean -ff -x -d -e ".project" -e ".cproject" -e ".idea" -e ".settings" -e ".vscode"

# --------------------------------------------------------------------

# All other targets are handled by PX4_MAKE. Add a rule here to avoid printing an error.
%:
	$(if $(filter $(FIRST_ARG),$@), \
		$(error "$@ cannot be the first argument. Use '$(MAKE) help|list_config_targets' to get a list of all possible [configuration] targets."),@#)

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
