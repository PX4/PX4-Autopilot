############################################################################
#
# Copyright (c) 2018 ECL Development Team. All rights reserved.
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

# Functions
# --------------------------------------------------------------------
# describe how to build a cmake config
define cmake-build
+@$(eval BUILD_DIR = $(SRC_DIR)/build/$@$(BUILD_DIR_SUFFIX))
+@if [ $(PX4_CMAKE_GENERATOR) = "Ninja" ] && [ -e $(BUILD_DIR)/Makefile ]; then rm -rf $(BUILD_DIR); fi
+@if [ ! -e $(BUILD_DIR)/CMakeCache.txt ]; then mkdir -p $(BUILD_DIR) && cd $(BUILD_DIR) && cmake $(2) -G"$(PX4_CMAKE_GENERATOR)" $(CMAKE_ARGS) $(3) $(4) || (rm -rf $(BUILD_DIR)); fi
+@(cd $(BUILD_DIR) && $(PX4_MAKE) $(PX4_MAKE_ARGS) $(ARGS))
endef


all:
	@$(call cmake-build,$@,$(SRC_DIR))

# Documentation
# --------------------------------------------------------------------
doxygen:
	@$(call cmake-build,$@,$(SRC_DIR), "-DBUILD_DOXYGEN=ON")
	@cmake --build $(SRC_DIR)/build/doxygen --target doxygen

# Testing
# --------------------------------------------------------------------

.PHONY: test_build test test_EKF

test_build:
	@$(call cmake-build,$@,$(SRC_DIR), "-DEKF_PYTHON_TESTS=ON")

test: test_build
	@cmake --build $(SRC_DIR)/build/test_build --target check

test_EKF: test_build
	@cmake --build $(SRC_DIR)/build/test_build --target ecl_EKF_pytest-quick
	
test_EKF_plots: test_build
	@cmake --build $(SRC_DIR)/build/test_build --target ecl_EKF_pytest-plots

test_build_asan:
	@$(call cmake-build,$@,$(SRC_DIR), "-DECL_ASAN=ON")

test_asan: test_build_asan
	@cmake --build $(SRC_DIR)/build/test_build_asan --target check

# Code coverage
# --------------------------------------------------------------------

coverage_build:
	@$(call cmake-build,$@,$(SRC_DIR), "-DCMAKE_BUILD_TYPE=Coverage", "-DEKF_PYTHON_TESTS=ON")
	
coverage: coverage_build
	@cmake --build $(SRC_DIR)/build/coverage_build --target coverage

coverage_html: coverage
	@cmake --build $(SRC_DIR)/build/coverage_build --target coverage_html

# Cleanup
# --------------------------------------------------------------------
.PHONY: clean distclean

clean:
	@rm -rf $(SRC_DIR)/build
	
distclean:
	@git clean -ff -x -d .

