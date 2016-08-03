############################################################################
#
# Copyright (c) 2015 - 2016 PX4 Development Team. All rights reserved.
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

CMAKE_VER := $(shell Tools/check_cmake.sh; echo $$?)
ifneq ($(CMAKE_VER),0)
    $(warning Not a valid CMake version or CMake not installed.)
    $(warning On Ubuntu, install or upgrade via:)
    $(warning )
    $(warning 3rd party PPA:)
    $(warning sudo add-apt-repository ppa:george-edison55/cmake-3.x -y)
    $(warning sudo apt-get update)
    $(warning sudo apt-get install cmake)
    $(warning )
    $(warning Official website:)
    $(warning wget https://cmake.org/files/v3.4/cmake-3.4.3-Linux-x86_64.sh)
    $(warning chmod +x cmake-3.4.3-Linux-x86_64.sh)
    $(warning sudo mkdir /opt/cmake-3.4.3)
    $(warning sudo ./cmake-3.4.3-Linux-x86_64.sh --prefix=/opt/cmake-3.4.3 --exclude-subdir)
    $(warning export PATH=/opt/cmake-3.4.3/bin:$$PATH)
    $(warning )
    $(error Fatal)
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
ARGS := $(wordlist 2,$(words $(MAKECMDGOALS)),$(MAKECMDGOALS))
j ?= 4

NINJA_BUILD := $(shell ninja --version 2>/dev/null)
ifdef NINJA_BUILD
    PX4_CMAKE_GENERATOR ?= "Ninja"
    PX4_MAKE = ninja
    PX4_MAKE_ARGS =
else

ifdef SYSTEMROOT
	# Windows
	PX4_CMAKE_GENERATOR ?= "MSYS Makefiles"
else
	PX4_CMAKE_GENERATOR ?= "Unix Makefiles"
endif
    PX4_MAKE = make
    PX4_MAKE_ARGS = -j$(j) --no-print-directory
endif

# check if replay env variable is set & set build dir accordingly
ifdef replay
	BUILD_DIR_SUFFIX := _replay
else
	BUILD_DIR_SUFFIX :=
endif

# Functions
# --------------------------------------------------------------------
# describe how to build a cmake config
define cmake-build
+@if [ $(PX4_CMAKE_GENERATOR) = "Ninja" ] && [ -e ./build_$@$(BUILD_DIR_SUFFIX)/Makefile ]; then rm -rf ./build_$@$(BUILD_DIR_SUFFIX); fi
+@if [ ! -e ./build_$@$(BUILD_DIR_SUFFIX)/CMakeCache.txt ]; then Tools/check_submodules.sh && mkdir -p ./build_$@$(BUILD_DIR_SUFFIX) && cd ./build_$@$(BUILD_DIR_SUFFIX) && cmake .. -G$(PX4_CMAKE_GENERATOR) -DCONFIG=$(1) || (cd .. && rm -rf ./build_$@$(BUILD_DIR_SUFFIX)); fi
+@Tools/check_submodules.sh
+@(echo "PX4 CONFIG: $@$(BUILD_DIR_SUFFIX)" && cd ./build_$@$(BUILD_DIR_SUFFIX) && $(PX4_MAKE) $(PX4_MAKE_ARGS) $(ARGS))
endef

define cmake-build-other
+@if [ $(PX4_CMAKE_GENERATOR) = "Ninja" ] && [ -e ./build_$@/Makefile ]; then rm -rf ./build_$@; fi
+@if [ ! -e ./build_$@/CMakeCache.txt ]; then Tools/check_submodules.sh && mkdir -p ./build_$@ && cd ./build_$@ && cmake $(2) -G$(PX4_CMAKE_GENERATOR) || (cd .. && rm -rf ./build_$@); fi
+@(cd ./build_$@ && $(PX4_MAKE) $(PX4_MAKE_ARGS) $(ARGS))
endef

# create empty targets to avoid msgs for targets passed to cmake
define cmake-targ
$(1):
	@#
.PHONY: $(1)
endef

define colorecho
      @tput setaf 6
      @echo $1
      @tput sgr0
endef

# ADD CONFIGS HERE
# --------------------------------------------------------------------
#  Do not put any spaces between function arguments.

tap-v1_default:
	$(call cmake-build,nuttx_tap-v1_default)

px4fmu-v1_default:
	$(call cmake-build,nuttx_px4fmu-v1_default)

px4fmu-v2_default:
	$(call cmake-build,nuttx_px4fmu-v2_default)

px4fmu-v2_test:
	$(call cmake-build,nuttx_px4fmu-v2_test)

px4fmu-v4_default:
	$(call cmake-build,nuttx_px4fmu-v4_default)

px4-stm32f4discovery_default:
	$(call cmake-build,nuttx_px4-stm32f4discovery_default)

px4fmu-v2_ekf2:
	$(call cmake-build,nuttx_px4fmu-v2_ekf2)

mindpx-v2_default:
	$(call cmake-build,nuttx_mindpx-v2_default)

posix_sitl_default:
	$(call cmake-build,$@)

posix_sitl_lpe:
	$(call cmake-build,$@)

posix_sitl_replay:
	$(call cmake-build,$@)

posix_sitl_broadcast:
	$(call cmake-build,$@)

ros_sitl_default:
	@echo "This target is deprecated. Use make 'posix_sitl_default gazebo' instead."

qurt_eagle_travis:
	$(call cmake-build,$@)

qurt_eagle_default:
	$(call cmake-build,$@)

posix_eagle_default:
	$(call cmake-build,$@)

eagle_default: posix_eagle_default qurt_eagle_default
eagle_legacy_default: posix_eagle_legacy_driver_default qurt_eagle_legacy_driver_default

qurt_eagle_legacy_driver_default:
	$(call cmake-build,$@)

posix_eagle_legacy_driver_default:
	$(call cmake-build,$@)

qurt_excelsior_default:
	$(call cmake-build,$@)

posix_excelsior_default:
	$(call cmake-build,$@)

excelsior_default: posix_excelsior_default qurt_excelsior_default

posix_rpi_native:
	$(call cmake-build,$@)

posix_rpi_cross:
	$(call cmake-build,$@)

posix_bebop_default:
	$(call cmake-build,$@)

posix: posix_sitl_default

broadcast: posix_sitl_broadcast

sitl_deprecation:
	@echo "Deprecated. Use 'make posix_sitl_default jmavsim' or"
	@echo "'make posix_sitl_default gazebo' if Gazebo is preferred."

run_sitl_quad: sitl_deprecation
run_sitl_plane: sitl_deprecation
run_sitl_ros: sitl_deprecation

# Other targets
# --------------------------------------------------------------------

.PHONY: gazebo_build uavcan_firmware check check_format unittest tests qgc_firmware package_firmware clean submodulesclean distclean
.NOTPARALLEL: gazebo_build uavcan_firmware check check_format unittest tests qgc_firmware package_firmware clean submodulesclean distclean

gazebo_build:
	@mkdir -p build_gazebo
	@if [ ! -e ./build_gazebo/CMakeCache.txt ];then cd build_gazebo && cmake -Wno-dev -G$(PX4_CMAKE_GENERATOR) ../Tools/sitl_gazebo; fi
	@cd build_gazebo && $(PX4_MAKE) $(PX4_MAKE_ARGS)
	@cd build_gazebo && $(PX4_MAKE) $(PX4_MAKE_ARGS) sdf

uavcan_firmware:
ifeq ($(VECTORCONTROL),1)
	$(call colorecho,"Downloading and building Vector control (FOC) firmware for the S2740VC and PX4ESC 1.6")
	@(rm -rf vectorcontrol && git clone --quiet --depth 1 https://github.com/thiemar/vectorcontrol.git && cd vectorcontrol && BOARD=s2740vc_1_0 make --silent --no-print-directory && BOARD=px4esc_1_6 make --silent --no-print-directory && ../Tools/uavcan_copy.sh)
endif

checks_defaults: \
	check_px4fmu-v1_default \
	check_px4fmu-v2_default \
	check_mindpx-v2_default \
	check_px4-stm32f4discovery_default \
	check_tap-v1_default \

checks_bootloaders: \


checks_tests: \
	check_px4fmu-v2_test

checks_alts: \
	check_px4fmu-v2_ekf2 \

checks_uavcan: \
	check_px4fmu-v4_default_and_uavcan

checks_sitls: \
	check_posix_sitl_default

checks_last: \
	check_tests \
	check_format \

check: checks_defaults checks_tests checks_alts checks_uavcan checks_bootloaders checks_last
quick_check: check_px4fmu-v2_default check_px4fmu-v4_default check_tests check_format

check_format:
	$(call colorecho,"Checking formatting with astyle")
	@./Tools/fix_code_style.sh
	@./Tools/check_code_style_all.sh

check_%:
	@echo
	$(call colorecho,"Building" $(subst check_,,$@))
	@$(MAKE) --no-print-directory $(subst check_,,$@)
	@echo

check_px4fmu-v4_default: uavcan_firmware
check_px4fmu-v4_default_and_uavcan: check_px4fmu-v4_default
	@echo
ifeq ($(VECTORCONTROL),1)
	@echo "Cleaning up vectorcontrol firmware"
	@rm -rf vectorcontrol
	@rm -rf ROMFS/px4fmu_common/uavcan
endif

unittest: posix_sitl_default
	$(call cmake-build-other,unittest, ../unittests)
	@(cd build_unittest && ctest -j2 --output-on-failure)

run_tests_posix: posix_sitl_default
	@mkdir -p build_posix_sitl_default/src/firmware/posix/rootfs/fs/microsd
	@mkdir -p build_posix_sitl_default/src/firmware/posix/rootfs/eeprom
	@touch build_posix_sitl_default/src/firmware/posix/rootfs/eeprom/parameters
	@(cd build_posix_sitl_default/src/firmware/posix && ./px4 -d ../../../../posix-configs/SITL/init/rcS_tests | tee test_output)
	@(cd build_posix_sitl_default/src/firmware/posix && grep --color=always "All tests passed" test_output)

tests: check_unittest run_tests_posix

# QGroundControl flashable firmware
qgc_firmware: \
	check_px4fmu-v1_default \
	check_px4fmu-v2_default \
	check_mindpx-v2_default \
	check_px4fmu-v4_default_and_uavcan \
	check_format

extra_firmware: \
	check_px4-stm32f4discovery_default \
	check_px4fmu-v2_test \
	check_px4fmu-v2_ekf2

package_firmware:
	@zip --junk-paths Firmware.zip `find . -name \*.px4`

clean:
	@rm -rf build_*/
	@(cd NuttX/nuttx && make clean)

submodulesclean:
	@git submodule sync --recursive
	@git submodule deinit -f .
	@git submodule update --init --recursive --force

distclean: submodulesclean
	@git clean -ff -x -d -e ".project" -e ".cproject"

# targets handled by cmake
cmake_targets = install test upload package package_source debug debug_tui debug_ddd debug_io debug_io_tui debug_io_ddd check_weak \
	run_cmake_config config gazebo gazebo_gdb gazebo_lldb jmavsim replay \
	jmavsim_gdb jmavsim_lldb gazebo_gdb_iris gazebo_lldb_tailsitter gazebo_iris gazebo_iris_opt_flow gazebo_tailsitter \
	gazebo_gdb_standard_vtol gazebo_lldb_standard_vtol gazebo_standard_vtol gazebo_plane gazebo_solo gazebo_typhoon_h480
$(foreach targ,$(cmake_targets),$(eval $(call cmake-targ,$(targ))))

.PHONY: clean

CONFIGS:=$(shell ls cmake/configs | sed -e "s~.*/~~" | sed -e "s~\..*~~")

#help:
#	@echo
#	@echo "Type 'make ' and hit the tab key twice to see a list of the available"
#	@echo "build configurations."
#	@echo
