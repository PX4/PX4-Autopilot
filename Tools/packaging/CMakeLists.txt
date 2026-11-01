############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

# PX4 board package.
#
# One configure of this project packages one board from its descriptor,
# boards/<vendor>/<board>/package.yaml, after the component targets have
# been built. `make <vendor>_<board>_deb` drives it through
# build_package.py; a distribution recipe can call it directly:
#
#   cmake -S Tools/packaging -B build/<board>_package -DPX4_PACKAGE_BOARD=<vendor>_<board>
#   cpack -G DEB                                   (in build/<board>_package)
#   DESTDIR=<staging> cmake --install build/<board>_package   (rpm, Yocto, ...)
#
# Every install destination is absolute, so the tree under DESTDIR is the
# tree on the target. package_descriptor.py validates the descriptor and
# generates the board-specific install rules; the runtime below is the same
# for every PX4 package.

cmake_minimum_required(VERSION 3.16)
project(px4_package NONE)

set(PX4_PACKAGE_BOARD "" CACHE STRING "<vendor>_<board> with a boards/<vendor>/<board>/package.yaml")
get_filename_component(default_source_dir "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(PX4_SOURCE_DIR "${default_source_dir}" CACHE PATH "PX4 source tree holding boards/")
set(PX4_BUILD_ROOT "${PX4_SOURCE_DIR}/build" CACHE PATH "Directory holding the built component targets")
set(PX4_PACKAGE_VERSION "" CACHE STRING "Debian version; derived from git describe when empty")
set(PX4_PACKAGE_GIT_TAG "" CACHE STRING "git describe output to derive the version from")

if(NOT PX4_PACKAGE_BOARD)
	message(FATAL_ERROR "PX4_PACKAGE_BOARD is required: cmake -DPX4_PACKAGE_BOARD=<vendor>_<board>")
endif()

string(REPLACE "_" "/" board_path "${PX4_PACKAGE_BOARD}")
set(descriptor "${PX4_SOURCE_DIR}/boards/${board_path}/package.yaml")
if(NOT EXISTS "${descriptor}")
	message(FATAL_ERROR "no package descriptor ${descriptor}")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${descriptor}")

find_package(Python3 3.8 REQUIRED COMPONENTS Interpreter)

set(emit_args)
if(PX4_PACKAGE_VERSION)
	list(APPEND emit_args --version "${PX4_PACKAGE_VERSION}")
endif()
if(PX4_PACKAGE_GIT_TAG)
	list(APPEND emit_args --git-tag "${PX4_PACKAGE_GIT_TAG}")
endif()

execute_process(
	COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/package_descriptor.py emit "${descriptor}"
		--build-root "${PX4_BUILD_ROOT}" --output-dir "${CMAKE_CURRENT_BINARY_DIR}" ${emit_args}
	RESULT_VARIABLE emit_result
)
if(NOT emit_result EQUAL 0)
	message(FATAL_ERROR "package_descriptor.py emit failed for ${descriptor}")
endif()

# PX4_PACKAGE_* variables and the board's install rules
include(${CMAKE_CURRENT_BINARY_DIR}/package_descriptor.cmake)

# Hook runtime shared by every PX4 package
install(PROGRAMS hooks/run-hooks DESTINATION /usr/lib/px4)
install(DIRECTORY hooks/apply DESTINATION /usr/lib/px4 USE_SOURCE_PERMISSIONS)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/package.env DESTINATION /usr/lib/px4)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/package.json DESTINATION ${PX4_PACKAGE_SHARE_DIR})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PX4_PACKAGE_SERVICE_NAME}.service DESTINATION /lib/systemd/system)

# CPack
set(CPACK_GENERATOR "DEB")
# Destinations are absolute, so stage under DESTDIR. The prefix only decides
# which directory CPack pre-creates; /usr is in the tree anyway.
set(CPACK_SET_DESTDIR ON)
set(CPACK_INSTALL_PREFIX "/usr")
set(CPACK_PACKAGE_NAME "${PX4_PACKAGE_NAME}")
set(CPACK_PACKAGE_VERSION "${PX4_PACKAGE_VERSION}")
set(CPACK_PACKAGE_VENDOR "${PX4_PACKAGE_BOARD_VENDOR}")
set(CPACK_PACKAGE_CONTACT "${PX4_PACKAGE_MAINTAINER}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PX4_PACKAGE_DESCRIPTION}")
if(EXISTS "${PX4_SOURCE_DIR}/LICENSE")
	set(CPACK_RESOURCE_FILE_LICENSE "${PX4_SOURCE_DIR}/LICENSE")
endif()

set(CPACK_DEBIAN_PACKAGE_NAME "${PX4_PACKAGE_NAME}")
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "${PX4_PACKAGE_ARCHITECTURE}")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "${PX4_PACKAGE_MAINTAINER}")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "${PX4_PACKAGE_DESCRIPTION}")
set(CPACK_DEBIAN_PACKAGE_SECTION "${PX4_PACKAGE_SECTION}")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "${PX4_PACKAGE_DEB_DEPENDS}")
set(CPACK_DEBIAN_PACKAGE_RECOMMENDS "${PX4_PACKAGE_DEB_RECOMMENDS}")
set(CPACK_DEBIAN_PACKAGE_CONFLICTS "${PX4_PACKAGE_DEB_CONFLICTS}")
set(CPACK_DEBIAN_PACKAGE_REPLACES "${PX4_PACKAGE_DEB_REPLACES}")
set(CPACK_DEBIAN_PACKAGE_PROVIDES "${PX4_PACKAGE_DEB_PROVIDES}")
# Cross builds cannot run dpkg-shlibdeps; the descriptor lists the dependencies.
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS OFF)
set(CPACK_DEBIAN_COMPRESSION_TYPE xz)
set(CPACK_DEBIAN_FILE_NAME "${PX4_PACKAGE_NAME}_${PX4_PACKAGE_VERSION}_${PX4_PACKAGE_ARCHITECTURE}.deb")
set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
	"${CMAKE_CURRENT_BINARY_DIR}/postinst;${CMAKE_CURRENT_BINARY_DIR}/prerm;${CMAKE_CURRENT_BINARY_DIR}/postrm")

message(STATUS "PX4 package: ${CPACK_DEBIAN_FILE_NAME}")

include(CPack)
