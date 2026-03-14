############################################################################
#
#   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

# packaging

set(CPACK_PACKAGE_VENDOR "px4")
set(CPACK_PACKAGE_CONTACT "daniel@agar.ca")
set(CPACK_RESOURCE_FILE_LICENSE "${PX4_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${PX4_SOURCE_DIR}/README.md")

set(CPACK_SOURCE_GENERATOR "ZIP;TBZ2")

# Debian version: convert git describe to Debian-compliant format
# v1.17.0-beta1 -> 1.17.0~beta1, v1.17.0 -> 1.17.0
string(REGEX REPLACE "^v" "" DEB_VERSION "${PX4_GIT_TAG}")
# Replace first hyphen with tilde for pre-release (Debian sorts ~ before anything)
string(REGEX REPLACE "^([0-9]+\\.[0-9]+\\.[0-9]+)-([a-zA-Z])" "\\1~\\2" DEB_VERSION "${DEB_VERSION}")
# Strip any trailing commit info (e.g. -42-gabcdef)
string(REGEX REPLACE "-[0-9]+-g[0-9a-f]+$" "" DEB_VERSION "${DEB_VERSION}")

set(CPACK_PACKAGE_VERSION_MAJOR ${PX4_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PX4_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PX4_VERSION_PATCH})

if("${CMAKE_SYSTEM}" MATCHES "Linux")
	set(CPACK_GENERATOR "TBZ2")

	find_program(DPKG_PROGRAM dpkg)
	if(EXISTS ${DPKG_PROGRAM})
		list(APPEND CPACK_GENERATOR "DEB")

		execute_process(COMMAND ${DPKG_PROGRAM} --print-architecture
			OUTPUT_VARIABLE DEB_ARCHITECTURE OUTPUT_STRIP_TRAILING_WHITESPACE)

		# Detect Ubuntu/Debian codename for version suffix
		find_program(LSB_RELEASE lsb_release)
		if(EXISTS ${LSB_RELEASE})
			execute_process(COMMAND ${LSB_RELEASE} -cs
				OUTPUT_VARIABLE DEB_CODENAME OUTPUT_STRIP_TRAILING_WHITESPACE)
		else()
			set(DEB_CODENAME "unknown")
		endif()

		# Override CPACK_PACKAGE_VERSION with full Debian version.
		# CPack DEB ignores CPACK_PACKAGE_VERSION_MAJOR/MINOR/PATCH
		# when CPACK_PACKAGE_VERSION is set, so we must replace them.
		unset(CPACK_PACKAGE_VERSION_MAJOR)
		unset(CPACK_PACKAGE_VERSION_MINOR)
		unset(CPACK_PACKAGE_VERSION_PATCH)
		set(CPACK_PACKAGE_VERSION "${DEB_VERSION}-${DEB_CODENAME}")

		# Label-aware package metadata
		if(PX4_BOARD_LABEL STREQUAL "sih")
			set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/px4")
			set(CPACK_DEBIAN_PACKAGE_NAME "px4")
			set(CPACK_DEBIAN_FILE_NAME "px4_${DEB_VERSION}-${DEB_CODENAME}_${DEB_ARCHITECTURE}.deb")
			set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6, libstdc++6")
			set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "PX4 SITL autopilot with SIH physics (no Gazebo)")
			set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
				"${PX4_SOURCE_DIR}/Tools/packaging/sih/postinst;${PX4_SOURCE_DIR}/Tools/packaging/sih/postrm")
		else()
			set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/px4-gazebo")
			set(CPACK_DEBIAN_PACKAGE_NAME "px4-gazebo")
			set(CPACK_DEBIAN_FILE_NAME "px4-gazebo_${DEB_VERSION}-${DEB_CODENAME}_${DEB_ARCHITECTURE}.deb")
			set(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6, libstdc++6, gz-sim8-cli, libgz-sim8-plugins, libgz-physics7-dartsim, gz-tools2")
			set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "PX4 SITL autopilot with Gazebo Harmonic simulation resources")
			set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
				"${PX4_SOURCE_DIR}/Tools/packaging/postinst;${PX4_SOURCE_DIR}/Tools/packaging/postrm")
		endif()

		# Bake the install prefix into the px4 binary so it can locate its ROMFS
		# (etc/) without a wrapper script or command-line argument.
		if(TARGET px4)
			target_compile_definitions(px4 PRIVATE PX4_INSTALL_PREFIX="${CPACK_PACKAGING_INSTALL_PREFIX}")
		endif()

		set(CPACK_DEBIAN_PACKAGE_SECTION "misc")
		set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
		set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Daniel Agar <${CPACK_PACKAGE_CONTACT}>")

		set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
		set(CPACK_DEBIAN_COMPRESSION_TYPE xz)
		set(CPACK_DEBIAN_ARCHITECTURE ${DEB_ARCHITECTURE})

		message(STATUS "PX4 SITL .deb version: ${DEB_VERSION}-${DEB_CODENAME} (${DEB_ARCHITECTURE})")

	endif()
else()
	set(CPACK_GENERATOR "ZIP")
endif()

include(CPack)
