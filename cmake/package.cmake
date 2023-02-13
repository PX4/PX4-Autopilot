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

set(CPACK_PACKAGE_NAME ${PROJECT_NAME}-${PX4_CONFIG})

set(CPACK_PACKAGE_VENDOR "px4")

set(CPACK_PACKAGE_VERSION_MAJOR ${PX4_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${PX4_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${PX4_VERSION_PATCH})
#set(CPACK_PACKAGE_VERSION ${PX4_GIT_TAG})

set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${PX4_CONFIG}-${PX4_GIT_TAG}")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${PX4_CONFIG}-${PX4_GIT_TAG}-src")

set(CPACK_PACKAGE_CONTACT "daniel@agar.ca")

set(CPACK_RESOURCE_FILE_LICENSE "${PX4_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${PX4_SOURCE_DIR}/README.md")

set(CPACK_COMPONENTS_GROUPING ALL_COMPONENTS_IN_ONE)#ONE_PER_GROUP)
# without this you won't be able to pack only specified component
set(CPACK_DEB_COMPONENT_INSTALL YES)

#set(CPACK_STRIP_FILES YES)

set(CPACK_SOURCE_GENERATOR "ZIP;TBZ2")
set(CPACK_PACKAGING_INSTALL_PREFIX "")
set(CPACK_SET_DESTDIR "OFF")

if("${CMAKE_SYSTEM}" MATCHES "Linux")
	set(CPACK_GENERATOR "TBZ2")

	find_program(DPKG_PROGRAM dpkg)
	if(EXISTS ${DPKG_PROGRAM})
		list(APPEND CPACK_GENERATOR "DEB")

		set(CPACK_SET_DESTDIR true)
		set(CPACK_PACKAGING_INSTALL_PREFIX "/tmp")

		execute_process(COMMAND ${DPKG_PROGRAM} --print-architecture OUTPUT_VARIABLE DEB_ARCHITECTURE OUTPUT_STRIP_TRAILING_WHITESPACE)
		message("Architecture:  " ${DEB_ARCHITECTURE})
		set(CPACK_PACKAGE_FILE_NAME "${CPACK_PACKAGE_FILE_NAME}_${DEB_ARCHITECTURE}")

		set(CPACK_INSTALL_PREFIX @DEB_INSTALL_PREFIX@)
		message ("==> CPACK_INSTALL_PREFIX = " ${CPACK_INSTALL_PREFIX})

		################################################################################

		set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Daniel Agar <${CPACK_PACKAGE_CONTACT}>")
		set(CPACK_DEBIAN_PACKAGE_VERSION ${CPACK_PACKAGE_VERSION})
		set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)

		set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "PX4 autopilot")
		set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
		set(CPACK_DEBIAN_PACKAGE_SECTION "misc")
		set(CPACK_DEBIAN_ARCHITECTURE ${CMAKE_SYSTEM_PROCESSOR})

		# autogenerate dependency information
		set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
		set(CPACK_DEBIAN_COMPRESSION_TYPE xz)

	endif()
else()
	set(CPACK_GENERATOR "ZIP")
endif()

include(CPack)
