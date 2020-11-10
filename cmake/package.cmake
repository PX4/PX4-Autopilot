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
set(CPACK_PACKAGE_VERSION ${PX4_GIT_TAG})
set(CPACK_PACKAGE_CONTACT ${package-contact})
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS OFF) # TODO: review packaging for linux boards
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "The PX4 Pro autopilot.")
set(CPACK_PACKAGE_FILE_NAME "${PROJECT_NAME}-${PX4_CONFIG}-${PX4_GIT_TAG}")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "${PROJECT_NAME}-${PX4_GIT_TAG}")
set(CPACK_SOURCE_GENERATOR "ZIP;TBZ2")
set(CPACK_PACKAGING_INSTALL_PREFIX "")
set(CPACK_SET_DESTDIR "OFF")

if ("${CMAKE_SYSTEM}" MATCHES "Linux")
	set(CPACK_GENERATOR "TBZ2")
	find_program(DPKG_PROGRAM dpkg)
	if (EXISTS ${DPKG_PROGRAM})
		list (APPEND CPACK_GENERATOR "DEB")
	endif()
else()
	set(CPACK_GENERATOR "ZIP")
endif()

include(CPack)
include(bloaty)
