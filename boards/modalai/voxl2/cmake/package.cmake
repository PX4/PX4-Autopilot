############################################################################
#
#   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

# VOXL2 board-specific CPack overrides
# Loaded after cmake/package.cmake sets up CPack defaults

# Derive Debian-compatible version from git tag (e.g. v1.17.0-alpha1-42-gabcdef -> 1.17.0~alpha1.42.gabcdef)
string(REGEX REPLACE "^v" "" _deb_ver "${PX4_GIT_TAG}")
string(REGEX REPLACE "-" "~" _deb_ver "${_deb_ver}" )
string(REGEX REPLACE "~([0-9]+)~" ".\\1." _deb_ver "${_deb_ver}")

# VOXL2 is always aarch64 regardless of build host
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
set(CPACK_DEBIAN_PACKAGE_NAME "voxl-px4")
set(CPACK_DEBIAN_FILE_NAME "voxl-px4_${_deb_ver}_arm64.deb")
set(CPACK_PACKAGING_INSTALL_PREFIX "/usr")
set(CPACK_INSTALL_PREFIX "/usr")
set(CPACK_SET_DESTDIR true)

set(CPACK_DEBIAN_PACKAGE_DEPENDS "libfc-sensor (>=1.0.10), voxl-px4-params (>=0.3.10), voxl3-system-image(>=0.0.2) | voxl2-system-image(>=1.5.4) | rb5-system-image(>=1.6.2), modalai-slpi(>=1.2.2) | modalai-adsp(>=1.0.5)")
set(CPACK_DEBIAN_PACKAGE_CONFLICTS "px4-rb5-flight")
set(CPACK_DEBIAN_PACKAGE_REPLACES "px4-rb5-flight")
set(CPACK_DEBIAN_PACKAGE_DESCRIPTION "PX4 Autopilot for ModalAI VOXL2")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "ModalAI <support@modalai.com>")

# Disable shlibdeps for cross-compiled boards
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS OFF)

set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA
	"${PX4_BOARD_DIR}/debian/postinst;${PX4_BOARD_DIR}/debian/prerm")

# Install rules are in boards/modalai/voxl2/cmake/install.cmake,
# included from platforms/posix/CMakeLists.txt where the px4 target exists.
