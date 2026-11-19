############################################################################
#
# Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#=============================================================================
#
#	Release stage of the build, and the MAVLink diagnostics that follow from it.
#
#	PX4_RELEASE_STAGE is taken from the git tag (PX4_GIT_TAG), unless it is
#	set with -DPX4_RELEASE_STAGE=<stage> (or make <target> RELEASE_STAGE=<stage>):
#
#		dev      anything but an exact beta or release tag: main, daily
#		         builds, commits after a tag, builds without a tag (v0.0.0)
#		beta     exactly a vX.Y.Z-alphaN or vX.Y.Z-betaN tag
#		release  exactly a vX.Y.Z-rcN or vX.Y.Z tag
#
#	MAVLink WIP definitions: code guarded by PX4_MAVLINK_WIP() (see the
#	generated mavlink_wip.h) is compiled in dev builds only. In beta and
#	release builds it is compiled out, and any unguarded use of a WIP
#	message or enum entry fails to compile.
#
#	PX4_MAVLINK_WIP_LEGACY (default ON) lets the uses listed in
#	src/modules/mavlink/mavlink_wip_legacy.yaml ship unguarded.
#
#	Writes ${PX4_BINARY_DIR}/px4_mavlink_diagnostics.h, which
#	px4_add_common_flags() force-includes in every source.
#

set(PX4_RELEASE_STAGE "" CACHE STRING "Release stage: dev, beta or release (empty: from the git tag)")
set_property(CACHE PX4_RELEASE_STAGE PROPERTY STRINGS "" dev beta release)
option(PX4_MAVLINK_WIP_LEGACY "Let the MAVLink WIP uses in src/modules/mavlink/mavlink_wip_legacy.yaml ship unguarded" ON)

if(PX4_RELEASE_STAGE)
	if(NOT PX4_RELEASE_STAGE MATCHES "^(dev|beta|release)$")
		message(FATAL_ERROR "PX4_RELEASE_STAGE must be dev, beta or release, not '${PX4_RELEASE_STAGE}'")
	endif()

elseif(PX4_GIT_TAG STREQUAL "v0.0.0")
	# no tag found (e.g. a shallow clone): CMakeLists.txt falls back to v0.0.0
	set(PX4_RELEASE_STAGE "dev")

elseif(PX4_GIT_TAG MATCHES "^v[0-9]+\\.[0-9]+\\.[0-9]+(-rc[0-9]+)?$")
	set(PX4_RELEASE_STAGE "release")

elseif(PX4_GIT_TAG MATCHES "^v[0-9]+\\.[0-9]+\\.[0-9]+-(alpha|beta)[0-9]+$")
	set(PX4_RELEASE_STAGE "beta")

else()
	set(PX4_RELEASE_STAGE "dev")
endif()

if(PX4_RELEASE_STAGE STREQUAL "dev")
	set(PX4_MAVLINK_WIP_ENABLED 1)
else()
	set(PX4_MAVLINK_WIP_ENABLED 0)
endif()

if(PX4_MAVLINK_WIP_LEGACY)
	set(PX4_MAVLINK_WIP_LEGACY_ENABLED 1)
else()
	set(PX4_MAVLINK_WIP_LEGACY_ENABLED 0)
endif()

message(STATUS "PX4_RELEASE_STAGE: ${PX4_RELEASE_STAGE} (MAVLink WIP code compiled in: ${PX4_MAVLINK_WIP_ENABLED}, legacy WIP uses allowed: ${PX4_MAVLINK_WIP_LEGACY_ENABLED})")

configure_file(${CMAKE_CURRENT_LIST_DIR}/px4_mavlink_diagnostics.h.in ${PX4_BINARY_DIR}/px4_mavlink_diagnostics.h @ONLY)
