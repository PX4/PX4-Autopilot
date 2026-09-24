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


# Build-time script (cmake -P): copies the upstream MAVLink definitions next to
# the staged external dialect so mavgen can resolve its <include>s. Runs after
# the mavlink submodule target, since a fresh checkout fetches it during the
# build, not at configure time.
#
# Arguments: -DUPSTREAM_DIR=<submodule message_definitions/v1.0>
#            -DSTAGING_DIR=<build staging directory>
#            -DDIALECT=<external dialect name>

foreach(arg UPSTREAM_DIR STAGING_DIR DIALECT)
	if(NOT ${arg})
		message(FATAL_ERROR "px4_stage_external_mavlink_dialect: ${arg} not set")
	endif()
endforeach()

if(EXISTS "${UPSTREAM_DIR}/${DIALECT}.xml")
	message(FATAL_ERROR "External MAVLink dialect '${DIALECT}' clashes with an upstream dialect name; rename the XML")
endif()

file(GLOB upstream_xmls "${UPSTREAM_DIR}/*.xml")

if(NOT upstream_xmls)
	message(FATAL_ERROR "No MAVLink definitions in ${UPSTREAM_DIR}; is the mavlink submodule checked out?")
endif()

# file(COPY) preserves timestamps and skips unchanged files
file(COPY ${upstream_xmls} DESTINATION "${STAGING_DIR}")
