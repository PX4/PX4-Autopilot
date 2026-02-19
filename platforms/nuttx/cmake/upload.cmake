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

# Uploader script auto-detects PX4 devices by USB VID/PID
set(PX4_UPLOADER_SCRIPT "${PX4_SOURCE_DIR}/Tools/px4_uploader.py")
if (NOT DEFINED UPLOAD_FIRMWARE_FILES)
	set(UPLOAD_FIRMWARE_FILES ${fw_package})
endif()
if (NOT DEFINED UPLOAD_FIRMWARE_PRIMARY_FILE)
	set(UPLOAD_FIRMWARE_PRIMARY_FILE ${fw_package})
endif()
if (NOT DEFINED UPLOAD_FIRMWARE_SECONDARY_FILE)
	set(UPLOAD_FIRMWARE_SECONDARY_FILE "")
endif()

add_custom_target(upload
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_UPLOADER_SCRIPT} --update-mode primary ${UPLOAD_FIRMWARE_PRIMARY_FILE}
	DEPENDS ${UPLOAD_FIRMWARE_PRIMARY_FILE}
	COMMENT "uploading px4 primary firmware"
	VERBATIM
	USES_TERMINAL
	WORKING_DIRECTORY ${PX4_BINARY_DIR}
)

add_custom_target(force-upload
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_UPLOADER_SCRIPT} --force --update-mode primary ${UPLOAD_FIRMWARE_PRIMARY_FILE}
	DEPENDS ${UPLOAD_FIRMWARE_PRIMARY_FILE}
	COMMENT "uploading px4 primary firmware with --force"
	VERBATIM
	USES_TERMINAL
	WORKING_DIRECTORY ${PX4_BINARY_DIR}
)

add_custom_target(upload-verbose
	COMMAND ${PYTHON_EXECUTABLE} ${PX4_UPLOADER_SCRIPT} --verbose --update-mode primary ${UPLOAD_FIRMWARE_PRIMARY_FILE}
	DEPENDS ${UPLOAD_FIRMWARE_PRIMARY_FILE}
	COMMENT "uploading px4 primary firmware with verbose output"
	VERBATIM
	USES_TERMINAL
	WORKING_DIRECTORY ${PX4_BINARY_DIR}
)

if (UPLOAD_FIRMWARE_SECONDARY_FILE)
	add_custom_target(upload-secondary
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_UPLOADER_SCRIPT} --update-mode secondary ${UPLOAD_FIRMWARE_FILES}
		DEPENDS ${UPLOAD_FIRMWARE_FILES}
		COMMENT "uploading px4 secondary firmware"
		VERBATIM
		USES_TERMINAL
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
	)

	add_custom_target(upload-both
		COMMAND ${PYTHON_EXECUTABLE} ${PX4_UPLOADER_SCRIPT} --update-mode both ${UPLOAD_FIRMWARE_FILES}
		DEPENDS ${UPLOAD_FIRMWARE_FILES}
		COMMENT "uploading px4 primary and secondary firmware"
		VERBATIM
		USES_TERMINAL
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
	)
endif()
