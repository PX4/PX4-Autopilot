############################################################################
#
# Copyright (c) 2022 ModalAI, Inc. All rights reserved.
#
############################################################################
#
# This cmake config builds for QURT which is the operating system running on
# the DSP side of VOXL 2
#
# Required environment variables:
#	HEXAGON_TOOLS_ROOT
#	HEXAGON_SDK_ROOT
#
############################################################################

if ("$ENV{HEXAGON_SDK_ROOT}" STREQUAL "")
	message(FATAL_ERROR "Enviroment variable HEXAGON_SDK_ROOT must be set")
else()
	set(HEXAGON_SDK_ROOT $ENV{HEXAGON_SDK_ROOT})
endif()

if ("$ENV{HEXAGON_TOOLS_ROOT}" STREQUAL "")
	message(FATAL_ERROR "Environment variable HEXAGON_TOOLS_ROOT must be set")
else()
	set(HEXAGON_TOOLS_ROOT $ENV{HEXAGON_TOOLS_ROOT})
endif()

include(px4_git)

list(APPEND CMAKE_MODULE_PATH
	"${PX4_SOURCE_DIR}/platforms/qurt/cmake"
)

include(Toolchain-qurt)
include(qurt_reqs)

include_directories(${HEXAGON_SDK_INCLUDES})

add_definitions(-DORB_COMMUNICATOR)

set(CONFIG_PARAM_CLIENT "1")

set(DISABLE_PARAMS_MODULE_SCOPING TRUE)
