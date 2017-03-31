# This cmake config is for QuRT which is the RTOS running on the Hexagon DSP
include(qurt/px4_impl_qurt)

if ("$ENV{HEXAGON_SDK_ROOT}" STREQUAL "")
	message(FATAL_ERROR "Enviroment variable HEXAGON_SDK_ROOT must be set")
else()
	set(HEXAGON_SDK_ROOT $ENV{HEXAGON_SDK_ROOT})
endif()

set(CONFIG_SHMEM "1")

# Get $QC_SOC_TARGET from environment if existing.
if (DEFINED ENV{QC_SOC_TARGET})
	set(QC_SOC_TARGET $ENV{QC_SOC_TARGET})
else()
	set(QC_SOC_TARGET "APQ8074")
endif()

# Disable the creation of the parameters.xml file by scanning individual
# source files, and scan all source files.  This will create a parameters.xml
# file that contains all possible parameters, even if the associated module
# is not used.  This is necessary for parameter synchronization between the 
# ARM and DSP processors.
set(DISABLE_PARAMS_MODULE_SCOPING TRUE)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PX4_SOURCE_DIR}/src/lib/DriverFramework/dspal/cmake_hexagon")
include(toolchain/Toolchain-qurt)
include(qurt_flags)
include_directories(${HEXAGON_SDK_INCLUDES})

add_definitions(
   -D__PX4_QURT
   )

