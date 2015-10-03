# defines:
#
# NM
# OBJCOPY
# LD
# CXX_COMPILER
# C_COMPILER
# CMAKE_SYSTEM_NAME
# CMAKE_SYSTEM_VERSION
# GENROMFS
# LINKER_FLAGS
# CMAKE_EXE_LINKER_FLAGS
# CMAKE_FIND_ROOT_PATH
# CMAKE_FIND_ROOT_PATH_MODE_PROGRAM
# CMAKE_FIND_ROOT_PATH_MODE_LIBRARY
# CMAKE_FIND_ROOT_PATH_MODE_INCLUDE

include(CMakeForceCompiler)

# this one is important
set(CMAKE_SYSTEM_NAME Generic)

#this one not so much
set(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
find_program(C_COMPILER arm-none-eabi-gcc)
if(NOT C_COMPILER)
	message(FATAL_ERROR "could not find arm-none-eabi-gcc compiler")
endif()
cmake_force_c_compiler(${C_COMPILER} GNU)

find_program(CXX_COMPILER arm-none-eabi-g++)
if(NOT CXX_COMPILER)
	message(FATAL_ERROR "could not find arm-none-eabi-g++ compiler")
endif()
cmake_force_cxx_compiler(${CXX_COMPILER} GNU)

set(LINKER_FLAGS "-Wl,-gc-sections")
set(CMAKE_EXE_LINKER_FLAGS ${LINKER_FLAGS})

# where is the target environment 
set(CMAKE_FIND_ROOT_PATH  get_file_component(${C_COMPILER} PATH))

# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

set(warnings
	-Wall
	-Wno-sign-compare
	-Wextra
	#-Wshadow # very verbose due to eigen
	-Wfloat-equal
	-Wpointer-arith
	-Wmissing-declarations
	-Wpacked
	-Wno-unused-parameter
	-Werror=format-security
	-Werror=array-bounds
	-Wfatal-errors
	-Werror=unused-variable
	-Werror=reorder
	-Werror=uninitialized
	-Werror=init-self
	#-Wcast-qual  - generates spurious noreturn attribute warnings,
	#               try again later
	#-Wconversion - would be nice, but too many "risky-but-safe"
	#               conversions in the code
	#-Wcast-align - would help catch bad casts in some cases,
	#               but generates too many false positives
	)

set(max_optimization -Os)

set(optimization_flags
	-fno-strict-aliasing
	-fomit-frame-pointer
	-funsafe-math-optimizations
	-ffunction-sections
	-fdata-sections
	)

set(c_warnings
	-Wbad-function-cast
	-Wstrict-prototypes
	-Wmissing-prototypes
	-Wnested-externs
	)

set(c_compile_flags
	-g
	-fno-common
	-nodefaultlibs
	-nostdlib
	 -DCONFIG_WCHAR_BUILTIN
	)

set(cxx_warnings
	-Wno-missing-field-initializers
	)
set(cxx_compile_flags
	-g
	-fno-exceptions
	-fno-rtti
	-fno-threadsafe-statics
	 -DCONFIG_WCHAR_BUILTIN
	-nodefaultlibs
	-nostdlib
	)

set(cpu_flags
	-mcpu=cortex-m4
	-mthumb
	-march=armv7e-m
	-mfpu=fpv4-sp-d16
	-mfloat-abi=hard
	)

find_path(NUTTX_EXPORT_DIR libs/libnuttx.a
	PATHS
	$ENV{HOME}/git/px4/Firmware/build_px4fmu-v2_default/px4fmu-v2/NuttX/nuttx-export
	NO_DEFAULT_PATH
	NO_CMAKE_FIND_ROOT_PATH
	)
if(NOT NUTTX_EXPORT_DIR)
	message(FATAL_ERROR "failed to find NUTTX_EXPORT_DIR, please set")
else()
	message(STATUS "nuttx export: ${NUTTX_EXPORT_DIR}")
endif()

set(uavcan_extra_flags
	-DUAVCAN_NO_ASSERTIONS
	-DUAVCAN_STM32_NUM_IFACES=2
	-DUAVCAN_USE_EXTERNAL_SNPRINT
	-DUAVCAN_MEM_POOL_BLOCK_SIZE=48
	-DUAVCAN_MAX_NETWORK_SIZE_HINT=16
	-DUAVCAN_STM32_TIMER_NUMBER=5
	-DUAVCAN_STM32_NUTTX=1
	-DUAVCAN_CPP_VERSION=UAVCAN_CPP03
	-I${NUTTX_EXPORT_DIR}/include
	-I${NUTTX_EXPORT_DIR}/include/cxx
	-I${NUTTX_EXPORT_DIR}/arch/chip
	-I${NUTTX_EXPORT_DIR}/arch/common
	)

set(added_c_flags
	${cpu_flags}
	${c_compile_flags}
	${warnings}
	${c_warnings}
	${max_optimization}
	${optimization_flags}
	${uavcan_extra_flags}
	)

set(added_cxx_flags
	${cpu_flags}
	${cxx_compile_flags}
	${warnings}
	${cxx_warnings}
	${max_optimization}
	${optimization_flags}
	${uavcan_extra_flags}
	)

string (REPLACE ";" " " DEFAULT_CMAKE_CXX_FLAGS "${added_cxx_flags}")
string (REPLACE ";" " " DEFAULT_CMAKE_C_FLAGS "${added_c_flags}")
set(DEFAULT_CMAKE_BUILD_TYPE "RelWithDebInfo")
set(DEFAULT_UAVCAN_PLATFORM "stm32")
set(DEFAULT_UAVCAN_USE_CPP03 ON)
