# arm-linux-gnueabihf-gcc toolchain

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_VERSION 1)

set(triple arm-linux-gnueabihf)
set(CMAKE_LIBRARY_ARCHITECTURE ${triple})
set(TOOLCHAIN_PREFIX ${triple})

set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_C_COMPILER_TARGET ${triple})

set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++)
set(CMAKE_CXX_COMPILER_TARGET ${triple})

set(CMAKE_ASM_COMPILER ${CMAKE_C_COMPILER})

# compiler tools
foreach(tool nm ld objcopy ranlib strip)
	string(TOUPPER ${tool} TOOL)
	find_program(CMAKE_${TOOL} ${TOOLCHAIN_PREFIX}-${tool})
	if(CMAKE-${TOOL} MATCHES "NOTFOUND")
		message(FATAL_ERROR "could not find ${TOOLCHAIN_PREFIX}-${tool}")
	endif()
endforeach()

set(CMAKE_FIND_ROOT_PATH get_file_component(${CMAKE_C_COMPILER} PATH))
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# os tools
foreach(tool grep make)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} ${tool})
	if(NOT ${TOOL})
		message(FATAL_ERROR "could not find ${tool}")
	endif()
endforeach()

# optional compiler tools
foreach(tool gdb gdbtui)
	string(TOUPPER ${tool} TOOL)
	find_program(${TOOL} ${TOOLCHAIN_PREFIX}-${tool})
endforeach()
