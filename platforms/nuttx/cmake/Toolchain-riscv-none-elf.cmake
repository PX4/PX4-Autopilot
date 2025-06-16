# riscv64-unknown-elf toolchain

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(PLATFORM_NAME "nuttx")

set(triple riscv-none-elf)
set(CMAKE_LIBRARY_ARCHITECTURE ${triple})
set(TOOLCHAIN_PREFIX ${triple})

set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_C_COMPILER_TARGET ${triple})

set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++)
set(CMAKE_CXX_COMPILER_TARGET ${triple})

set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}-gcc)

# needed for test compilation
set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nosys.specs")

# compiler tools
find_program(CMAKE_AR ${TOOLCHAIN_PREFIX}-gcc-ar)
find_program(CMAKE_GDB ${TOOLCHAIN_PREFIX}-gdb)
find_program(CMAKE_LD ${TOOLCHAIN_PREFIX}-ld)
find_program(CMAKE_LINKER ${TOOLCHAIN_PREFIX}-ld)
find_program(CMAKE_NM ${TOOLCHAIN_PREFIX}-gcc-nm)
find_program(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}-objcopy)
find_program(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}-objdump)
find_program(CMAKE_RANLIB ${TOOLCHAIN_PREFIX}-gcc-ranlib)
find_program(CMAKE_STRIP ${TOOLCHAIN_PREFIX}-strip)

set(CMAKE_RUST_TARGET riscv64gc-unknown-none-elf)
set(CMAKE_RUST_COMPILER rustc)
set(CMAKE_RUST_COMPILER_FLAGS --target ${CMAKE_RUST_TARGET})

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

# Set architecture and ABI for RV64
set(CMAKE_C_FLAGS "-march=rv64gc -mabi=lp64d -mcmodel=medany ${CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS "-march=rv64gc -mabi=lp64d -mcmodel=medany ${CMAKE_CXX_FLAGS}")
set(CMAKE_ASM_FLAGS "-march=rv64gc -mabi=lp64d -mcmodel=medany ${CMAKE_ASM_FLAGS}")

# Ensure the linker uses the correct emulation
set(CMAKE_EXE_LINKER_FLAGS_INIT "-march=rv64gc -mabi=lp64d -mcmodel=medany -Wl,-melf64lriscv ${CMAKE_EXE_LINKER_FLAGS_INIT}")
