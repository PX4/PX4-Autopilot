# # arm-none-eabi-gcc toolchain

# set(CMAKE_SYSTEM_NAME Generic)
# set(CMAKE_SYSTEM_VERSION 1)

# set(triple arm-none-eabi)
# set(CMAKE_LIBRARY_ARCHITECTURE ${triple})
# set(TOOLCHAIN_PREFIX ${triple})

# set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
# set(CMAKE_C_COMPILER_TARGET ${triple})

# set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++)
# set(CMAKE_CXX_COMPILER_TARGET ${triple})

# set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}-gcc)

# # needed for test compilation
# set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nosys.specs")

# # compiler tools
# find_program(CMAKE_AR ${TOOLCHAIN_PREFIX}-gcc-ar)
# find_program(CMAKE_GDB ${TOOLCHAIN_PREFIX}-gdb)
# find_program(CMAKE_LD ${TOOLCHAIN_PREFIX}-ld)
# find_program(CMAKE_LINKER ${TOOLCHAIN_PREFIX}-ld)
# find_program(CMAKE_NM ${TOOLCHAIN_PREFIX}-gcc-nm)
# find_program(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}-objcopy)
# find_program(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}-objdump)
# find_program(CMAKE_RANLIB ${TOOLCHAIN_PREFIX}-gcc-ranlib)
# find_program(CMAKE_STRIP ${TOOLCHAIN_PREFIX}-strip)

# set(CMAKE_FIND_ROOT_PATH get_file_component(${CMAKE_C_COMPILER} PATH))
# set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
# set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# # os tools
# foreach(tool grep make)
# 	string(TOUPPER ${tool} TOOL)
# 	find_program(${TOOL} ${tool})
# 	if(NOT ${TOOL})
# 		message(FATAL_ERROR "could not find ${tool}")
# 	endif()
# endforeach()

# arm-none-eabi-gcc toolchain

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)

set(triple arm-none-eabi)
set(CMAKE_LIBRARY_ARCHITECTURE ${triple})
set(TOOLCHAIN_PREFIX ${triple})

# Define the path to the new toolchain
set(NEW_TOOLCHAIN_PATH "/home/sindre/Dropbox/Studier/2025_Var/dev/PX4-Autopilot-public/src/lib/tflm/tflite_micro/tensorflow/lite/micro/tools/make/downloads/gcc_embedded/bin")

# Set the compiler paths
set(CMAKE_C_COMPILER ${NEW_TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${NEW_TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}-g++)
set(CMAKE_ASM_COMPILER ${NEW_TOOLCHAIN_PATH}/${TOOLCHAIN_PREFIX}-gcc)

# needed for test compilation
set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nosys.specs")

# compiler tools
find_program(CMAKE_AR ${TOOLCHAIN_PREFIX}-gcc-ar PATHS ${NEW_TOOLCHAIN_PATH})
find_program(CMAKE_GDB ${TOOLCHAIN_PREFIX}-gdb PATHS ${NEW_TOOLCHAIN_PATH})
find_program(CMAKE_LD ${TOOLCHAIN_PREFIX}-ld PATHS ${NEW_TOOLCHAIN_PATH})
find_program(CMAKE_LINKER ${TOOLCHAIN_PREFIX}-ld PATHS ${NEW_TOOLCHAIN_PATH})
find_program(CMAKE_NM ${TOOLCHAIN_PREFIX}-gcc-nm PATHS ${NEW_TOOLCHAIN_PATH})
find_program(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}-objcopy PATHS ${NEW_TOOLCHAIN_PATH})
find_program(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}-objdump PATHS ${NEW_TOOLCHAIN_PATH})
find_program(CMAKE_RANLIB ${TOOLCHAIN_PREFIX}-gcc-ranlib PATHS ${NEW_TOOLCHAIN_PATH})
find_program(CMAKE_STRIP ${TOOLCHAIN_PREFIX}-strip PATHS ${NEW_TOOLCHAIN_PATH})

set(CMAKE_FIND_ROOT_PATH ${NEW_TOOLCHAIN_PATH})
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
