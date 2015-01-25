include(CMakeForceCompiler)

# this one is important
set(CMAKE_SYSTEM_NAME Arm)

#this one not so much
set(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
find_program(C_COMPILER arm-none-eabi-gcc)
cmake_force_c_compiler(${C_COMPILER} GNU)
find_program(CXX_COMPILER arm-none-eabi-g++)
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
