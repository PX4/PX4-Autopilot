
set(cpu_flags "-mcpu=cortex-a8 -mfpu=neon-vfpv3 -mfloat-abi=hard -mthumb-interwork")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${cpu_flags}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${cpu_flags}")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${cpu_flags} -D__ASSEMBLY__")
