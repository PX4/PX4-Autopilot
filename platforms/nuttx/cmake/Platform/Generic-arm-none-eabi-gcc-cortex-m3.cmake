
set(cpu_flags "-mcpu=cortex-m3 -mthumb -march=armv7-m")

set(CMAKE_C_FLAGS "${cpu_flags}")
set(CMAKE_CXX_FLAGS "${cpu_flags}")
set(CMAKE_ASM_FLAGS "${cpu_flags} -D__ASSEMBLY__")
