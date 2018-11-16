
set(cpu_flags "-mcpu=cortex-m4 -mthumb -march=armv7e-m -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

set(CMAKE_C_FLAGS "${cpu_flags}")
set(CMAKE_CXX_FLAGS "${cpu_flags}")
set(CMAKE_ASM_FLAGS "${cpu_flags} -D__ASSEMBLY__")
