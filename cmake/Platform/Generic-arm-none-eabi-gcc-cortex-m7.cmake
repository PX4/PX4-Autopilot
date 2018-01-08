
set(cpu_flags "-mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard")

set(CMAKE_C_FLAGS "${cpu_flags}")
set(CMAKE_CXX_FLAGS "${cpu_flags}")
set(CMAKE_ASM_FLAGS "${cpu_flags} -D__ASSEMBLY__")
