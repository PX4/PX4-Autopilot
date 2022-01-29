set(cpu_flags "-mcpu=cortex-m0plus -mthumb -mfloat-abi=soft")			# Nuttx toochain uses -mfloat-abi=soft

set(CMAKE_C_FLAGS "${cpu_flags}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${cpu_flags}" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS "${cpu_flags} -D__ASSEMBLY__" CACHE STRING "" FORCE)
