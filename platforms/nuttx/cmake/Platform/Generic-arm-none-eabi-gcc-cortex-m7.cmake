
if (CONFIG_ARCH_DPFPU)
	message(STATUS "Enabling double FP precision hardware instructions")
	set(mfpu_type "fpv5-d16")
else()
	set(mfpu_type "fpv5-sp-d16")
endif()

set(cpu_flags "-mcpu=cortex-m7 -mthumb -mfpu=${mfpu_type} -mfloat-abi=hard")

set(CMAKE_C_FLAGS "${cpu_flags}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${cpu_flags}" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS "${cpu_flags} -D__ASSEMBLY__" CACHE STRING "" FORCE)
