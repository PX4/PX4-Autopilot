if(CONFIG_ARCH_ARMV8A)
  set(arch_flags "-march=armv8-a")
endif()

if(CONFIG_ARCH_CORTEX_A53)
  set(cpu_flags "-mcpu=cortex-a53")
endif()

if(CONFIG_ARCH_CORTEX_A55)
  set(cpu_flags "-mcpu=cortex-a55")
endif()

if(CONFIG_ARCH_CORTEX_A57)
  set(cpu_flags "-mcpu=cortex-a57")
endif()

if(CONFIG_ARCH_CORTEX_A72)
  set(cpu_flags "-mcpu=cortex-a72")
endif()

set(CMAKE_C_FLAGS "${arch_flags} ${cpu_flags}" -D_LDBL_EQ_DBL CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${arch_flags} ${cpu_flags}" -D_LDBL_EQ_DBL CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS "${arch_flags} ${cpu_flags} -D__ASSEMBLY__" CACHE STRING "" FORCE)
