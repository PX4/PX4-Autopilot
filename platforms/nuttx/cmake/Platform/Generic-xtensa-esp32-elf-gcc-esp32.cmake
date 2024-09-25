# set(cpu_flags "-mlongcalls -Wno-unused-function -Wno-format -fstrict-volatile-bitfields")#-DDEBUG_BUILD
set(cpu_flags "-mlongcalls -Wno-format -Wno-unused-function")#-DDEBUG_BUILD
# set(cpu_flags "")

set(CMAKE_C_FLAGS "${cpu_flags}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${cpu_flags}" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS "${cpu_flags} -D__ASSEMBLY__" CACHE STRING "" FORCE)
