set(cpu_flags "-mlongcalls -Wno-format -Wno-unused-function -fdata-sections -Wno-pointer-arith")#-DDEBUG_BUILD
set(CMAKE_C_FLAGS "${cpu_flags}  -Wstrict-prototypes -Wshadow" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${cpu_flags} -nostdinc++ -Wshadow" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS "${cpu_flags} -D__ASSEMBLY__" CACHE STRING "" FORCE)
