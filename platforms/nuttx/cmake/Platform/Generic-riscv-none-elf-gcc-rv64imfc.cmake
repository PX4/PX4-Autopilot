
# -mno-fdiv: AIFoundry ET-Minion (Erbium) has no hardware for FDIV.S and
# FSQRT.S; they raise an M-code emulation exception.
set(cpu_flags "-march=rv64imfc_zicsr_zifencei -mabi=lp64f -mcmodel=medany -mno-fdiv")

set(CMAKE_C_FLAGS "${cpu_flags}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${cpu_flags}" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS "${cpu_flags} -D__ASSEMBLY__" CACHE STRING "" FORCE)
