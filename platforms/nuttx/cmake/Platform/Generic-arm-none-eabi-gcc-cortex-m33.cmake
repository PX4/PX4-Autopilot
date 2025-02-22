# based on platforms/nuttx/NuttX/nuttx/arch/arm/src/armv8-m/Toolchain.defs
cmake_minimum_required(VERSION 3.13...3.27)
if(CONFIG_ARCH_DPFPU)
	message(STATUS "Enabling double FP precision hardware instructions")
	set(mfpu_type "fpv5-d16")
else()
    message(STATUS "Disabling double FP precision hardware instructions")
	set(mfpu_type "fpv5-sp-d16")
endif()

# -DLIB_PICO_BINARY_INFO=0
set(cpu_flags "-mtune=cortex-m33 -mthumb -march=armv8-m.main+dsp -mfpu=${mfpu_type} -DPICO_RP2040=0 -DPICO_RP2350=1 -DPICO_SECURE=1 -DPICO_PLATFORM=rp2350 -DPICO_CYW43_SUPPORTED=1 -DLIB_PICO_BINARY_INFO=0 -D__ARM_ARCH_6M__=0")

set(CMAKE_C_FLAGS "${cpu_flags}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${cpu_flags}" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS "${cpu_flags} -D__ASSEMBLY__" CACHE STRING "" FORCE)

#set(ARCHPICFLAGS  "-fpic -msingle-pic-base -mpic-register=r10")
# Define NXFLATLDFLAGS1 and NXFLATLDFLAGS2
#set(NXFLATLDFLAGS1 "-r -d -warn-common")
# FIXME!!!
set(custom_linker_script "/Users/vidma/flyuav/px4_rpi/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/binfmt/libnxflat/gnu-nxflat-pcrel.ld")
#set(NXFLATLDFLAGS2 "${NXFLATLDFLAGS1} ")

# Define LDNXFLATFLAGS
# set(LDNXFLATFLAGS "-e main -s 2048")
# Example: Applying linker flags to 'px4_autopilot' target
# FIXME: "-no-check-sections"
# "-e" "_entry_point"
# FIXME: "-T${custom_linker_script}"
add_link_options("-T${custom_linker_script}" "-mpic-register=r10" "-msingle-pic-base" "-fpic")
# "-r"  "-fpic" "-msingle-pic-base" "-mpic-register=r10"
# "-warn-common"
# "-d"
# FIXME: "-s" "2048"

# Verify that the linker script exists
#if(NOT EXISTS "${custom_linker_script}")
#    message(FATAL_ERROR "Linker script not found: ${custom_linker_script}")
#endif()
# target_link_options(raspberrypi_pico-2 PRIVATE "-T${TOPDIR}/binfmt/libnxflat/gnu-nxflat-pcrel.ld")

