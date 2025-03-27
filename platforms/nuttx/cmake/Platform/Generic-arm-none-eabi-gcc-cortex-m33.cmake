# based on platforms/nuttx/NuttX/nuttx/arch/arm/src/armv8-m/Toolchain.defs
cmake_minimum_required(VERSION 3.13...3.27)
set(CONFIG_NUTTX_SIMPLIFIED_BUILD "1")

if(CONFIG_NUTTX_SIMPLIFIED_BUILD)
	message(STATUS "Simplified build to match NuttX")
	# NUTTX, .map file:
	# .data          0x00000000        0x0 .../thumb/v8-m.main/nofp/libgcc.a(_aeabi_uldivmod.o)
	#set(mfpu_type "none")
	# undefined instruction: set(extra_cpu_flags "-mlittle-endian -march=armv8-m.main -mtune=cortex-m33 -mfloat-abi=soft ")
	# same: set(extra_cpu_flags "-mlittle-endian -mcpu=cortex-m33  -mfloat-abi=soft  -march=armv8-m.main+nofp ")
	# not valid: nofp16 nosimd
	# -march=armv8-m.main+nofp+nodsp
	#set(extra_cpu_flags "-mlittle-endian -mcpu=cortex-m33 -mfloat-abi=soft ")
	# set(extra_cpu_flags "-mlittle-endian -mtune=cortex-m33  -march=armv8-m.main+nofp -mfloat-abi=soft ")
	# (gdb)
	#0x10000252	133	    div = (uint32_t) ((((uint64_t) src_freq) << 16) / (uint64_t)freq);
	#=> 0x10000252 <rp23xx_clock_configure+38>:	32 f1 55 fc	bl	0x10132b00 <____aeabi_uldivmod_veneer>
	#
	#(gdb)
	#0x10132b00 in ____aeabi_uldivmod_veneer ()
	#=> 0x10132b00 <____aeabi_uldivmod_veneer+0>:	5f f8 00 f0	ldr.w	pc, [pc]	; 0x10132b04 <____aeabi_uldivmod_veneer+4>
	#
	#(gdb)
	#0x20000bd4 in __aeabi_uldivmod ()
	#=> 0x20000bd4 <__aeabi_uldivmod+0>:	12 dd	ble.n	0x20000bfc <__aeabi_uldivmod+40>
	#
	#(gdb)
	#0x20000bd6 in __aeabi_uldivmod ()
	#=> 0x20000bd6 <__aeabi_uldivmod+2>:	cc a8	add	r0, sp, #816	; 0x330
	#
	#(gdb)
	#0x20000bd8 in __aeabi_uldivmod ()
	#=> 0x20000bd8 <__aeabi_uldivmod+4>:	9d b7			; <UNDEFINED> instruction: 0xb79d

	# set(extra_cpu_flags "-mlittle-endian -mcpu=cortex-m33  -march=armv8-m.main+nofp -mfloat-abi=soft ")


	# endless loop in => 0x2000b2c6:	00 00	movs	r0, r0
	# before clock configure was in: => 0x10000224 <rp23xx_clock_configure+28>:	0c 24	movs	r4, #12
	# it jumped to RAM!?
	set(extra_cpu_flags "-mlittle-endian -mcpu=cortex-m33 -mfloat-abi=softfp -mfpu=fpv5-sp-d16")



	# arm-none-eabi-gcc -c -Wstrict-prototypes -Wno-attributes -Wno-unknown-pragmas -Wno-psabi -Os
	# -fno-strict-aliasing -fomit-frame-pointer --param=min-pagesize=0 -fno-common -Wall -Wshadow -Wundef -ffunction-sections -fdata-sections "-g" -mlittle-endian -march=armv8-m.main -mtune=cortex-m33 -mfloat-abi=soft -mthumb -Wa,-mthumb -Wa,-mimplicit-it=always
    #  -isystem /Users/vidma/flyuav/px4_rpi/nuttx_orig/nuttx/include -D__NuttX__ -DNDEBUG  -pipe -I /Users/vidma/flyuav/px4_rpi/nuttx_orig/nuttx/libs/libc    misc/lib_utsname.c -o  bin/lib_utsname.o
else()
	# .data          0x00000000        0x0 .../thumb/v8-m.main+fp/hard/libgcc.a(_aeabi_uldivmod.o)
	if(CONFIG_ARCH_DPFPU)
		message(STATUS "Enabling double FP precision hardware instructions")
		set(mfpu_type "fpv5-d16")
	else()
		message(STATUS "Disabling double FP precision hardware instructions")
		set(mfpu_type "fpv5-sp-d16")
	endif()
	set(extra_cpu_flags "-mcpu=cortex-m33 -mfloat-abi=hard -march=armv8-m.main+fp+dsp -mfpu=${mfpu_type}")
endif()

# it seems these are needed both during linking AND compilation!
# FIXME: causes weight - Thread 1 "rp2350.dap.core0" received signal SIGTRAP, Trace/breakpoint trap.
#exception_common () at /Users/vidma/flyuav/px4_rpi/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/arch/arm/src/armv8-m/arm_exception.S:119
#119		mrs		r12, control				/* R12=control */
#=> 0x1000012e <exception_common+0>:	ef f3 14 8c	mrs	r12, CONTROL
#
#(gdb) bt
##0  exception_common () at /Users/vidma/flyuav/px4_rpi/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/arch/arm/src/armv8-m/arm_exception.S:119
##1  <signal handler called>
##2  0x00000000 in _GLOBAL_OFFSET_TABLE_ ()
#Backtrace stopped: previous frame identical to this frame (corrupt stack?)
#set(registers_flags "-mpic-register=r10 -msingle-pic-base -fpic")
set(registers_flags "")

# FIXME: -mno-unaligned-access
# -DLIB_PICO_BINARY_INFO=0
# arm-none-eabi-gcc -mthumb -mcpu=cortex-m33 -mfloat-abi=hard -print-multi-directory
# https://github.com/raspberrypi/pico-sdk/blob/ee68c78d0afae2b69c03ae1a72bf5cc267a2d94c/bazel/toolchain/BUILD.bazel#L35
# was: -march=armv8-m.main+dsp

# -mno-unaligned-access
# -mfloat-abi=softfp
# bad / unneeded params?: -DPICO_RP2040=0 -DPICO_RP2350=1 -DPICO_SECURE=0 -DPICO_PLATFORM=rp2350 -DLIB_PICO_BINARY_INFO=0 -D__ARM_ARCH_6M__=0 -DPICO_BOARD= -DPICO_CYW43_SUPPORTED=1
set(pico_flags "-DPICO_RP2040=0 -DPICO_RP2350=1 -DPICO_SECURE=0 -DPICO_PLATFORM=rp2350 -DLIB_PICO_BINARY_INFO=0 -D__ARM_ARCH_6M__=0")

set(cpu_flags "-mthumb ${extra_cpu_flags}  -mno-unaligned-access ${pico_flags}")
# https://developer.arm.com/documentation/100720/0200/CMSE-support
# __ARM_FEATURE_CMSE


set(CMAKE_C_FLAGS "${cpu_flags} ${registers_flags}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "${cpu_flags} ${registers_flags}" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS "${cpu_flags} -D__ASSEMBLY__" CACHE STRING "" FORCE)

#set(ARCHPICFLAGS  "-fpic -msingle-pic-base -mpic-register=r10")
# Define NXFLATLDFLAGS1 and NXFLATLDFLAGS2
#set(NXFLATLDFLAGS1 "-r -d -warn-common")
# FIXME!!! try to remove these ones; seem like these are on Nuttx only, but not on PX4 2040
# set(custom_linker_script "/Users/vidma/flyuav/px4_rpi/PX4-Autopilot/platforms/nuttx/NuttX/nuttx/binfmt/libnxflat/gnu-nxflat-pcrel.ld")
#set(NXFLATLDFLAGS2 "${NXFLATLDFLAGS1} ")

# Define LDNXFLATFLAGS
# set(LDNXFLATFLAGS "-e main -s 2048")
# Example: Applying linker flags to 'px4_autopilot' target
# FIXME: "-no-check-sections"
# "-e" "_entry_point"
# FIXME: "-T${custom_linker_script}"
# arm-none-eabi-g++: error: unrecognized command-line option '-s 2048': "-s 2048"
#
# FIXME: stack size !?
# FIXME!!! try to remove these ones; seem like these are on Nuttx only, but not on PX4 2040
# add_link_options("-T${custom_linker_script}" "-e main" "-mpic-register=r10" "-msingle-pic-base" "-fpic")
# "-r"  "-fpic" "-msingle-pic-base" "-mpic-register=r10"
# "-warn-common"
# "-d"
# FIXME: "-s" "2048"

# Verify that the linker script exists
#if(NOT EXISTS "${custom_linker_script}")
#    message(FATAL_ERROR "Linker script not found: ${custom_linker_script}")
#endif()
# target_link_options(raspberrypi_pico-2 PRIVATE "-T${TOPDIR}/binfmt/libnxflat/gnu-nxflat-pcrel.ld")

