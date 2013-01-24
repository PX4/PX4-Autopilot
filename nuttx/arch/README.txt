Architecture-Specific Code
^^^^^^^^^^^^^^^^^^^^^^^^^^
Table of Contents
^^^^^^^^^^^^^^^^^

  o Architecture-Specific Code
  o Summary of Files
  o Supported Architectures
  o Configuring NuttX

Architecture-Specific Code
^^^^^^^^^^^^^^^^^^^^^^^^^^

The NuttX configuration consists of:

o Processor architecture specific files.  These are the files contained
  in the arch/<arch-name>/ directory discussed in this README.

o Chip/SoC specific files.  Each processor processor architecture
  is embedded in chip or System-on-a-Chip (SoC) architecture.  The
  full chip architecture includes the processor architecture plus
  chip-specific interrupt logic, general purpose I/O (GIO) logic, and
  specialized, internal peripherals (such as UARTs, USB, etc.).

  These chip-specific files are contained within chip-specific
  sub-directories in the arch/<arch-name>/ directory and are selected
  via the CONFIG_ARCH_name selection

o Board specific files.  In order to be usable, the chip must be
  contained in a board environment.  The board configuration defines
  additional properties of the board including such things as
  peripheral LEDs, external peripherals (such as network, USB, etc.).

  These board-specific configuration files can be found in the
  configs/<board-name>/ sub-directories.

This README will address the processor architecture specific files
that are contained in the arch/<arch-name>/ directory. The file
include/nuttx/arch.h identifies all of the APIs that must
be provided by this architecture specific logic.  (It also includes
arch/<arch-name>/arch.h as described below).

Directory Structure
^^^^^^^^^^^^^^^^^^^

The arch directory contains architecture specific logic.  The complete
board port in is defined by the architecture-specific code in this
directory (plus the board-specific configurations in the config/
subdirectory).  Each architecture must provide a subdirectory <arch-name>
under arch/ with the following characteristics:


        <arch-name>/
        |-- include/
        |   |--<chip-name>/
        |   |  `-- (chip-specific header files)
        |   |--<other-chips>/
        |   |-- arch.h
        |   |-- irq.h
        |   `-- types.h
        `-- src/
            |--<chip-name>/
            |  `-- (chip-specific source files)
            |--<other-chips>/
            |-- Makefile
            `-- (architecture-specific source files)

Summary of Files
^^^^^^^^^^^^^^^^

include/<chip-name>/
  This sub-directory contains chip-specific header files.

include/arch.h
  This is a hook for any architecture specific definitions that may
  be needed by the system.  It is included by include/nuttx/arch.h

include/types.h
  This provides architecture/toolchain-specific definitions for
  standard types.  This file should typedef:

    _int8_t, _uint8_t, _int16_t, _uint16_t, _int32_t, _uint32_t

  and if the architecture supports 64-bit integers.

    _int24_t, _uint24_t, int64_t, uint64_t

  NOTE that these type names have a leading underscore character.  This
  file will be included(indirectly) by include/stdint.h and typedef'ed to
  the final name without the underscore character.  This roundabout way of
  doings things allows the stdint.h to be removed from the include/
  directory in the event that the user prefers to use the definitions
  provided by their toolchain header files

    irqstate_t

  Must be defined to the be the size required to hold the interrupt
  enable/disable state.

  This file will be included by include/sys/types.h and be made
  available to all files.

include/irq.h
  This file needs to define some architecture specific functions (usually
  inline if the compiler supports inlining) and structure.  These include:

  - struct xcptcontext.  This structures represents the saved context
    of a thread.

  - irqstate_t irqsave(void) -- Used to disable all interrupts.

  - void irqrestore(irqstate_t flags) -- Used to restore interrupt
    enables to the same state as before irqsave was called.

  This file must also define NR_IRQS, the total number of IRQs supported
  by the board.

src/<chip-name>/
  This sub-directory contains chip-specific source files.

src/Makefile
  This makefile will be executed to build the targets src/libup.a and
  src/up_head.o.  The up_head.o file holds the entry point into the system
  (power-on reset entry point, for example).  It will be used in
  the final link with libup.a and other system archives to generate the
  final executable.

Supported Architectures
^^^^^^^^^^^^^^^^^^^^^^^

arch/sim - Linux/Cygwin simulation
    A user-mode port of NuttX to the x86 Linux platform is available.
    The purpose of this port is primarily to support OS feature development.
    This port does not support interrupts or a real timer (and hence no
    round robin scheduler)  Otherwise, it is complete.

arch/arm - ARM-based micro-controllers
    This directory holds common ARM architectures.  At present, this includes
    the following subdirectories:

    arch/arm/include and arch/arm/src/common
        Common ARM/Cortex-M3 logic.

    arch/arm/src/arm and arch/arm/include/arm
        Common ARM-specific logic

    arch/arm/src/armv7-m and arch/arm/include/armv7-m
        Common ARMv7-M logic (Cortex-M3 and Cortex-M4)

    arch/arm/include/c5471 and arch/arm/src/c5471
        TI TMS320C5471 (also called TMS320DM180 or just C5471).
        NuttX operates on the ARM7 of this dual core processor. This port
        complete, verified, and included in the NuttX release 0.1.1.

    arch/arm/include/calypso and arch/arm/src/calypso
        TI "Calypso" MCU used in various cell phones (and, in particular,
        by the Osmocom-bb project).  Like the c5471, NuttX operates on the
        ARM7 of this dual core processor. This port was contributed by
        Denis Carilki and includes the work of Denis, Alan Carvalho de Assis,
        and Stefan Richter.  Calypso support first appeared in NuttX-6.17.

    arch/arm/include/dm320 and arch/arm/src/dm320
        TI TMS320DM320 (also called just DM320).
        NuttX operates on the ARM9EJS of this dual core processor.  This port
        complete, verified, and included in the NuttX release 0.2.1.

    arch/arm/include/imx and arch/arm/src/imx
        Freescale MC9328MX1 or i.MX1. This port uses the Freescale MX1ADS
        development board with a GNU arm-elf toolchain* under either Linux or Cygwin.
        STATUS: This port has stalled because of development tool issues. Coding
        is complete on the basic port (timer, serial console, SPI).

    arch/arm/include/lm and arch/arm/src/lm
        These directories contain support for the Luminary LM3S/4F family. The 
        initial, release of this port was included in NuttX version 0.4.6.  The
        current port includes timer, serial console, Ethernet, SSI, and microSD
        support. There are working configurations the NuttX OS test, to run the
        NuttShell (NSH), the NuttX networking test, and the uIP web server.

    arch/arm/include/lpc214x and arch/arm/src/lpc214x
        These directories provide support for NXP LPC214x family of
        ARM7TDMI processors.  This port boots and passes the OS test (examples/ostest).
        The port is complete and verifed.  As of NuttX 0.3.17, the port includes:
        timer interrupts, serial console, USB driver, and SPI-based MMC/SD card
        support.  A verifed NuttShell (NSH) configuration is also available.

    arch/arm/include/lpc2378 and arch/arm/src/lpc2378.
        NXP LPC2378. Support is provided for the NXP LPC2378 MCU. This port was
        contributed by Rommel Marcelo is was first released in NuttX-5.3.
        STATUS: This port boots and passes the OS test (examples/ostest) and
        includes a working implementation of the NuttShell (NSH). The port is
        complete and verified. As of NuttX 5.3, the port includes only basic
        timer interrupts and serial console support.

    arch/arm/include/lpc31xx and arch/arm/src/lpc31xx
        These directories provide support for NXP LPC31xx family of
        ARM926EJ-S processors.  The port for the NXP LPC3131 was first
        released in NuttX-5.1 (but was not functional until NuttX-5.2).
        STATUS: The basic EA3131 port is complete and verified in NuttX-5.2
        This basic port includes basic boot-up, serial console, and timer
        interrupts. This port was extended in NuttX 5.3 with a USB high
        speed driver contributed by David Hewson. This port has been
        verified using the NuttX OS test, USB serial and mass storage tests
        and includes a working implementation of the NuttShell ((NSH)).

        This port was later extended to support additional members of the
        LPC31xx family including, specifically, the LPC3152.

    arch/arm/include/sam3u and arch/arm/src/sam3u
        Atmel AT91SAM3U. This port is for Atmel AT91SAM3U4E MCU.
        STATUS: The basic AT91SAM3U port was released in NuttX version 5.1.
        The basic port includes boot-up logic, interrupt driven serial
        console, and system timer interrupts. That release passes the
        NuttX OS test and is proven to have a valid OS implementation. A 
        onfiguration to support the NuttShell is also included.

    arch/arm/include/stm32 and arch/arm/src/stm32
        These directories contain support for the STMicro STM32 F1, F2, and
        F4 families.

        STATUS: The basic STM32 F1 port was released in NuttX version 0.4.12.
        and has continued to develop consistently over time.  It now includes
        support for the F2 and F4 families and a rich offering of peripheral
        drivers.

    arch/arm/include/str71x and arch/arm/src/str71x
        These directories provide support for the STMicro STR71x processors.
        Coding is complete on the basic port (boot logic, system time, serial console),
        but no testing has been performed due to some problems I am having with my
        JTAG wiggler and OpenOCD on Linux.

arch/avr
    This directory is dedicated to ports to the Atmel AVR (8-bit) and AVR32 (32-bit)
    MCU families.  STATUS:  Under development.

    arch/avr/include/avr and arch/avr/src/avr
      Common support for all 8-bit VR MCUs

    arch/avr/include/atmega and arch/avr/src/atmega
      Support specifically for the AVR ATMega family (specifically only for
      the ATMega128 at the moment).

    arch/avr/include/at90usb and arch/avr/src/at90usb
      Support specifically for the AVR AT90USB646, 647, 1286, and 1287 family.

    arch/avr/include/avr32 and arch/avr/src/avr32
      Common support for all AVR32 MCUs

    arch/avr/include/at32uc3 and arch/avr/src/at32uc3
      Support specifically for the AT32UC3Bxxx family (specifically only for
      the AT32UC3B0256 at the moment).

arch/hc
    This directory is dedicated to ports to the Freescale HC family.

    arch/arm/include/m9s12 and arch/arm/src/m9s12
      These directories provide support for the Freescale mc9s12x family.
      STATUS:  Fragments of this port were first released in nuttx-5.0 and
      the port was "code-complete" as nuttx-5.18.  However, the final
      verification effort has been stalled because of higher priority tasks.

arch/m68322
    A work in progress.
    STATUS:  Stalled for the time being.

arch/mips
    This directory is dedicated to ports to the MIPS family.

    arch/mips/include/mips32 and arch/mips/src/mips32
      Common support for all MIPS32 architectures

    arch/mips/include/pic32mx and arch/mips/src/pic32mx
      Support for all MicroChip PIC32MX architectures

arch/rgmp

    RGMP stands for RTOS and GPOS on Multi-Processor.  RGMP is a project
    for running GPOS and RTOS simultaneously on multi-processor platforms.
    You can port your favorite RTOS to RGMP together with an unmodified
    Linux to form a hybrid operating system. This makes your application
    able to use both RTOS and GPOS features.

    See http://rgmp.sourceforge.net/wiki/index.php/Main_Page for further
    information about RGMP.

arch/sh - SuperH and related Hitachi/Renesas microcontrollers

    arch/sh/include and arch/sh/src/common
        Common SuperH logic.

    arch/sh/include/shs and arch/sh/src/sh1
        Support for the SH-1 processor.

arch/8051 - 8051/52 microcontrollers
    8051 Microcontroller.  This port is not quite ready for prime time.

arch/x86 - Intel x86 architectures
    This directory holds related, 32- and 64-bit architectures from Intel.
    At present, this includes the following subdirectories:

    arch/x86/include and arch/x86/src/common
        Common x86 logic.

    arch/x86/include/i486 and arch/x86/src/i486
        These directories hold definitions and logic appropriate for any
        instantiation of the 32-bit i486 architecture.

    arch/x86/include/qemu and arch/x86/src/qemu
        This is the implementation of NuttX on the QEMU x86 simulation.

arch/z16 - ZiLOG 16-bit processors
    This directory holds related, 16-bit architectures from ZiLOG.  At
    present, this includes the following subdirectories:

    arch/z16/include and arch/z16/src/common
        Common microcontroller logic.

    arch/z16/include/z16f and arch/z16/src/z16f
        ZiLOG z16f Microcontroller.
        STATUS: Released in nuttx-0.3.7.  Fully functional other than issues
        addressed in ${TOPDIR}/TODO.

arch/z80 - ZiLOG 8-bit microcontrollers
    This directory holds related, 8-bit architectures from ZiLOG.  At
    present, this includes the following subdirectories:

    arch/z80/include and arch/z80/src/common
        Common microcontroller logic.

    arch/z80/include/z80 and arch/z80/src/z80
        Classic ZiLOG z80 Microcontroller.
        STATUS: Functional with no known defects.  There are still several
         OS features that have not yet been tested (e.g., networking).

    arch/z80/include/z8 and arch/z80/src/z8
        ZiLOG Z8Encore! Microcontroller

    arch/z80/include/ez80 and arch/z80/src/ez80
        ZiLOG ez80 Acclaim! Microcontroller
