README
^^^^^^

  This README discusses issues unique to NuttX configurations for the
  Freescale DEMO9S12NE64 development board.

CONTENTS
^^^^^^^^
  • MC9S12NE64 Features
  • Development Environment
  • NuttX Buildroot Toolchain
  • FreeScale HCS12 Serial Monitor
  • Soft Registers
  • HCS12/DEMO9S12NEC64-specific Configuration Options
  • Configurations

MC9S12NE64 Features
^^^^^^^^^^^^^^^^^^^

  • 16-bit HCS12 core
    - HCS12 CPU
    - Upward compatible with M68HC11 instruction set
    - Interrupt stacking and programmer’s model identical to M68HC11
    - Instruction queue
    - Enhanced indexed addressing
    - Memory map and interface (MMC)
    - Interrupt control (INT)
    - Background debug mode (BDM)
    - Enhanced debug12 module, including breakpoints and change-of-flow
      trace buffer (DBG)
    - Multiplexed expansion bus interface (MEBI) - available only in
      112-pin package version
  • Wakeup interrupt inputs
    - Up to 21 port bits available for wakeup interrupt function with
      digital filtering
  • Memory
    - 64K bytes of FLASH EEPROM
    - 8K bytes of RAM
  • Analog-to-digital converter (ATD)
    - One 8-channel module with 10-bit resolution
    - External conversion trigger capability
  • Timer module (TIM)
    - 4-channel timer
    - Each channel configurable as either input capture or output
      compare
    - Simple PWM mode
    - Modulo reset of timer counter
    - 16-bit pulse accumulator
    - External event counting
    - Gated time accumulation
  • Serial interfaces
    - Two asynchronous serial communications interface (SCI)
    - One synchronous serial peripheral interface (SPI)
    - One inter-IC bus (IIC)
  • Ethernet Media access controller (EMAC)
    - IEEE 802.3 compliant
    - Medium-independent interface (MII)
    - Full-duplex and half-duplex modes
    - Flow control using pause frames
    - MII management function
    - Address recognition
    - Frames with broadcast address are always accepted or always
      rejected
    - Exact match for single 48-bit individual (unicast) address
    - Hash (64-bit hash) check of group (multicast) addresses
    - Promiscuous mode
  • Ethertype filter
  • Loopback mode
  • Two receive and one transmit Ethernet buffer interfaces
  • Ethernet 10/100 Mbps transceiver (EPHY)
    - IEEE 802.3 compliant
    - Digital adaptive equalization
    - Half-duplex and full-duplex
    - Auto-negotiation next page ability
    - Baseline wander (BLW) correction
    - 125-MHz clock generator and timing recovery
    - Integrated wave-shaping circuitry
    - Loopback modes
  • CRG (clock and reset generator module)
    - Windowed COP watchdog
    - Real-time interrupt
    - Clock monitor
    - Pierce oscillator
    - Phase-locked loop clock frequency multiplier
    - Limp home mode in absence of external clock
    - 25-MHz crystal oscillator reference clock
  • Operating frequency
    - 50 MHz equivalent to 25 MHz bus speed for single chip
    - 32 MHz equivalent to 16 MHz bus speed in expanded bus modes
  • Internal 2.5-V regulator
    - Supports an input voltage range from 3.3 V ± 5%
    - Low-power mode capability
    - Includes low-voltage reset (LVR) circuitry
  • 80-pin TQFP-EP or 112-pin LQFP package
    - Up to 70 I/O pins with 3.3 V input and drive capability (112-pin
      package)
    - Up to two dedicated 3.3 V input only lines (IRQ, XIRQ)
  • Development support
    - Single-wire background debug™ mode (BDM)
    - On-chip hardware breakpoints
    - Enhanced DBG debug features

Development Environment
^^^^^^^^^^^^^^^^^^^^^^^

  Either Linux or Cygwin on Windows can be used for the development
  environment. The source has been built only using the GNU toolchain
  (see below).  Other toolchains will likely cause problems.

NuttX Buildroot Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the HC12 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no HC12 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot).
  This GNU toolchain builds and executes in the Linux or Cygwin
  environments.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh demo9s12nec64/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/m9s12x-defconfig-3.3.6 .config

  6. make oldconfig

  7. make

     If the make fails because it can't find the file to download, you may
     have to locate the file on the internet and download it into the archives/
     directory manually.  For example, binutils-2.18 can be found here:
     http://ftp.gnu.org/gnu/binutils/

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  detailed PLUS some special instructions that you will need to follow if you are
  building a Cortex-M3 toolchain for Cygwin under Windows.

FreeScale HCS12 Serial Monitor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  General:
    The NuttX HCS12 port is configured to use the Freescale HCS serial
    monitor.  This monitor supports primitive debug commands that allow
    FLASH/EEPROM programming and debugging through an RS-232 serial
    interface.  The serial monior is 2Kb in size and resides in FLASH at
    addresses 0xf800-0xffff.  The monitor does not use any RAM other than
    the stack itself.

  AN2458
    The serial monitor is described in detail in Freescale Application
    Note AN2458.pdf.

  COP:
    The serial monitor uses the COP for the cold reset function and should
    not be used by the application without some precautions (see AN2458).

  Clocking:
    The serial monitor sets the operating frequency to 24 MHz.  This is
    not altered by the NuttX start-up; doing so would interfere with the
    operation of the serial monitor.

  Memory Configuration:
    Registers:
    • Register space is located at 0x0000–0x03ff.
    FLASH:
    • FLASH memory is any address greater than 0x4000. All paged
      addresses are assumed to be FLASH memory.
    • Application code should exclude the 0xf780–0xff7f memory.
    SRAM:
    • RAM ends at 0x3FFF and builds down to the limit of the device’s
      available RAM.
    • The serial monitor's stack pointer is set to the end of RAM+1
      (0x4000).
    EEPROM:
    • EEPROM (if the target device has any) is limited to the available
      space between the registers and the RAM (0x0400–to start of RAM).
    External Devices:
    • External devices attached to the multiplexed external bus
      interface are not supported

  Serial Communications:
    The serial monitor uses RS-232 serial communications through SCI0 at
    115,200 baud. The monitor must have exclusive use of this interface.
    Access to the serial port is available through a monitor jump table.

  Interrrupts:
    The serial monitor redirects interrupt vectors to an unprotected
    portion of FLASH just before the protected monitor program
    (0xf780–0xf7fe).  The monitor will automatically redirect vector
    programming operations to these user vectors.  The user code should
    therefore keep the normal (non-monitor) vector locations
    (0xff80–0xfffe).

Soft Registers
^^^^^^^^^^^^^^

  The mc68hcs12 compilation is prone to errors like the following:

    CC:  lib_b16sin.c
    lib_b16sin.c: In function `b16sin':
    lib_b16sin.c:110: error: unable to find a register to spill in class `S_REGS'
    lib_b16sin.c:110: error: this is the insn:
    (insn:HI 41 46 44 8 (parallel [
                (set (subreg:SI (reg:DI 58 [ rad ]) 4)
                    (reg/v:SI 54 [ rad ]))
                (clobber (scratch:HI))
            ]) 20 {movsi_internal} (insn_list 46 (nil))
        (expr_list:REG_UNUSED (scratch:HI)
            (expr_list:REG_NO_CONFLICT (reg/v:SI 54 [ rad ])
                (nil))))
    lib_b16sin.c:110: confused by earlier errors, bailing out

  There are several ways that this error could be fixed:

  1. Increase the number of soft registers (i.e., "fake" registers defined
     at fixed memory locations).  This can be done by adding something like
     -msoft-reg-count=4 to the CFLAGS.  This approach was not taken
     because:

     - This slows hcs12 performance
     - All of these soft registers wouil have to be saved and restored
       on every interrupt and context switch.

  2. Lowering the optimization level by dropping -Os to -O2 or, more likely,
     by removing -fomit-frame-pointer.  Also not desireable becauase 99% of the
     files that do not have this problem also increase in size.  Special case
     compilation with reduced optimization levels just for the files that need
     it could be done, but this would complicate the make system.

  3. Restructuring files to reduce the complexity.  If you add local variables
     to hold intermediate computational results, this error can be eliminated.
     This is the approach taken in NuttX.  It has disadvantages only in that
     (1) it takes some effort and good guessing to eliminate the problem, and (2)
     the problem is not really eliminated -- it can and will re-occur when files
     are changed or new files are added.

  4. Many files are built that are needed by DEM09S12NE64.  Another very simple
     option if those problem files are needed is to just remove the offending
     files from the Make.defs file so that they no longer cause a problem.

HCS12/DEMO9S12NEC64-specific Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
	   be set to:

	   CONFIG_ARCH=hc

	CONFIG_ARCH_family - For use in C code:

	   CONFIG_ARCH_HC=y

	CONFIG_ARCH_architecture - For use in C code:

	   CONFIG_ARCH_HCS12=y

	CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

	   CONFIG_ARCH_CHIP=mc92s12nec64

	CONFIG_ARCH_CHIP_name - For use in C code

	   CONFIG_ARCH_CHIP_MCS92S12NEC64

	CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
	   hence, the board that supports the particular chip or SoC.

	   CONFIG_ARCH_BOARD=demo9s12nec64

	CONFIG_ARCH_BOARD_name - For use in C code

	   CONFIG_ARCH_BOARD_DEMOS92S12NEC64 (for the Freescale DEMO9S12NE64 development board)

	CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
	   of delay loops

	CONFIG_ENDIAN_BIG - define if big endian (default is little
	   endian)

	CONFIG_DRAM_SIZE - Describes the installed RAM.

	CONFIG_DRAM_START - The start address of installed RAM

	CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
	   have LEDs

	CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
	   stack. If defined, this symbol is the size of the interrupt
	   stack in bytes.  If not defined, the user task stacks will be
	  used during interrupt handling.

	CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

	CONFIG_ARCH_CALIBRATION - Enables some build in instrumentation that
	   cause a 100 second delay during boot-up.  This 100 second delay
	   serves no purpose other than it allows you to calibratre
	   CONFIG_ARCH_LOOPSPERMSEC.  You simply use a stop watch to measure
	   the 100 second delay then adjust CONFIG_ARCH_LOOPSPERMSEC until
	   the delay actually is 100 seconds.

  GPIO Interrupts
  
    CONFIG_GPIO_IRQ - Enable general support for GPIO IRQs
    CONFIG_HCS12_PORTG_INTS - Enable PortG IRQs
    CONFIG_HCS12_PORTH_INTS - Enable PortH IRQs
    CONFIG_HCS12_PORTJ_INTS - Enable PortJ IRQs
   
  HCS12 build options:

	CONFIG_HCS12_SERIALMON - Indicates that the target systems uses
	  the Freescale serial bootloader.

	CONFIG_HCS12_NONBANKED - Indicates that the target systems does not
	  support banking.  Only short calls are made; one fixed page is
	  presented the the paging window.  Only 48Kb of FLASH is usable
	  in this configuration: pages 3e, 3d, then 3f will appear as a
	  contiguous address space in memory.

  HCS12 Sub-system support

	CONFIG_HCS12_SCI0
	CONFIG_HCS12_SCI1

  HCS12 specific device driver settings:

	CONFIG_SCIn_SERIAL_CONSOLE - selects SCIn for the console and ttys0
	  (default is the SCI0).

	CONFIG_SCIn_RXBUFSIZE - Characters are buffered as received.
	   This specific the size of the receive buffer

	CONFIG_SCIn_TXBUFSIZE - Characters are buffered before
	   being sent.  This specific the size of the transmit buffer

	CONFIG_SCIn_BAUD - The configure BAUD of the UART.

	CONFIG_SCIn_BITS - The number of bits.  Must be either 7 or 8.

	CONFIG_SCIn_PARTIY - 0=no parity, 1=odd parity, 2=even parity, 3=mark 1, 4=space 0

	CONFIG_SCIn_2STOP - Two stop bits

Configurations
^^^^^^^^^^^^^^

Each Freescale HCS12 configuration is maintained in a sudirectory and
can be selected as follow:

	cd tools
	./configure.sh demo9s12nec64/<subdir>
	cd -
	. ./setenv.sh

Where <subdir> is one of the following:

ostest:
  This configuration directory, performs a simple OS test using
  examples/ostest.

