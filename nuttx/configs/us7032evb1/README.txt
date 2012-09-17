Status
^^^^^^

*** UNSTABLE ***
The port is basically complete and many examples run correctly.  However, there
are remaining instabilities that make the port un-usable.  The nature of these
is not understood; the behavior is that certain SH-1 instructions stop working
as advertised.  This could be a silicon problem, some pipeline issue that is not
handled properly by the gcc 3.4.5 toolchain (which has very limit SH-1 support
to begin with), or perhaps with the CMON debugger.  At any rate, I have exhausted
all of the energy that I am willing to put into this cool old processor for the
time being.

Toolchain
^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the SH toolchain (if
  different from the default).

  If you have no SH toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot/).

  1. You must have already configured Nuttx in <some-dir>nuttx.

     cd tools
     ./configure.sh us7032evb1/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack

  4. cd <some-dir>/buildroot

  5. cp configs/sh-defconfig .config

  6. make oldconfig

  7. make

  8. Edit setenv.h so that the PATH variable includes the path to the
     newly built binaries.

shterm
^^^^^^

  The USB7032EVB1 supports CMON in PROM.  CMON requires special
  serial interactions in order to upload and download program files.
  Therefore, a standard terminal emulation program (such as minicom)
  cannot be used.

  The shterm subdirectory contains a small terminal emulation
  program that supports these special interactions for file transfers.

Configurations
^^^^^^^^^^^^^^

Each SH-1 configuration is maintained in a sudirectory and
can be selected as follow:

	cd tools
	./configure.sh us7032evb1/<subdir>
	cd -
	. ./setenv.sh

Where <subdir> is one of the following:

ostest
^^^^^^as advertised.  This could be a silicon problem, some pipeline issue that is not handled properly by the gcc 3.4.5 tool

This configuration directory, performs a simple OS test using
examples/ostest.

nsh
^^^

Configures the NuttShell (nsh) located at examples/nsh.  The
Configuration enables only the serial NSH interfaces.

NOTE:  At present, the NSH example does not run.  See the "Status"
discussion above for a full explanation.

Configuration Options
^^^^^^^^^^^^^^^^^^^^^

In additional to the common configuration options listed in the
file configs/README.txt, there are other configuration options
specific to the SH-1

Architecture selection

  CONFIG_ARCH - identifies the arch subdirectory and, hence, the
    processor architecture.  This should be sh (for arch/sh)
  CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory.
    This should be sh1 (for arch/sh/src/sh1 and arch/sh/include/sh1)
  CONFIG_ARCH_SH1 and CONFIG_ARCH_CHIP_SH7032 - for use in C code.  These
    identify the particular chip or SoC that the architecture is
    implemented in.
  CONFIG_ARCH_BOARD - identifies the configs subdirectory and, hence,
    the board that supports the particular chip or SoC.  This
    should be us7032evb1 for (configs/us7032evb1).
  CONFIG_ARCH_BOARD_US7032EVB1 - for use in C code
  CONFIG_ENDIAN_BIG - the SH-1 usually runs big-endian
  CONFIG_ARCH_NOINTC - define if the architecture does not
    support an interrupt controller or otherwise cannot support
    APIs like up_enable_irq() and up_disable_irq().  Should be
    defined.
  CONFIG_ARCH_IRQPRIO
    Define if the architecture suports prioritizaton of interrupts
    and the up_prioritize_irq() API.  Should be defined.
  CONFIG_BOARD_LOOPSPERMSEC - for delay loops
  CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to SH1_LCEVB1
  CONFIG_DRAM_SIZE - Describes the internal DRAM.
  CONFIG_DRAM_START - The start address of internal DRAM
  CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
    stack. If defined, this symbol is the size of the interrupt
    stack in bytes.  If not defined, the user task stacks will be
    used during interrupt handling.
  CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

  CONFIG_SH1_DMAC0, CONFIG_SH1_DMAC1, CONFIG_SH1_DMAC2, CONFIG_SH1_DMAC3,
  CONFIG_SH1_ITU1, CONFIG_SH1_ITU2, CONFIG_SH1_ITU3, CONFIG_SH1_ITU4,
  CONFIG_SH1_SCI0, CONFIG_SH1_SCI1, CONFIG_SH1_PCU, CONFIG_SH1_AD,
  CONFIG_SH1_WDT, CONFIG_SH1_CMI - Each unused chip block should b
    disabled to save space

SH1 specific device driver settings

  CONFIG_SCIn_SERIAL_CONSOLE - selects the SCIn for the
    console and ttys0 (default is the UART0).
  CONFIG_SCIn_RXBUFSIZE - Characters are buffered as received.
    This specific the size of the receive buffer
  CONFIG_SCIn_TXBUFSIZE - Characters are buffered before
    being sent.  This specific the size of the transmit buffer
  CONFIG_SCIn_BAUD - The configure BAUD of the UART.  Must be
  CONFIG_SCIn_BITS - The number of bits.  Must be either 7 or 8.
  CONFIG_SCIn_PARTIY - 0=no parity, 1=odd parity, 2=even parity, 3=mark 1, 4=space 0
  CONFIG_SCIn_2STOP - Two stop bits
