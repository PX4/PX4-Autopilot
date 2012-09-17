README
^^^^^^

Toolchain
^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the ARM920T GCC toolchain (if
  different from the default).

  If you have no ARM toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot/).

  1. You must have already configured Nuttx in <some-dir>nuttx.

     cd tools
     ./configure.sh mx1ads/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack

  4. cd <some-dir>/buildroot

  5. cp configs/arm920t-defconfig-4.2.4 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h so that the PATH variable includes the path to the
     newly built binaries.

ARM/i.MX1-specific Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
	   be set to:

	   CONFIG_ARCH=arm

	CONFIG_ARCH_family - For use in C code:

	   CONFIG_ARCH_ARM=y

	CONFIG_ARCH_architecture - For use in C code:

	   CONFIG_ARCH_ARM920T=y

	CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

	   CONFIG_ARCH_CHIP=imx

	CONFIG_ARCH_CHIP_name - For use in C code.  Could be line _IMX1,
	   _IMXL, _IMX21, _IMX27, _IMX31, etc. (not all of which are
	   supported).

	   CONFIG_ARCH_CHIP_IMX1

	CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
	   hence, the board that supports the particular chip or SoC.

	   CONFIG_ARCH_BOARD=mx1ads (for the Freescale MX1ADS evaluation board)

	CONFIG_ARCH_BOARD_name - For use in C code

	   CONFIG_ARCH_BOARD_MX1ADS (for the Spectrum Digital C5471 EVM)

	CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
	   of delay loops

	CONFIG_ENDIAN_BIG - define if big endian (default is little
	   endian)

	CONFIG_DRAM_SIZE - Describes the installed DRAM.

	CONFIG_DRAM_START - The start address of installed DRAM

	CONFIG_DRAM_VSTART - The startaddress of DRAM (virtual)

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

  IMX specific device driver settings

	CONFIG_UARTn_SERIAL_CONSOLE - selects the UARTn for the
	   console and ttys0 (default is the UART0).
	CONFIG_UARTn_RXBUFSIZE - Characters are buffered as received.
	   This specific the size of the receive buffer
	CONFIG_UARTn_TXBUFSIZE - Characters are buffered before
	   being sent.  This specific the size of the transmit buffer
	CONFIG_UARTn_BAUD - The configure BAUD of the UART.  Must be
	CONFIG_UARTn_BITS - The number of bits.  Must be either 7 or 8.
	CONFIG_UARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
	CONFIG_UARTn_2STOP - Two stop bits

  IMX USB Configuration

	CONFIG_IMX_GIO_USBATTACH
	   GIO that detects USB attach/detach events
	CONFIG_IMX_GIO_USBDPPULLUP
	   GIO 
	CONFIG_DMA320_USBDEV_DMA
	   Enable IMX-specific DMA support
	CONFIG_IMX_GIO_USBATTACH=6

Configurations
^^^^^^^^^^^^^^

Each MX1ADS configuration is maintained in a sudirectory and
can be selected as follow:

	cd tools
	./configure.sh imxads/<subdir>
	cd -
	. ./setenv.sh

Where <subdir> is one of the following:

ostest
^^^^^^

This configuration directory, performs a simple OS test using
examples/ostest.
