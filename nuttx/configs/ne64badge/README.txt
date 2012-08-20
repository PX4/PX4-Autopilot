README
^^^^^^

  This README discusses issues unique to NuttX configurations for the
  Future Electronics Group NE64 /PoE Badge board based on the
  MC9S12NE64 hcs12 cpu.

CONTENTS
^^^^^^^^
  • MC9S12NE64 Features
  • NE64 Badge Pin Usage
  • Development Environment
  • NuttX Buildroot Toolchain
  • FreeScale HCS12 Serial Monitor
  • Soft Registers
  • HCS12/NE64BADGE-specific Configuration Options
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

NE64 Badge Pin Usage
^^^^^^^^^^^^^^^^^^^^

PIN PIN NAME            BOARD SIGNAL   NOTES
--- ------------------- -------------- ----------------------
 44 RESET               J3 RESET_L     Also to SW3
 57 BKGD/MODC/TAGHI_B   BDM BKGD CON6A

 85 PAD0                VR1            Potentiometer
 86 PAD1                J3 ANALOG_IN0  Not used on board
 87 PAD2                J3 ANALOG_IN1  " " "  " "" "   "
 88 PAD3                J3 ANALOG_IN2  " " "  " "" "   "
 89 PAD4                J3 ANALOG_IN3  " " "  " "" "   "

 70 PHY_TXP             J7 TD+         RJ45 connector
 71 PHY_TXN             J7 TD-         RJ45 connector
 73 PHY_RXP             J7 RD+         RJ45 connector
 74 PHY_RXN             J7 RD-         RJ45 connector

 Ports A,B,E,K managed by the MEBI block
 ---------------------------------------
 60 PA0/ADDR8/DATA8     J3 ADDR_DATA8  Not used on board
 61 PA1/ADDR9/DATA9     J3 ADDR_DATA9  " " "  " "" "   "
 62 PA2/ADDR10/DATA10   J3 ADDR_DATA10 " " "  " "" "   "
 63 PA3/ADDR11/DATA11   J3 ADDR_DATA11 " " "  " "" "   "
 77 PA4/ADDR12/DATA12   J3 ADDR_DATA12 " " "  " "" "   "
 78 PA5/ADDR13/DATA13   J3 ADDR_DATA13 " " "  " "" "   "
 79 PA6/ADDR14/DATA14   J3 ADDR_DATA14 " " "  " "" "   "
 80 PA7/ADDR15/DATA15   J3 ADDR_DATA15 " " "  " "" "   "

 10 PB0/ADDR0/DATA0     J3 ADDR_DATA0  Not used on board
 11 PB1/ADDR1/DATA1     J3 ADDR_DATA1  " " "  " "" "   "
 12 PB2/ADDR2/DATA2     J3 ADDR_DATA2  " " "  " "" "   "
 13 PB3/ADDR3/DATA3     J3 ADDR_DATA3  " " "  " "" "   "
 16 PB4/ADDR4/DATA4     J3 ADDR_DATA4  " " "  " "" "   "
 17 PB5/ADDR5/DATA5     J3 ADDR_DATA5  " " "  " "" "   "
 18 PB6/ADDR6/DATA6     J3 ADDR_DATA6  " " "  " "" "   "
 19 PB7/ADDR7/DATA7     J3 ADDR_DATA7  " " "  " "" "   "

 56 PE0/XIRQ_B          BUTTON1        SW1
 55 PE1/IRQ_B           J3 IRQ         Not used on board
 54 PE2/R_W             J3 RW          " " "  " "" "   "
 53 PE3/LSTRB_B/TAGLO_B J3 LSTRB       " " "  " "" "   "
 41 PE4/ECLK            J3 ECLK        " " "  " "" "   "
 40 PE5/IPIPE0/MODA     J3 MODA        " " "  " "" "   "
 39 PE6/IPIPE1/MODB     J3 MODB        " " "  " "" "   "
 38 PE7/NOACC/XCLKS_B   pulled low     pulled low

 97 PK0/XADR14          N/C            N/C
 98 PK1/XADR15          N/C            N/C
 99 PK2/XADR16          N/C            N/C
100 PK3/XADR17          N/C            N/C
103 PK4/XADR18          N/C            N/C
104 PK5/XADR19          N/C            N/C
105 PK6/XCS_B           J3 XCS         Not used on board
106 PK7/ECS_B/ROMCTL    J3 ECS         " " "  " "" "   "

 Ports T,S,G,H,J,L managed by the PIM Block
 ------------------------------------------
110 PT4/IOC1_4          J3 GPIO8       Not used on board
109 PT5/IOC1_5          J3 GPIO9       " " "  " "" "   "
108 PT6/IOC1_6          J3 GPIO10      " " "  " "" "   "
107 PT7/IOC1_7          N/C            N/C

 30 PS0/RXD0            RS232_RX       Eventually maps to J2 RXD
 31 PS1/TXD0            RS232_TX       Eventually maps to J2 TXD
 32 PS2/RXD1            J3&J4 UART_RX  Not used on board
 33 PS3/TXD1            J3&J4 UART_TX  " " "  " "" "   "
 34 PS4/MISO            J3 SPI_MISO    " " "  " "" "   "
 35 PS5/MOSI            J3 SPI_MOSI    " " "  " "" "   "
 36 PS6/SCK             J3 SPI_CLOCK   " " "  " "" "   "
 37 PS7/SS_B            J3 SPI_SS      " " "  " "" "   "
 
 22 PG0/RXD0/KWG0       J3 GPIO0       Not used on board
 23 PG1/RXD1/KWG1       J3 GPIO1       " " "  " "" "   "
 24 PG2/RXD2/KWG2       J3 GPIO2       " " "  " "" "   "
 25 PG3/RXD3/KWG3       J3 GPIO3       " " "  " "" "   "
 26 PG4/RXCLK/KWG4      J3 GPIO4       " " "  " "" "   "
 27 PG5/RXDV/KWG5       J3 GPIO5       " " "  " "" "   "
 28 PG6/RXER/KWG6       J3 GPIO6       " " "  " "" "   "
 29 PG7/KWG7            J3 GPIO7       " " "  " "" "   "
 
  7 PH0/TXD0/KWH0       N/C            N/C
  6 PH1/TXD1/KWH1       N/C            N/C
  5 PH2/TXD2/KWH2       J4 XBEE_RESET  Not used on board
  4 PH3/TXD3/KWH3       J4 XBEE_RSSI   Not used on board
  3 PH4/TXCLK/KWH4      BUTTON2        SW2
  2 PH5/TXDV/KWH5       J5 XBEE_LOAD_H Not used on board
  1 PH6/TXER/KWH6       J4 XBEE_LOAD_L Not used on board
 
  8 PJ0/MDC/KWJ0        LED1           D21, red
  9 PJ1/MDIO/KWJ1       LED2           D22, red
 20 PJ2/CRS/KWJ2        J3 SPI_CS      Not used on board
 21 PJ3/COL/KWJ3        N/C
112 PJ6/SDA/KWJ6        J3 I2C_DATA    Not used on board
111 PJ7/SCL/KWJ7        J3 I2C_CLOCK   " " "  " "" "   "

 51 PL6/TXER/KWL6       N/C            N/C
 52 PL5/TXDV/KWL5       N/C            N/C
 58 PL4/COLLED          Collision LED  red
 59 PL3/DUPLED          Full Duplex LED yellow
 81 PL2/SPDLED          100Mbps Speed LED yellow
 83 PL1/LNKLED          Link Good LED  green
 84 PL0/ACTLED          Activity LED   yellow

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
  SourceForge download site (https://sourceforge.net/project/showfiles.php?group_id=189573).
  This GNU toolchain builds and executes in the Linux or Cygwin
  environments.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh ne64badge/<sub-dir>

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

HCS12/NE64BADGE-specific Configuration Options
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

	   CONFIG_ARCH_BOARD=ne64badge

	CONFIG_ARCH_BOARD_name - For use in C code

	   CONFIG_ARCH_BOARD_NE64BADGE (for the Future Electronics Group NE64 Badge)

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
	./configure.sh ne64badge/<subdir>
	cd -
	. ./setenv.sh

Where <subdir> is one of the following:

ostest:
  This configuration directory, performs a simple OS test using
  examples/ostest.

