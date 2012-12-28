zp214xpa README FILE
====================

The ZP213X/4XPA board from the0.net with LPC2148 installed.
Includes support for the UG-2864AMBAG01 OLED from The0.net.

Contents
========

  o MCU Connections
  o Serial Console
  o Configurations

MCU Connections:
================

Module Socket:
--------------
PIN NAME  PIN NAME
 1  VBAT  56  VCC
 2  3V3   55  Vusb
 3  VREF  54  3V3
 4  P0.0  53  RESET
 5  P0.1  52  P1.31
 6  P0.2  51  P1.30
 7  P0.3  50  P1.29
 8  P0.4  49  P1.28
 9  P0.5  48  P1.27
10  P0.6  47  P1.26
11  P0.7  46  P1.25
12  P0.8  45  P1.24
13  P0.9  44  P1.23
14  P0.10 43  P1.22
15  P0.11 42  P1.21
16  P0.12 41  P1.20
17  P0.13 40  P1.19
18  P0.14 39  P1.18
19  P0.15 38  P1.17
20  P0.16 37  P1.16
21  P0.17 36  P0.31
22  P0.18 35  P0.30
23  P0.19 34  P0.29
24  P0.20 33  P0.28
25  P0.21 32  P0.27
26  P0.22 31  P0.26
27  P0.23 30  P0.25
28  GND   29  GND

JTAG Debug:
-----------
PIN NAME       PIN NAME
 1  VCC1        2  3V3
 3  P1.31 NTRST 4  GND
 5  P1.28 TDI   6  GND
 7  P1.30 TMS   8  GND
 9  P1.29 TCK  10  GND
11  P1.26 RTCK 12  GND
13  P1.27 TDO  14  GND
15  RESET NRTS 16  GND
17  N/C   NC0  18  GND
19  N/C   NC1  20  GND

Z28160 Net Module:
------------------
PIN NAME      PIN NAME
 1  P0.7  /CS 10  3V3   VCC
 2  P0.4  SCK  9  P1.24 RST
 3  P0.6  SI   8  N/C   CLKOUT
 4  P0.5  SO   7  INT   P1.25
 5  GND        6  N/C   WOL

SPI LCD:
--------
PIN NAME
 1  3V3   3V3
 2  VCC   5V
 3  P0.18 RESET(DO)
 4  P0.19 DI
 5  P0.20 CS
 6  P0.17 SCK
 7  P0.23 A0(RESET)
 8  N/C   LED-
 9  N/C   LED+(BL)
10  GND   GND

USB Interface:
--------------
Vusb, P0.26, P0.27

Serial Console:
===============

Both UART0 and UART1 are always enabled.  UART0 is configured to be the
serial console in these configurations.

P0.0/TXD0/PWM1        Module Socket, Pin 4
P0.1/RxD0/PWM3/EINT0  Module Socket, Pin 5

P0.8/TXD1/PWM4/AD1.1  Module Socket, Pin 12
P0.9/RxD1/PWM6/EINT3  Module Socket, Pin 13

Configurations:
===============

Each NXP LPC214x configuration is maintained in a sudirectory and
can be selected as follow:

    cd tools
    ./configure.sh zp214xpa/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

nsh:
----

  Configures the NuttShell (nsh) located at examples/nsh.  The
  Configuration enables only the serial NSH interfaces.

    NOTES:
 
    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the mconf tool.  See nuttx/README.txt and
          misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Default platform/toolchain:

       CONFIG_HOST_LINUX=y             : Windows
       CONFIG_ARM_TOOLCHAIN_GNU_EABI=y : Buildroot (arm-nuttx-elf-gcc)
       CONFIG_RAW_BINARY=y             : Output formats: ELF and raw binary
