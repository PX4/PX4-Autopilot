P112 README
===========

The P112 is notable because it was the first of the hobbyist single board
computers to reach the production stage. The P112 hobbyist computers
were relatively widespread and inspired other hobbyist centered home brew
computing projects such as N8VEM home brew computing project. The P112
project still maintains many devoted enthusiasts and has an online
repository of software and other information.

The P112 computer originated as a commercial product of "D-X Designs Pty
Ltd" of Australia. They describe the computer as "The P112 is a stand-alone
8-bit CPU board. Typically running CP/M (tm) or a similar operating system,
it provides a Z80182 (Z-80 upgrade) CPU with up to 1MB of memory, serial,
parallel and diskette IO, and realtime clock, in a 3.5-inch drive form factor.
Powered solely from 5V, it draws 150mA (nominal: not including disk drives)
with a 16MHz CPU clock. Clock speeds up to 24.576MHz are possible."

The P112 board was last available new in 1996 by Dave Brooks. In late 2004
on the Usenet Newsgroup comp.os.cpm, talk about making another run of P112
boards was discussed. David Griffith decided to produce additional P112 kits
with Dave Brooks blessing and the assistance of others. In addition Terry
Gulczynski makes additional P112 derivative hobbyist home brew computers.
Hal Bower was very active in the mid 1990's on the P112 project and ported
the "Banked/Portable BIOS".

Dave Brooks was successfully funded through Kickstarter for and another
run of P112 boards in November of 2012.

Pin Configuration
=================

The P112 is based on the 5V Z8018216FSG running at 16MHz.  The Z8018216FSG
comes in a 100-pin QFP package:

PIN  NAME
  1  /INT0                      INT0, pulled up, J1 DIN48 pin 13C
  2  /INT1/PC6                  FINTR, Floppy disk controller
  3  /INT2/PC7                  PINTR1, Floppy disk controller
  4  ST                         ST, to AEN of Floppy disk controller
  5  A0                         A0-A12 Common memory bus
  ...
 17  A12                        "               "
 18  VSS                        ---
 19  A13                        A13-A17 Common memory bus
  ...
 23  A17                        "               "
 24  A18/TOUT                   A18 Common memory bus
 25  VDD                        ---
 26  A19                        A19 Common memory bus
 27  D0                         D0-D4 Common memory bus
 ...
 30  D3                         "              "

 31  D4                         D4-D7 Common memory bus
 ...
 34  D7                         "              "
 35  /RTS0/PB0                  RTS0, 20-pin P14, pin 3
 36  /CTS0/PB1                  CTS0, pulled high (U16), 20-pin P14, pin 4
 37  /DCD0/PB2                  DCD0, pulled high (U16), 20-pin P14, pin 10
 38  TXA0/PB3                   TXA0, 20-pin P14, pin 8
 39  RXA0/PB4                   RXA0, pulled high (U17), 20-pin P14, pin 2
 40  TXA1/PB5                   TXA1, 20-pin P14, pin 1
 41  RXA1/PB6                   RXA1, pulled high (U17), 20-pin P14, pin 9
 42  RXS//CTS1/PB7              CTS1, pulled high (U17), 20-pin P14, pin 7
 43  CKA0//DREQ0                /DREQ0, DMA Request Select, 5-pin P2, pin 2
 44  VSS                        ---
 45  CKA1//TEND0                /TEND0, J1 DIN48 pin 14A
 46  TXS//DTR//REQB//HINTR      DTRB, 20-pin P14, pin 6
 47  CKS//W//REQB//HTXRDY       SIORQ, DMA Request Select, 5-pin P2, pin 5 (may be DREQ 0 or DREQ1)
 48  /DREQ1                     /DREQ1, DMA Request Select, 5-pin P2, pin 4
 49  VDD                        ---
 50  /TEND1//RTSB//HRXRDY       NB /TEND1 = RTSB, 20-pin P14, pin 5; J1 DIN48 pin 14B

 51  /RAMCS                     /RAMCS, Chip select logic (U11B); also J1 DIN48 pin 9B
 52  /ROMCS                     /ROMCS, Chip select logic (P2); also J1 DIN48 pin 12B
 53  EV1                        Grounded
 54  EV2                        Grounded
 55  PA0/HD0                    IO, U6 DS1202 Serial Timekeeping chip
 56  PA1/HD1                    CLK, U6 DS1202 Serial Timekeeping chip
 57  PA2/HD2                    /RST, U6 DS1202 Serial Timekeeping chip
 58  PA3/HD3                    N/C
 59  PA4/HD4                    N/C
 60  PA5/HD5                    U12 NMF0512S, Isolated 1W regulated single output DC/DC converter
 61  PA6/HD6                    DSR, U7 LT1133, Advanced Low Power 5V RS232 Driver/Receiver
 62  PA7/HD7                    RTS, U7 LT1133, Advanced Low Power 5V RS232 Driver/Receiver
 63  /W//REQA/PC5               WREQA, N/C
 64  /DTR//REQA/PC3             DTRA, U7 LT1133, Advanced Low Power 5V RS232 Driver/Receiver
 65  /MWR/PC2//RTSA             /MWR, Common memory bus signal
 66  /CTSA/PC1                  CTSA, U7 LT1133, Advanced Low Power 5V RS232 Driver/Receiver
 67  /DCDA/PC0                  DCDA, U7 LT1133, Advanced Low Power 5V RS232 Driver/Receiver
 68  /SYNCA/PC4                 SYNCA, U7 LT1133, Advanced Low Power 5V RS232 Driver/Receiver
 69  /RTXCA                     ?
 70  VSS                        ---
 71  /IOCS/IEO                  /IOCS, Logic circuit with M1, generates LIVE which conditions inputs
                                to the floppy disk controller
 72  IEI                        IEI, J1 DIN48 pin 14C
 73  VDD                        ---
 74  RXDA                       RXDA, U7 LT1133, Advanced Low Power 5V RS232 Driver/Receiver
 75  /TRXCA                     ?
 76  TXDA                       TXDA, U7 LT1133, Advanced Low Power 5V RS232 Driver/Receiver
 77  /DCDB//HRD                 DCDB, pulled high (U16), 20-pin P14, pin 12
 78  /CTSB//HWR                 DCDB, pulled high (U17), 20-pin P14, pin 11
 79  TXDB//HDDIS                TXDB, 20-pin P14, pin 14
 80  /TRXCB/HA0                 TRXCB, pulled high (U17), 20-pin P14, pin 15

 81  RXDB/HA1                   RXDB, pulled high (U16), 20-pin P14, pin 16
 82  /RTXCB/HA2                 RTXCB, pulled high (U17), 20-pin P14, pin 17
 83  /SYNCB//HCS                SYNCB, pulled high (U16), 20-pin P14, pin 18
 84  /HALT                      ?
 85  /RFSH                      ?
 86  /IORQ                      /IORQ, J1 DIN48 pin 12A
 87  /MRD//MREQ                 /MRD, Common memory bus signal
 88  E                          E, Conditions inputs to floppy disk controller; also J1 DIN48 pin 13B
 89  /M1                        /M1, Logic circuit with /IOCS, generates LIVE which conditions inputs
                                to the floppy disk controller; also J1 DIN48 pin 11A
 90  /WR                        /WR, Common memory bus; Conditions inputs to floppy disk controller;
                                also J1 DIN48 pin 12C
 91  /RD                        /RD, J1 DIN48 pin 11C
 92  PHI                        PHI, J1 DIN48 pin 15B
 93  VSS                        ---
 94  XTAL                       16 MHz XTAL
 95  EXTAL                      16 MHz XTAL
 96  /WAIT                      /WAIT, J1 DIN48 pin 11B
 97  /BUSACK                    ?
 98  /BUSREQ                    /BUSREQ, Pulled high
 99  /RESET                     /RST (to lots of places)
100  /NMI                       /NMI, Pulled high

P112 Serial Console
===================

The UARTs are not used on the P112 board (the UART signals are avaiable off-board through P14).
The serial console is provided by U7 LT1133, Advanced Low Power 5V RS232 Driver/Receiver
that connects to the P112 via the Z85230 ESCC channel A.
