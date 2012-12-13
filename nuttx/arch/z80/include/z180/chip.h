/****************************************************************************
 * arch/z80/include/z180/chip.h
 * arch/chip/io.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_Z80_INCLUDE_Z180_CHIP_H
#define __ARCH_Z80_INCLUDE_Z180_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Bits in the Z180 FLAGS register ******************************************/

#define Z180_C_FLAG      0x01       /* Bit 0: Carry flag */
#define Z180_N_FLAG      0x02       /* Bit 1: Add/Subtract flag  */
#define Z180_PV_FLAG     0x04       /* Bit 2: Parity/Overflow flag */
#define Z180_H_FLAG      0x10       /* Bit 4: Half carry flag */
#define Z180_Z_FLAG      0x40       /* Bit 5: Zero flag */
#define Z180_S_FLAG      0x80       /* Bit 7: Sign flag */

/* Z180 Chip Definitions ****************************************************/
/* Z800180
 *
 * The 8-bit Z80180 MPU provides the benefits of reduced system costs and
 * also provides full backward compatibility with existing ZiLOG Z80 devices.
 * Reduced system costs are obtained by incorporating several key system
 * functions on-chip with the CPU. These key functions include I/O devices
 * such as DMA, UART, and timer channels. Also included on-chip are wait-
 * state generators, a clock oscillator, and an interrupt controller. The
 * Z80180 MPU is housed in 80-pin QFP, 68-pin PLCC, and 64-pin DIP packages.
 *
 * Z80180 Features
 *
 *   Enhanced Z80 CPU
 *   1 MB MMU
 *   2 DMA channels*
 *   2 UARTs* (up to 512 Kbps)
 *   Two 16-Bit Timers
 *   Clock Serial I/O
 *   On-Chip Oscillator
 *   Power-Down Mode*
 *   Divide-by-One/Divide-by-Two/Multiply-by-Two Clock Options*
 *   Programmable Driver Strength for EMI Tuning
 *
 *   * Enhanced on the Z8S180 and Z8L180 MPUs
 */

#if   defined(CONFIG_ARCH_CHIP_Z8018006VSG) || /* 68-pin PLCC */ \
      defined(CONFIG_ARCH_CHIP_Z8018010VSG) || /* 68-pin PLCC */ \
      defined(CONFIG_ARCH_CHIP_Z8018008VSG) || /* 68-pin PLCC */ \
      defined(CONFIG_ARCH_CHIP_Z8018010FSG) || /* 80-pin QFP (11 pins N/C) */ \
      defined(CONFIG_ARCH_CHIP_Z8018008VEG) || /* 68-pin PLCC */ \
      defined(CONFIG_ARCH_CHIP_Z8018006VEG)    /* 68-pin PLCC */

#  undef  HAVE_Z8S180                             /* Not Z8S180 (5V) or Z8L180 (3.3V) core */
#  define HAVE_Z8X180    1                        /* Z8x180 registers */
#  undef  HAVE_Z8X181                             /* Z8x181 registers */
#  undef  HAVE_Z8X182                             /* Z8x182 registers */
#  define HAVE ROM       0                        /* No on-chip ROM */
#  define HAVE_SERIALIO  1                        /* Have clocked serial I/O */
#  undef  HAVE_WDT                                /* No Watchdog timer */
#  define HAVE_NTIMERS16 2                        /* Two (2) 16-bit timers */
#  define HAVE_NCTCS     0                        /* No Counter/Timers (CTCs) */
#  define HAVE_NDMA      2                        /* Two (2) DMA channels */
#  define HAVE_NUARTS    2                        /* Two (2) UARTs (up to 512 Kbps) */
#  define HAVE_NSCC      0                        /* No serial communication controller */
#  define HAVE_NESCC     0                        /* No Enhanced Serial Communication Controllers */
#  undef  HAVE_16550                              /* No 16550 MIMIC */
#  define HAVE_NIOLINES  0                        /* No I/O lines */
#  define HAVE_NPAR8     0                        /* No 8-bit parallel ports */
#  undef  HAVE_IEEE1284                           /* No bidirectional centronics interface (IEEE 1284) */

#elif defined(CONFIG_ARCH_CHIP_Z8018006PSG) || /* 64-pin DIP 6 MHz 5V */ \
      defined(CONFIG_ARCH_CHIP_Z8018008FSG) || /* 80-pin QFP (11 pins N/C) 8MHz 5V */ \
      defined(CONFIG_ARCH_CHIP_Z8018010PSG) || /* 64-pin DIP 10MHz 5V */ \
      defined(CONFIG_ARCH_CHIP_Z8018006PEG) || /* 64-pin DIP 6MHz 5V */ \
      defined(CONFIG_ARCH_CHIP_Z8018010VEG) || /* 68-pin PLCC 10MHz 5V */ \
      defined(CONFIG_ARCH_CHIP_Z8018010PEG) || /* 64-pin DIP 10MHz 5V */ \
      defined(CONFIG_ARCH_CHIP_Z8018008PSG) || /* 64-pin DIP 8MHz 5V */ \
      defined(CONFIG_ARCH_CHIP_Z8018006FSG)    /* 80-pin QFP (11 pins N/C) 6MHz 5V */

#  undef  HAVE_Z8S180                             /* Not Z8S180 (5V) or Z8L180 (3.3V) core */
#  define HAVE_Z8X180    1                        /* Z8x180 registers */
#  undef  HAVE_Z8X181                             /* Z8x181 registers */
#  undef  HAVE_Z8X182                             /* Z8x182 registers */
#  define HAVE ROM       0                        /* No on-chip ROM */
#  undef  HAVE_SERIALIO                           /* No clocked serial I/O ? */
#  undef  HAVE_WDT                                /* No Watchdog timer */
#  define HAVE_NTIMERS16 2                        /* Two (2) 16-bit timers */
#  define HAVE_NCTCS     0                        /* No Counter/Timers (CTCs) */
#  define HAVE_NDMA      2                        /* Two (2) DMA channels */
#  define HAVE_NUARTS    2                        /* Two (2) UARTs (up to 512 Kbps) */
#  define HAVE_NSCC      0                        /* No serial communication controller */
#  define HAVE_NESCC     0                        /* No Enhanced Serial Communication Controllers */
#  undef  HAVE_16550                              /* No 16550 MIMIC */
#  define HAVE_NIOLINES  0                        /* No I/O lines */
#  define HAVE_NPAR8     0                        /* No 8-bit parallel ports */
#  undef  HAVE_IEEE1284                           /* No bidirectional centronics interface (IEEE 1284) */

#elif defined(CONFIG_ARCH_CHIP_Z8018000XSO)
      defined(CONFIG_ARCH_CHIP_Z8018010FEG)
      defined(CONFIG_ARCH_CHIP_Z8018000WSO)
      defined(CONFIG_ARCH_CHIP_Z8018008PEG)

#  undef  HAVE_Z8S180                             /* Not Z8S180 (5V) or Z8L180 (3.3V) core */
#  define HAVE_Z8X180    1                        /* Z8x180 registers */
#  undef  HAVE_Z8X181                             /* Z8x181 registers */
#  undef  HAVE_Z8X182                             /* Z8x182 registers */
#  define HAVE ROM       0                        /* No on-chip ROM */
#  define HAVE_SERIALIO  1                        /* Have clocked serial I/O */
#  undef  HAVE_WDT                                /* No Watchdog timer */
#  define HAVE_NTIMERS16 2                        /* Two (2) 16-bit timers */
#  define HAVE_NCTCS     0                        /* No Counter/Timers (CTCs) */
#  define HAVE_NDMA      2                        /* Two (2) DMA channels */
#  define HAVE_NUARTS    2                        /* Two (2) UARTs (up to 512 Kbps) */
#  define HAVE_NSCC      0                        /* No serial communication controller */
#  define HAVE_NESCC     0                        /* No Enhanced Serial Communication Controllers */
#  undef  HAVE_16550                              /* No 16550 MIMIC */
#  define HAVE_NIOLINES  0                        /* No I/O lines */
#  define HAVE_NPAR8     0                        /* No 8-bit parallel ports */
#  undef  HAVE_IEEE1284                           /* No bidirectional centronics interface (IEEE 1284) */

/* Z80181
 *
 * The Z80181 SAC Smart Access Controller is an 8-bit CMOS microprocessor that
 * combines a Z180-compatible MPU, one channel of the Z85C30 Serial Communications
 * Controller, a Z80 CTC, two 8-bit general-purpose parallel ports, and two Chip
 * Select signals, into a single 100-pin Quad Flat Pack package.
 *
 * Z80181 Features
 *
 *   Enhanced Z80 CPU
 *   1 MB MMU
 *   2 DMAs
 *   2 UARTs
 *   Two 16-Bit Timers
 *   Clock Serial I/O
 *   1 Channel SCC
 *   1 8-Bit Counter/Timer
 *   16 I/O Lines
 *   Emulation Mode
 */

#elif defined(CONFIG_ARCH_CHIP_Z8018110FEG)       /* 100-pin QFP */

#  undef  HAVE_Z8S180                             /* Not Z8S180 (5V) or Z8L180 (3.3V) core */
#  undef  HAVE_Z8X180                             /* Z8x180 registers */
#  define HAVE_Z8X181    1                        /* Z8x181 registers */
#  undef  HAVE_Z8X182                             /* Z8x182 registers */
#  define HAVE ROM       0                        /* No on-chip ROM */
#  define HAVE_SERIALIO  1                        /* Have clocked serial I/O */
#  undef  HAVE_WDT                                /* No Watchdog timer */
#  define HAVE_NTIMERS16 2                        /* Two (2) 16-bit timers */
#  define HAVE_NCTCS     1                        /* One (1) 8-bit counter/timer */
#  define HAVE_NDMA      2                        /* Two (2) DMA channels */
#  define HAVE_NUARTS    2                        /* Two (2) UARTs (up to 512 Kbps) */
#  define HAVE_NSCC      1                        /* One (1) Z85C30 serial communication controller */
#  define HAVE_NESCC     0                        /* No Enhanced Serial Communication Controllers */
#  undef  HAVE_16550                              /* No 16550 MIMIC */
#  define HAVE_NIOLINES  16                       /* Sixteen (16) I/O lines */
#  define HAVE_NPAR8     0                        /* No 8-bit parallel ports */
#  undef  HAVE_IEEE1284                           /* No bidirectional centronics interface (IEEE 1284) */

/* Z80182
 *
 * The Z80182 and Z8L182 MPUs are smart peripheral controller ICs for modems, fax,
 * voice messaging, and other communications applications. It uses the Z80180
 * microprocessor linked with two channels of the industry-standard Z85230 ESCC,
 * 24 bits of parallel I/O, and a 16550 MIMIC for direct connection to the IBM PC,
 * XT, or AT bus
 *
 * Z80182 Features
 *
 *   Enhanced Z80 CPU
 *   1 MB MMU
 *   2 DMAs
 *   2 UARTs (up to 512 Kbps)
 *   Two 16-Bit Timers
 *   Clock Serial I/O
 *   Power-Down Mode
 *   Divide-by-One/Divide-by-Two/Multiply-by-Two Clock Options
 *   Enhanced Serial Communication Controller (ESCC) (2 Channels) with 32-Bit CRC
 *   16550 MIMIC
 *   24 Parallel I/O
 *   3.3 V and 5 V Version
 */

#elif defined(CONFIG_ARCH_CHIP_Z8018233FSG) ||    /* 100-pin QFP */ \
      defined(CONFIG_ARCH_CHIP_Z8018220AEG) ||    /* 100-pin LQFP 20MHz 5V */ \
      defined(CONFIG_ARCH_CHIP_Z8018216FSG) ||    /* 100-pin QFP 16MHz 5V */ \
      defined(CONFIG_ARCH_CHIP_Z8018216ASG) ||    /* 100-pin LQFP */ \
      defined(CONFIG_ARCH_CHIP_Z8018233ASG)       /* 100-pin LQFP 33MHz 5V */

#  undef  HAVE_Z8S180                             /* Not Z8S180 (5V) or Z8L180 (3.3V) core */
#  undef  HAVE_Z8X180                             /* Z8x180 registers */
#  undef  HAVE_Z8X181                             /* Z8x181 registers */
#  define HAVE_Z8X182    1                        /* Z8x182 registers */
#  define HAVE ROM       0                        /* No on-chip ROM */
#  define HAVE_SERIALIO  1                        /* Have clocked serial I/O */
#  undef  HAVE_WDT                                /* No Watchdog timer */
#  define HAVE_NTIMERS16 2                        /* Two (2) 16-bit timers ? */
#  define HAVE_NCTCS     0                        /* No Counter/Timers (CTCs) */
#  define HAVE_NDMA      2                        /* Two (2) DMA channels */
#  define HAVE_NUARTS    2                        /* Two (2) UARTs (up to 512 Kbps) */
#  define HAVE_NSCC      0                        /* No Z85C30 serial communication controller */
#  define HAVE_NESCC     2                        /* Two (2) Z85230 Enhanced Serial Communication Controllers */
#  define HAVE_16550     1                        /* Have 16550 MIMIC */
#  define HAVE_NIOLINES  0                        /* No I/O lines */
#  define HAVE_NPAR8     3                        /* Three (3) 8-bit parallel ports (24-bit) */
#  undef  HAVE_IEEE1284                           /* No bidirectional centronics interface (IEEE 1284) */

/* Z80195
 *
 * The Z80195 MPU is a smart peripheral controller device designed for general data
 * communications applications, and architected specifically to accommodate all
 * input and output (I/O) requirements for serial and parallel connectivity.
 * Combining a high-performance CPU core with a variety of system and I/O
 * resources, the Z80195 is useful in a broad range of applications.
 *
 * Z80195 Features
 *
 *   Enhanced Z80 CPU
 *   1 MB MMU
 *   2 DMAs
 *   2 UARTs (up to 512 Kbps)
 *   Two 16-Bit Timers
 *   4 Counter/Timers
 *   Clock Serial I/O
 *   Power-Down Mode
 *   32 K ROM (185)
 *   1 Ch ESCC
 *   IEEE 1284 Bi-Directional Centronics Parallel Port
 *   7 or 24 Bits of I/O 
 */

#elif defined(CONFIG_ARCH_CHIP_Z8019520FSG) ||    /* 100-pin QFP 20MHz 5V */ \
      defined(CONFIG_ARCH_CHIP_Z8019533FSG)       /* 100-pin QFP 33MHz 5V */

#  undef  HAVE_Z8S180                             /* No Z8S180 (5V) or Z8L180 (3.3V) core */
#  undef  HAVE ROM       0                        /* No 32KB on-chip ROM (z80185 only) */
#  define HAVE_SERIALIO  1                        /* Have clocked serial I/O */
#  defube HAVE_WDT       1                        /* Have Watchdog timer */
#  define HAVE_NTIMERS16 2                        /* Two (2) 16-bit counter/timers */
#  define HAVE_NCTCS     4                        /* Four (4) Counter/Timers (CTCs) */
#  define HAVE_NDMA      2                        /* Two (2) DMA channels */
#  define HAVE_NUARTS    2                        /* Two (2) UARTs (up to 512 Kbps) */
#  define HAVE_NSCC      0                        /* No Z85C30 serial communication controller */
#  define HAVE_NESCC     1                        /* One (1) Enhanced Serial Communication Controllers (EMSCC) */
#  undef  HAVE_16550                              /* No 16550 MIMIC */
#  define HAVE_NIOLINES  0                        /* No I/O lines */
#  define HAVE_NPAR8     2                        /* Two (s) 8-bit parallel ports (or 17-bit IEEE 1284) */
#  define HAVE_IEEE1284  1                        /* Have bidirectional centronics interface (IEEE 1284) */

/* Z8L180
 *
 * The enhanced Z8S180/Z8L180 significantly improves on previous Z80180 models,
 * while still providing full backward compatibility with existing ZiLOG Z80
 * devices. The Z8S180/Z8L180 now offers faster execution speeds, power-saving
 * modes, and EMI noise reduction.
 *
 * Z8L180 Features
 *
 *   Enhanced Z80 CPU
 *   1 MB MMU
 *   2 DMAs*
 *   2 UARTs* (up to 512 Kbps)
 *   Two 16-Bit Timers
 *   Clock Serial I/O
 *   On-Chip Oscillator
 *   Power-Down Mode*
 *   Divide-by-One/Divide-by-Two/Multiply-by-Two Clock Options*
 *   Programmable Driver Strength for EMI Tuning
 *
 *   * Enhanced on the Z8S180 and Z8L180 MPUs.
 */

#elif defined(CONFIG_ARCH_CHIP_Z8L18020VSG) ||    /* 68-pinn PLCC */ \
      defined(CONFIG_ARCH_CHIP_Z8L18020FSG) ||    /* 80-pin GFP 20MHz 3.3V */ \
      defined(CONFIG_ARCH_CHIP_Z8L18020PSG)

#  define HAVE_Z8S180    1                        /* Uses Z8S180 (5V) or Z8L180 (3.3V) core */
#  define HAVE_Z8X180    1                        /* Z8x180 registers */
#  undef  HAVE_Z8X181                             /* Z8x181 registers */
#  undef  HAVE_Z8X182                             /* Z8x182 registers */
#  define HAVE ROM       0                        /* No on-chip ROM */
#  define HAVE_SERIALIO  1                        /* Have clocked serial I/O */
#  undef  HAVE_WDT                                /* No Watchdog timer */
#  define HAVE_NTIMERS16 2                        /* Two (2) 16-bit timers */
#  define HAVE_NCTCS     0                        /* No Counter/Timers (CTCs) */
#  define HAVE_NDMA      2                        /* Two (2) DMA channels */
#  define HAVE_NUARTS    2                        /* Two (2) UARTs (up to 512 Kbps) */
#  define HAVE_NSCC      0                        /* No serial communication controller */
#  define HAVE_NESCC     0                        /* No Enhanced Serial Communication Controllers */
#  undef  HAVE_16550                              /* No 16550 MIMIC */
#  define HAVE_NIOLINES  0                        /* No I/O lines */
#  define HAVE_NPAR8     0                        /* No 8-bit parallel ports */
#  undef  HAVE_IEEE1284                           /* No bidirectional centronics interface (IEEE 1284) */

/* Z8L182
 *
 * The Z80182/Z8L182 is a smart peripheral controller IC for modem (in particular
 * V. Fast applications), fax, voice messaging and other communications
 * applications. It uses the Z80180 microprocessor (Z8S180 MPU core) linked with
 * two channels of the industry standard Z85230 ESCC (Enhanced Serial
 * Communications Controller), 24 bits of parallel I/O, and a 16550 MIMIC for
 * direct connection to the IBM PC, XT, AT bus.
 *
 * Z8L182 Features
 *
 *   Enhanced Z80 CPU
 *   1 MB MMU
 *   2 DMAs
 *   2 UARTs (up to 512 Kbps)
 *   Two 16-Bit Timers
 *   Clock Serial I/O
 *   Power-Down Mode
 *   Divide-by-One/Divide-by-Two/Multiply-by-Two Clock Options
 *   ESCC (2 Channels) with 32-Bit CRC
 *   16550 MIMIC
 *   24 Parallel I/O
 *   3.3 V and 5 V Version
 */

#elif defined(CONFIG_ARCH_CHIP_Z8L18220ASG) ||    /* 100-pin LQFP */ \
      defined(CONFIG_ARCH_CHIP_Z8L18220FSG) ||    /* 100-pin QFP 20MHz 3.3V */ \
      defined(CONFIG_ARCH_CHIP_Z8L18220AEG)

#  define HAVE_Z8S180    1                        /* Uses Z8S180 (5V) or Z8L180 (3.3V) core */
#  undef  HAVE_Z8X180                             /* Z8x180 registers */
#  undef  HAVE_Z8X181                             /* Z8x181 registers */
#  define HAVE_Z8X182    1                        /* Z8x182 registers */
#  define HAVE ROM       0                        /* No on-chip ROM */
#  define HAVE_SERIALIO  1                        /* Have clocked serial I/O */
#  undef  HAVE_WDT                                /* No Watchdog timer */
#  define HAVE_NTIMERS16 2                        /* Two (2) 16-bit timers ? */
#  define HAVE_NCTCS     0                        /* No Counter/Timers (CTCs) */
#  define HAVE_NDMA      2                        /* Two (2) DMA channels */
#  define HAVE_NUARTS    2                        /* Two (2) UARTs (up to 512 Kbps) */
#  define HAVE_NSCC      0                        /* No Z85C30 serial communication controller */
#  define HAVE_NESCC     2                        /* Two (2) Z85230 Enhanced Serial Communication Controllers */
#  define HAVE_16550     1                        /* Have 16550 MIMIC */
#  define HAVE_NIOLINES  0                        /* No I/O lines */
#  define HAVE_NPAR8     3                        /* Three (3) 8-bit parallel ports (24-bit) */
#  undef  HAVE_IEEE1284                           /* No bidirectional centronics interface (IEEE 1284) */

/* Z8SL180
 *
 * The enhanced Z8S180/Z8L180 significantly improves on previous Z80180 models,
 * while still providing full backward compatibility with existing ZiLOG Z80
 * devices. The Z8S180/Z8L180 now offers faster execution speeds, power-saving
 * modes, and EMI noise reduction.This enhanced Z180 design also incorporates
 * additional feature enhancements to the ASCIs, DMAs, and STANDBY mode power
 * consumption. With the addition of ESCC-like Baud Rate Generators (BRGs), the
 * two ASCIs offer the flexibility and capability to transfer data
 * asynchronously at rates of up to 512 Kbps.
 *
 * Z8S180 Features
 *
 *   External Memory - 1
 *   Voltage Range - 5.0V
 *   Communications Controller - CSIO, UART
 *   Other Features - 1MB MMU, 2xDMA’s, 2xUARTs
 *   Speed (MHz) - 20, 10, 33
 *   Core / CPU Used - Z180
 *   Pin Count - 64, 68, 80
 *   Timers - 2
 *   I/O - Clock Serial
 *   Package - DIP, PLCC, QFP
 */

#elif defined(CONFIG_ARCH_CHIP_Z8S18020VSG) ||    /* 68-pin PLCC */ \
      defined(CONFIG_ARCH_CHIP_Z8S18020VSG1960) || /* 68-pin PLCC */ \
      defined(CONFIG_ARCH_CHIP_Z8S18033VSG) ||    /* 68-pin PLCC */ \
      defined(CONFIG_ARCH_CHIP_Z8S18010FSG) ||    /* 80-pin QFP */ \
      defined(CONFIG_ARCH_CHIP_Z8S18010VEG) ||    /* 68-pin PLCC */ \
      defined(CONFIG_ARCH_CHIP_Z8S18020VEG) ||    /* 68-pin PLCC */ \
      defined(CONFIG_ARCH_CHIP_Z8S18010VSG) ||    /* 68-pin PLCC */ \
      defined(CONFIG_ARCH_CHIP_Z8S18020PSG) ||    /* 64-pin DIP 10Mhz 5V */ \
      defined(CONFIG_ARCH_CHIP_Z8S18033FSG) ||    /* 80-pin QFP 33MHz 5V */ \
      defined(CONFIG_ARCH_CHIP_Z8S18033FEG) ||    /* 80-pin QFP 33MHz 5V */ \
      defined(CONFIG_ARCH_CHIP_Z8S18020FSG) ||    /* 80-pin QFP 20MHz 5V */ \
      defined(CONFIG_ARCH_CHIP_Z8S18033VEG) ||    /* 68-pin PLCC 33MHz 5V */ \
      defined(CONFIG_ARCH_CHIP_Z8S18010PSG) ||    /* 64-pin DIP 10MHz 5V */ \
      defined(CONFIG_ARCH_CHIP_Z8S18020FEG) || \
      defined(CONFIG_ARCH_CHIP_Z8S18010PEG) || \
      defined(CONFIG_ARCH_CHIP_Z8S18010FEG)

#  define HAVE_Z8S180    1                        /* Uses Z8S180 (5V) or Z8L180 (3.3V) core */
#  define HAVE_Z8X180    1                        /* Z8x180 registers */
#  undef  HAVE_Z8X181                             /* Z8x181 registers */
#  undef  HAVE_Z8X182                             /* Z8x182 registers */
#  define HAVE ROM       0                        /* No on-chip ROM */
#  define HAVE_SERIALIO  1                        /* Have clocked serial I/O */
#  undef  HAVE_WDT                                /* No Watchdog timer */
#  define HAVE_NTIMERS16 2                        /* Two (2) 16-bit timers */
#  define HAVE_NCTCS     0                        /* No Counter/Timers (CTCs) */
#  define HAVE_NDMA      2                        /* Two (2) DMA channels */
#  define HAVE_NUARTS    2                        /* Two (2) UARTs (up to 512 Kbps) */
#  define HAVE_NSCC      0                        /* No serial communication controller */
#  define HAVE_NESCC     0                        /* No Enhanced Serial Communication Controllers */
#  undef  HAVE_16550                              /* No 16550 MIMIC */
#  define HAVE_NIOLINES  0                        /* No I/O lines */
#  define HAVE_NPAR8     0                        /* No 8-bit parallel ports */
#  undef  HAVE_IEEE1284                           /* No bidirectional centronics interface (IEEE 1284) */

#else
#  error "Unrecognized/undefined Z180 chip"
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_Z80_INCLUDE_Z180_CHIP_H */
