/********************************************************************************************************
 * arch/arm/src/lpc31xx/lpc31_evntrtr.h
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_EVNTRTR_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_EVNTRTR_H

/********************************************************************************************************
 * Included Files
 ********************************************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/********************************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************************/

/* EVNTRTR register base address offset into the APB0 domain ********************************************/

#define LPC31_EVNTRTR_VBASE                      (LPC31_APB0_VADDR+LPC31_APB0_EVNTRTR_OFFSET)
#define LPC31_EVNTRTR_PBASE                      (LPC31_APB0_PADDR+LPC31_APB0_EVNTRTR_OFFSET)

/* Sizes of things */

#define LPC31_EVNTRTR_NBANKS                     4                 /* Banks b=0-3 */
#define LPC31_EVNTRTR_NOUTPUTS                   5                 /* Outputs o=0-4 (incl CGU Wakeup) */
#define LPC31_EVNTRTR_NEVENTS                    (32*LPC31_EVNTRTR_NBANKS)

#define _B(b)                                    ((b)<<2)          /* Maps bank number 0-3 to word offset */
#define _O(o)                                    ((o)<<5)          /* Maps output to bank group offset */
#define _OB(o,b)                                 (_O(o)+_B(b))     /* Mqpw output and bank to word offset */

#define EVNTRTR_EVENT(bank,bit)                  ((bank)<<5|bit)   /* Makes a event number from a bank and bit */
#define EVNTRTR_BANK(e)                          ((e)>>5)          /* Maps a event to a bank */
#define EVNTRTR_BIT(e)                           ((e)&0x1f)        /* Maps a event to a bit */

/* EVNTRTR register offsets (with respect to the EVNTRTR base) ******************************************/

                                                                   /* 0x0000-0x0bff: Reserved */
#define LPC31_EVNTRTR_PEND_OFFSET(b)             (0x0c00+_B(b))    /* Input event pending */
#define LPC31_EVNTRTR_INTCLR_OFFSET(b)           (0x0c20+_B(b))    /* Input event clear */
#define LPC31_EVNTRTR_INTSET_OFFSET(b)           (0x0c40+_B(b))    /* Input event set */
#define LPC31_EVNTRTR_MASK_OFFSET(b)             (0x0c60+_B(b))    /* Input event mask */
#define LPC31_EVNTRTR_MASKCLR_OFFSET(b)          (0x0c80+_B(b))    /* Input event mask clear */
#define LPC31_EVNTRTR_MASKSET_OFFSET(b)          (0x0ca0+_B(b))    /* Input event mask set */
#define LPC31_EVNTRTR_APR_OFFSET(b)              (0x0cc0+_B(b))    /* Input event activation polarity */
#define LPC31_EVNTRTR_ATR_OFFSET(b)              (0x0ce0+_B(b))    /* Input event activation type */
#define LPC31_EVNTRTR_RSR_OFFSET(b)              (0x0d20+_B(b))    /* Input event raw status */
#define LPC31_EVNTRTR_INTOUT_OFFSET              0x0d40            /* State of interrupt output pins */
                                                                   /* 0x0e00-0x0ffc: Reserved */
#define LPC31_EVNTRTR_INTOUTPEND_OFFSET(o,b)     (0x1000+_OB(o,b)) /* Interrupt output 'o' pending */
#define LPC31_EVNTRTR_CGUWKUPPEND_OFFSET(b)      (0x1000+_OB(4,b)) /* cgu_wakeup pending */
#define LPC31_EVNTRTR_INTOUTMASK_OFFSET(o,b)     (0x1400+_OB(o,b)) /* Interrupt output 'o' mask */
#define LPC31_EVNTRTR_CGUWKUPMASK_OFFSET(b)      (0x1400+_OB(4,b)) /* cgu_wakeup mask */
#define LPC31_EVNTRTR_INTOUTMASKCLR_OFFSET(o,b)  (0x1800+_OB(o,b)) /* Interrupt output 'o' mask clear */
#define LPC31_EVNTRTR_CGUWKUPMASKCLR_OFFSET(b)   (0x1800+_OB(4,b)) /* cgu_wakeup mask clear */
#define LPC31_EVNTRTR_INTOUTMASKSET_OFFSET(o,b)  (0x1c00+_OB(o,b)) /* Interrupt output 'o' mask set */
#define LPC31_EVNTRTR_CGUWKUPMASKSET_OFFSET(b)   (0x1c00+_OB(4,b)) /* cgu_wakeup mask set */

/* EVNTRTR register (virtual) addresses *********************************************************************/

#define LPC31_EVNTRTR_PEND(b)                    (LPC31_EVNTRTR_VBASE+LPC31_EVNTRTR_PEND_OFFSET(b))
#define LPC31_EVNTRTR_INTCLR(b)                  (LPC31_EVNTRTR_VBASE+LPC31_EVNTRTR_INTCLR_OFFSET(b))
#define LPC31_EVNTRTR_INTSET(b)                  (LPC31_EVNTRTR_VBASE+LPC31_EVNTRTR_INTSET_OFFSET(b))
#define LPC31_EVNTRTR_MASK(b)                    (LPC31_EVNTRTR_VBASE+LPC31_EVNTRTR_MASK_OFFSET(b))
#define LPC31_EVNTRTR_MASKCLR(b)                 (LPC31_EVNTRTR_VBASE+LPC31_EVNTRTR_MASKCLR_OFFSET(b))
#define LPC31_EVNTRTR_MASKSET(b)                 (LPC31_EVNTRTR_VBASE+LPC31_EVNTRTR_MASKSET_OFFSET(b))
#define LPC31_EVNTRTR_APR(b)                     (LPC31_EVNTRTR_VBASE+LPC31_EVNTRTR_APR_OFFSET(b))
#define LPC31_EVNTRTR_ATR(b)                     (LPC31_EVNTRTR_VBASE+LPC31_EVNTRTR_ATR_OFFSET(b))
#define LPC31_EVNTRTR_RSR(b)                     (LPC31_EVNTRTR_VBASE+LPC31_EVNTRTR_RSR_OFFSET(b))
#define LPC31_EVNTRTR_INTOUT                     (LPC31_EVNTRTR_VBASE+LPC31_EVNTRTR_INTOUT_OFFSET)
#define LPC31_EVNTRTR_INTOUTPEND(o,b)            (LPC31_EVNTRTR_VBASE+LPC31_EVNTRTR_INTOUTPEND_OFFSET(o,b))
#define LPC31_EVNTRTR_CGUWKUPPEND(b)             (LPC31_EVNTRTR_VBASE+LPC31_EVNTRTR_CGUWKUPPEND_OFFSET(b))
#define LPC31_EVNTRTR_INTOUTMASK(o,b)            (LPC31_EVNTRTR_VBASE+LPC31_EVNTRTR_INTOUTMASK_OFFSET(o,b))
#define LPC31_EVNTRTR_CGUWKUPMASK(b)             (LPC31_EVNTRTR_VBASE+LPC31_EVNTRTR_CGUWKUPMASK_OFFSET(b))
#define LPC31_EVNTRTR_INTOUTMASKCLR(o,b)         (LPC31_EVNTRTR_VBASE+LPC31_EVNTRTR_INTOUTMASKCLR_OFFSET(o,b))
#define LPC31_EVNTRTR_CGUWKUPMASKCLR(b)          (LPC31_EVNTRTR_VBASE+LPC31_EVNTRTR_CGUWKUPMASKCLR_OFFSET(b))
#define LPC31_EVNTRTR_INTOUTMASKSET(o,b)         (LPC31_EVNTRTR_VBASE+LPC31_EVNTRTR_INTOUTMASKSET_OFFSET(o,b))
#define LPC31_EVNTRTR_CGUWKUPMASKSET(b)          (LPC31_EVNTRTR_VBASE+LPC31_EVNTRTR_CGUWKUPMASKSET_OFFSET(b)

/* EVNTRTR event definitions ********************************************************************************/
/* Bank 0 */

#define EVENTRTR_EBID6                             EVNTRTR_EVENT(0,31) /* Input event from GPIO pin */
#define EVENTRTR_EBID5                             EVNTRTR_EVENT(0,30) /* Input event from GPIO pin */
#define EVENTRTR_EBID4                             EVNTRTR_EVENT(0,29) /* Input event from GPIO pin */
#define EVENTRTR_EBID3                             EVNTRTR_EVENT(0,28) /* Input event from GPIO pin */
#define EVENTRTR_EBID2                             EVNTRTR_EVENT(0,27) /* Input event from GPIO pin */
#define EVENTRTR_EBID1                             EVNTRTR_EVENT(0,26) /* Input event from GPIO pin */
#define EVENTRTR_EBID0                             EVNTRTR_EVENT(0,25) /* Input event from GPIO pin */
#define EVENTRTR_MNANDRYBN3                        EVNTRTR_EVENT(0,24) /* Input event from GPIO pin */
#define EVENTRTR_MNANDRYBN2                        EVNTRTR_EVENT(0,23) /* Input event from GPIO pin */
#define EVENTRTR_MNANDRYBN1                        EVNTRTR_EVENT(0,22) /* Input event from GPIO pin */
#define EVENTRTR_MNANDRYBN0                        EVNTRTR_EVENT(0,21) /* Input event from GPIO pin */
#define EVENTRTR_MLCDRWWR                          EVNTRTR_EVENT(0,20) /* Input event from GPIO pin */
#define EVENTRTR_MLCDERD                           EVNTRTR_EVENT(0,19) /* Input event from GPIO pin */
#define EVENTRTR_MLCDCSB                           EVNTRTR_EVENT(0,18) /* Input event from GPIO pin */
#define EVENTRTR_MLCDRS                            EVNTRTR_EVENT(0,17) /* Input event from GPIO pin */
#define EVENTRTR_MLCDDB15                          EVNTRTR_EVENT(0,16) /* Input event from GPIO pin */
#define EVENTRTR_MLCDDB14                          EVNTRTR_EVENT(0,15) /* Input event from GPIO pin */
#define EVENTRTR_MLCDDB13                          EVNTRTR_EVENT(0,14) /* Input event from GPIO pin */
#define EVENTRTR_MLCDDB12                          EVNTRTR_EVENT(0,13) /* Input event from GPIO pin */
#define EVENTRTR_MLCDDB11                          EVNTRTR_EVENT(0,12) /* Input event from GPIO pin */
#define EVENTRTR_MLCDDB10                          EVNTRTR_EVENT(0,11) /* Input event from GPIO pin */
#define EVENTRTR_MLCDDB9                           EVNTRTR_EVENT(0,10) /* Input event from GPIO pin */
#define EVENTRTR_MLCDDB8                           EVNTRTR_EVENT(0,9)  /* Input event from GPIO pin */
#define EVENTRTR_MLCDDB7                           EVNTRTR_EVENT(0,8)  /* Input event from GPIO pin */
#define EVENTRTR_MLCDDB6                           EVNTRTR_EVENT(0,7)  /* Input event from GPIO pin */
#define EVENTRTR_MLCDDB5                           EVNTRTR_EVENT(0,6)  /* Input event from GPIO pin */
#define EVENTRTR_MLCDDB4                           EVNTRTR_EVENT(0,5)  /* Input event from GPIO pin */
#define EVENTRTR_MLCDDB3                           EVNTRTR_EVENT(0,4)  /* Input event from GPIO pin */
#define EVENTRTR_MLCDDB2                           EVNTRTR_EVENT(0,3)  /* Input event from GPIO pin */
#define EVENTRTR_MLCDDB1                           EVNTRTR_EVENT(0,2)  /* Input event from GPIO pin */
#define EVENTRTR_MLCDDB0                           EVNTRTR_EVENT(0,1)  /* Input event from GPIO pin */
#define EVENTRTR_PCMINT                            EVNTRTR_EVENT(0,0)  /* Input event from PCM */

/* Bank 1 */

#define EVENTRTR_GPIO16                            EVNTRTR_EVENT(1,31) /* Input event from GPIO pin */
#define EVENTRTR_GPIO15                            EVNTRTR_EVENT(1,30) /* Input event from GPIO pin */
#define EVENTRTR_GPIO14                            EVNTRTR_EVENT(1,29) /* Input event from GPIO pin */
#define EVENTRTR_GPIO13                            EVNTRTR_EVENT(1,28) /* Input event from GPIO pin */
#define EVENTRTR_GPIO12                            EVNTRTR_EVENT(1,27) /* Input event from GPIO pin */
#define EVENTRTR_GPIO11                            EVNTRTR_EVENT(1,26) /* Input event from GPIO pin */
#define EVENTRTR_MGPIO10                           EVNTRTR_EVENT(1,25) /* Input event from GPIO pin */
#define EVENTRTR_MGPIO9                            EVNTRTR_EVENT(1,24) /* Input event from GPIO pin */
#define EVENTRTR_MGPIO8                            EVNTRTR_EVENT(1,23) /* Input event from GPIO pin */
#define EVENTRTR_MGPIO7                            EVNTRTR_EVENT(1,22) /* Input event from GPIO pin */
#define EVENTRTR_MGPIO6                            EVNTRTR_EVENT(1,21) /* Input event from GPIO pin */
#define EVENTRTR_MGPIO5                            EVNTRTR_EVENT(1,20) /* Input event from GPIO pin */
#define EVENTRTR_GPIO4                             EVNTRTR_EVENT(1,19) /* Input event from GPIO pin */
#define EVENTRTR_GPIO3                             EVNTRTR_EVENT(1,18) /* Input event from GPIO pin */
#define EVENTRTR_GPIO2                             EVNTRTR_EVENT(1,17) /* Input event from GPIO pin */
#define EVENTRTR_GPIO1                             EVNTRTR_EVENT(1,16) /* Input event from GPIO pin */
#define EVENTRTR_GPIO0                             EVNTRTR_EVENT(1,15) /* Input event from GPIO pin */
#define EVENTRTR_EBINRASBLOUT1                     EVNTRTR_EVENT(1,14) /* Input event from GPIO pin */
#define EVENTRTR_EBINCASBLOUT0                     EVNTRTR_EVENT(1,13) /* Input event from GPIO pin */
#define EVENTRTR_EBIDQM0NOE                        EVNTRTR_EVENT(1,12) /* Input event from GPIO pin */
#define EVENTRTR_EBIA1CLE                          EVNTRTR_EVENT(1,11) /* Input event from GPIO pin */
#define EVENTRTR_EBIA0ALE                          EVNTRTR_EVENT(1,10) /* Input event from GPIO pin */
#define EVENTRTR_EBINWE                            EVNTRTR_EVENT(1,9)  /* Input event from GPIO pin */
#define EVENTRTR_EBID15                            EVNTRTR_EVENT(1,8)  /* Input event from GPIO pin */
#define EVENTRTR_EBID14                            EVNTRTR_EVENT(1,7)  /* Input event from GPIO pin */
#define EVENTRTR_EBID13                            EVNTRTR_EVENT(1,6)  /* Input event from GPIO pin */
#define EVENTRTR_EBID12                            EVNTRTR_EVENT(1,5)  /* Input event from GPIO pin */
#define EVENTRTR_EBID11                            EVNTRTR_EVENT(1,4)  /* Input event from GPIO pin */
#define EVENTRTR_EBID10                            EVNTRTR_EVENT(1,3)  /* Input event from GPIO pin */
#define EVENTRTR_EBID9                             EVNTRTR_EVENT(1,2)  /* Input event from GPIO pin */
#define EVENTRTR_EBID8                             EVNTRTR_EVENT(1,1)  /* Input event from GPIO pin */
#define EVENTRTR_EBID7                             EVNTRTR_EVENT(1,0)  /* Input event from GPIO pin */

/* Bank 2 */

#define EVENTRTR_PWMDATA                           EVNTRTR_EVENT(2,31) /* Input event from GPIO pin */
#define EVENTRTR_I2CSCL1                           EVNTRTR_EVENT(2,30) /* Input event from GPIO pin */
#define EVENTRTR_I2CSDA1                           EVNTRTR_EVENT(2,29) /* Input event from GPIO pin */
#define EVENTRTR_CLK256FSO                         EVNTRTR_EVENT(2,28) /* Input event from GPIO pin */
#define EVENTRTR_I2STXWS1                          EVNTRTR_EVENT(2,27) /* Input event from GPIO pin */
#define EVENTRTR_I2STXBCK1                         EVNTRTR_EVENT(2,26) /* Input event from GPIO pin */
#define EVENTRTR_I2STXDATA1                        EVNTRTR_EVENT(2,25) /* Input event from GPIO pin */
#define EVENTRTR_I2SRXWS1                          EVNTRTR_EVENT(2,24) /* Input event from GPIO pin */
#define EVENTRTR_I2SRXBCK1                         EVNTRTR_EVENT(2,23) /* Input event from GPIO pin */
#define EVENTRTR_I2SRXDATA1                        EVNTRTR_EVENT(2,22) /* Input event from GPIO pin */
#define EVENTRTR_I2SRXWS0                          EVNTRTR_EVENT(2,21) /* Input event from GPIO pin */
#define EVENTRTR_I2SRXDATA0                        EVNTRTR_EVENT(2,20) /* Input event from GPIO pin */
#define EVENTRTR_I2SRXBCK0                         EVNTRTR_EVENT(2,19) /* Input event from GPIO pin */
#define EVENTRTR_MI2STXWS0                         EVNTRTR_EVENT(2,18) /* Input event from GPIO pin */
#define EVENTRTR_MI2STXDATA0                       EVNTRTR_EVENT(2,17) /* Input event from GPIO pin */
#define EVENTRTR_MI2STXBCK0                        EVNTRTR_EVENT(2,16) /* Input event from GPIO pin */
#define EVENTRTR_MI2STXCLK0                        EVNTRTR_EVENT(2,15) /* Input event from GPIO pin */
#define EVENTRTR_MUARTRTSN                         EVNTRTR_EVENT(2,14) /* Input event from GPIO pin */
#define EVENTRTR_MUARTCTSN                         EVNTRTR_EVENT(2,13) /* Input event from GPIO pin */
#define EVENTRTR_UARTTXD                           EVNTRTR_EVENT(2,12) /* Input event from GPIO pin */
#define EVENTRTR_UARTRXD                           EVNTRTR_EVENT(2,11) /* Input event from GPIO pin */
#define EVENTRTR_SPICSOUT0                         EVNTRTR_EVENT(2,10) /* Input event from GPIO pin */
#define EVENTRTR_SPISCK                            EVNTRTR_EVENT(2,9)  /* Input event from GPIO pin */
#define EVENTRTR_SPICSIN                           EVNTRTR_EVENT(2,8)  /* Input event from GPIO pin */
#define EVENTRTR_SPIMOSI                           EVNTRTR_EVENT(2,7)  /* Input event from GPIO pin */
#define EVENTRTR_SPIMISO                           EVNTRTR_EVENT(2,6)  /* Input event from GPIO pin */
#define EVENTRTR_NANDNCS3                          EVNTRTR_EVENT(2,5)  /* Input event from GPIO pin */
#define EVENTRTR_NANDNCS2                          EVNTRTR_EVENT(2,4)  /* Input event from GPIO pin */
#define EVENTRTR_NANDNCS1                          EVNTRTR_EVENT(2,3)  /* Input event from GPIO pin */
#define EVENTRTR_NANDNCS0                          EVNTRTR_EVENT(2,2)  /* Input event from GPIO pin */
#define EVENTRTR_GPIO18                            EVNTRTR_EVENT(2,1)  /* Input event from GPIO pin */
#define EVENTRTR_GPIO17                            EVNTRTR_EVENT(2,0)  /* Input event from GPIO pin */

/* Bank 3 */
                                                                       /* 30-31: Reserved */
#define EVENTRTR_ISRAM1MRCFINISHED                 EVNTRTR_EVENT(3,29) /* ISRAM1 redundancy controller event */
#define EVENTRTR_ISRAM0MRCFINISHED                 EVNTRTR_EVENT(3,28) /* ISRAM0 redundancy controller event */
#define EVENTRTR_USBID                             EVNTRTR_EVENT(3,27) /* Input event from GPIO pin */
#define EVENTRTR_USBOTGVBUSPWREN                   EVNTRTR_EVENT(3,26) /* Input event from USB */
#define EVENTRTR_USBATXPLLLOCK                     EVNTRTR_EVENT(3,25) /* USB PLL lock event */
#define EVENTRTR_USBOTGAHBNEEDCLK                  EVNTRTR_EVENT(3,24) /* Input event from USB */
#define EVENTRTR_USBVBUS                           EVNTRTR_EVENT(3,23) /* Input event from USB VBUS pin */
#define EVENTRTR_MCICLK                            EVNTRTR_EVENT(3,22) /* Input event from GPIO pin */
#define EVENTRTR_MCICMD                            EVNTRTR_EVENT(3,21) /* Input event from GPIO pin */
#define EVENTRTR_MCIDAT7                           EVNTRTR_EVENT(3,20) /* Input event from GPIO pin */
#define EVENTRTR_MCIDAT6                           EVNTRTR_EVENT(3,19) /* Input event from GPIO pin */
#define EVENTRTR_MCIDAT5                           EVNTRTR_EVENT(3,18) /* Input event from GPIO pin */
#define EVENTRTR_MCIDAT4                           EVNTRTR_EVENT(3,17) /* Input event from GPIO pin */
#define EVENTRTR_MCIDAT3                           EVNTRTR_EVENT(3,16) /* Input event from GPIO pin */
#define EVENTRTR_MCIDAT2                           EVNTRTR_EVENT(3,15) /* Input event from GPIO pin */
#define EVENTRTR_MCIDAT1                           EVNTRTR_EVENT(3,14) /* Input event from GPIO pin */
#define EVENTRTR_MCIDAT0                           EVNTRTR_EVENT(3,13) /* Input event from GPIO pin */
#define EVENTRTR_ARM926LPNIRQ                      EVNTRTR_EVENT(3,12) /* Reflects nIRQ signal to ARM core */
#define EVENTRTR_ARM926LPNFIQ                      EVNTRTR_EVENT(3,11) /* Reflects nFIQ signal to ARM core */
#define EVENTRTR_I2C1SCLN                          EVNTRTR_EVENT(3,10) /* Input event from I2C1 */
#define EVENTRTR_I2C0SCLN                          EVNTRTR_EVENT(3,9)  /* Input event from I2C0 */
#define EVENTRTR_UART                              EVNTRTR_EVENT(3,8)  /* Input event from UART */
#define EVENTRTR_WDOGM0                            EVNTRTR_EVENT(3,7)  /* Input event from Watchdog Timer */
#define EVENTRTR_ADCINT                            EVNTRTR_EVENT(3,6)  /* Input event from ADC */
#define EVENTRTR_TIMER3INTCT1                      EVNTRTR_EVENT(3,5)  /* Input event from Timer 3 */
#define EVENTRTR_TIMER2INTCT1                      EVNTRTR_EVENT(3,4)  /* Input event from Timer 2 */
#define EVENTRTR_TIMER1INTCT1                      EVNTRTR_EVENT(3,3)  /* Input event from Timer 1 */
#define EVENTRTR_TIMER0INTCT1                      EVNTRTR_EVENT(3,2)  /* Input event from Timer 0 */
#define EVENTRTR_GPIO20                            EVNTRTR_EVENT(3,1)  /* Input event from GPIO20 */
#define EVENTRTR_GPIO19                            EVNTRTR_EVENT(3,0)  /* Input event from GPIO19 */

/********************************************************************************************************
 * Public Types
 ********************************************************************************************************/

/********************************************************************************************************
 * Public Data
 ********************************************************************************************************/

/********************************************************************************************************
 * Public Functions
 ********************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_EVNTRTR_H */
