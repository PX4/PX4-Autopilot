/************************************************************************************
 * arch/sh/src/m16c/m16c_timer.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_SH_SRC_M16C_M16C_TIMER_H
#define __ARCH_SH_SRC_M16C_M16C_TIMER_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Register Bit Definitions *********************************************************/

#define M16C_TA0IC         0x00055    /* Timer A0 interrupt control */
#define M16C_TA1IC         0x00056    /* Timer A1 interrupt control */
#define M16C_TA2IC         0x00057    /* Timer A2 interrupt control */
#define M16C_TA3IC         0x00058    /* Timer A3 interrupt control */
#define M16C_TA4IC         0x00059    /* Timer A4 interrupt control */

/* Count Start Flag Register (8-bit access) Register */

#define TABSR_TA0S         0x01       /* Bit 0: Timer A0 count start */
#define TABSR_TA1S         0x02       /* Bit 1: Timer A1 count start */
#define TABSR_TA2S         0x04       /* Bit 2: Timer A2 count start */
#define TABSR_TA3S         0x08       /* Bit 3: Timer A3 count start */
#define TABSR_TA4S         0x10       /* Bit 4: Timer A4 count start */
#define TABSR_TB0S         0x20       /* Bit 5: Timer B0 count start */
#define TABSR_TB1S         0x40       /* Bit 6: Timer B1 count start */
#define TABSR_TB2S         0x80       /* Bit 7: Timer B2 count start */

/* Clock Prescaler Reset Flag Register */
                                      /* Bits 0-6: Not used */
#define CPSRF_CPSR         0x80       /* Bit 7: 1=Prescaler is reset */

/* On-Shot Start Flag Register (8-bit access) */

#define ONSF_TA0OS         0x01       /* Bit 0: Timer A0 one shot start */
#define ONSF_TA1OS         0x02       /* Bit 1: Timer A1 one shot start */
#define ONSF_TA2OS         0x04       /* Bit 2: Timer A2 one shot start */
#define ONSF_TA3OS         0x08       /* Bit 3: Timer A3 one shot start */
#define ONSF_TA4OS         0x10       /* Bit 4: Timer A4 one shot start */
                                      /* Bit 5: Reserved */
#define ONSF_TA0TG_MASK    0xc0       /* Bit 6-7: Timer A0 event trigger select bits */
#define ONSF_TAOTG_INTAON  0x00       /*   00 : Input on TA0IN is selected */
#define ONSF_TAOTG_TB2OVF  0x40       /*   01 : TB2 overflow is selected */
#define ONSF_TAOTG_TB4OVF  0x80       /*   10 : TB4 overflow is selected */
#define ONSF_TAOTG_TB1OVF  0xc0       /*   11 : TB1 overflow is selected */

/* Trigger Select Register */

#define TRGSR_TA1TG_MASK   0x03       /* Bit 0-1: Timer A1 event trigger select bits */
#define TRGSR_TA1TG_INTAON 0x00       /*   00 : Input on TA1IN is selected */
#define TRGSR_TA1TG_TB2OVF 0x01       /*   01 : TB2 overflow is selected */
#define TRGSR_TA1TG_TB4OVF 0x02       /*   10 : TB4 overflow is selected */
#define TRGSR_TA1TG_TB1OVF 0x03       /*   11 : TB1 overflow is selected */
#define TRGSR_TA2TG_MASK   0x0c       /* Bit 2-3: Timer A2 event trigger select bits */
#define TRGSR_TA2TG_INTAON 0x00       /*   00 : Input on TA2IN is selected */
#define TRGSR_TA2TG_TB2OVF 0x04       /*   01 : TB2 overflow is selected */
#define TRGSR_TA2TG_TB4OVF 0x08       /*   10 : TB4 overflow is selected */
#define TRGSR_TA2TG_TB1OVF 0x0c       /*   11 : TB1 overflow is selected */
#define TRGSR_TA3TG_MASK   0x30       /* Bit 4-5: Timer A3 event trigger select bits */
#define TRGSR_TA3TG_INTAON 0x00       /*   00 : Input on TA3IN is selected */
#define TRGSR_TA3TG_TB2OVF 0x10       /*   01 : TB2 overflow is selected */
#define TRGSR_TA3TG_TB4OVF 0x20       /*   10 : TB4 overflow is selected */
#define TRGSR_TA3TG_TB1OVF 0x30       /*   11 : TB1 overflow is selected */
#define TRGSR_TA4TG_MASK   0xc0       /* Bit 6-7: Timer A4 event trigger select bits */
#define TRGSR_TA4TG_INTAON 0x00       /*   00 : Input on TA4IN is selected */
#define TRGSR_TA4TG_TB2OVF 0x40       /*   01 : TB2 overflow is selected */
#define TRGSR_TA4TG_TB4OVF 0x80       /*   10 : TB4 overflow is selected */
#define TRGSR_TA4TG_TB1OVF 0xc0       /*   11 : TB1 overflow is selected */

/* Up-Down Flag Register */

#define UDF_TA0UD          0x01       /* Bit 0: 1=Timer A0 up count */
#define UDF_TA1UD          0x02       /* Bit 1: 1=Timer A1 up count */
#define UDF_TA2UD          0x04       /* Bit 2: 1=Timer A2 up count */
#define UDF_TA3UD          0x08       /* Bit 3: 1=Timer A3 up count */
#define UDF_TA4UD          0x10       /* Bit 4: 1=Timer A4 up count */
#define UDF_TA2P           0x20       /* Bit 5: Timer A2 two-phase pulse signal processing select */
#define UDF_TA3P           0x40       /* Bit 5: Timer A3 two-phase pulse signal processing select */
#define UDF_TA4P           0x80       /* Bit 5: Timer A4 two-phase pulse signal processing select */

/* Timer A Registers (16-bit access), simple value range 0000-ffff
 *                                    (except in PWM mode)
 */

/* Timer B Registers (16-bit access), simple value range 0000-ffff
 *                                    (except in Pulse period/pulse width measurement mode)
 */

/* Timer A Mode Register (8-bit access) */

#define TAnMR_TMOD_MASK    0x03       /* Bits 0-1: Operation mode select */
#define TAnMR_TMOD_TIMER   0x00       /*   00 : Timer mode */
#define TAnMR_TMOD_EVENT   0x01       /*   01 : Event counter mode */
#define TAnMR_TMOD_ONESHOT 0x02       /*   10 : One-shot timer mode */
#define TAnMR_TMOD_PWM     0x03       /*   11 : Pulse width modulation (PWM) mode */
#define TAnMR_MR_MASK      0x3c       /* Bits 2-5: Mode function values */
                                      /* Timer Mode: */
#define TAnMR_MR_TMNOOUT   0x00       /*   0xx0 : No output */
#define TAnMR_MR_TMOUT     0x04       /*   0xx1 : Pulse is output */
#define TAnMR_MR_TMNOGATE  0x00       /*   00xx : Gate function not available */
#define TAnMR_MR_TMTAINLO  0x10       /*   010x : Timer counts when TAiIN pin is L */
#define TAnMR_MR_TMTAINHI  0x18       /*   011x : Timer counts when TAiIN pin is H */
                                      /* Event Counter Mode: */
#define TAnMR_MR_EC2PHASE  0x10       /*   0100 : Settings required for 2-phase mode */
#define TAnMR_MR_ECNOOUT   0x00       /*   0xx0 : No output */
#define TAnMR_MR_ECOUT     0x04       /*   0xx1 : 1=Pulse is output */
#define TAnMR_MR_ECFALLING 0x00       /*   0x0x : Count polarity falling edge */
#define TAnMR_MR_ECRISING  0x08       /*   0x1x : Count polarity rising edge */
#define TAnMR_MR_ECUDC     0x00       /*   00xx : Up/down switching on up/down content */
#define TAnMR_MR_ECINP     0x10       /*   01xx : Up/down switching on TAnOUT input signal */
                                      /* One Shot Mode: */
#define TAnMR_MR_OSNOOUT   0x00       /*   0xx0 : No output */
#define TAnMR_MR_OSOUT     0x04       /*   0xx1 : Pulse is output */
#define TAnMR_MR_OSFALLING 0x00       /*   0x0x : TAin falling edge */
#define TAnMR_MR_OSRISING  0x08       /*   0x1x : TAin rising edge */
#define TAnMR_MR_OSSFLAG   0x00       /*   00xx : Trigger select one-shot start flag */
#define TAnMR_MR_OSSTRIG   0x10       /*   01xx : Trigger Selected by event/trigger select bits */
                                      /* PWM Mode: */
#define TAnMR_MR_PMFALLING 0x00       /*   xx00 : TAin falling edge */
#define TAnMR_MR_PMRISING  0x08       /*   xx10 : TAin rising edge */
#define TAnMR_MR_PMSFLAG   0x00       /*   x0x0 : Trigger select one-shot start flag */
#define TAnMR_MR_PMTRIG    0x10       /*   x1x0 : Trigger Selected by event/trigger select bits */
#define TAnMR_MR_PM16BIT   0x00       /*   0xx0 : Functions as a 16-bit pulse width modulator */
#define TAnMR_MR_PM8BIT    0x20       /*   1xx0 : Functions as an 8-bit pulse width modul */
#define TAnMR_TCK_MASK     0xc0       /* Bits 6-7: Count source select */
                                      /* Timer Mode: */
#define TAnMR_TCK_TMF1     0x00       /*   00: f1 or f2 */
#define TAnMR_TCK_TMF8     0x40       /*   01: f8 */
#define TAnMR_TCK_TMF32    0x80       /*   10: f32 */
#define TAnMR_TCK_TMFC32   0xc0       /*   11: fc32 */
                                      /* Event Counter Mode: */
#define TAnMR_TCK_ECRELOAD 0x00       /*   x0: Reload count operation*/
#define TAnMR_TCK_ECFRUN   0x40       /*   x1: Free run count operation*/
#define TAnMR_TCK_ECNORMAL 0x00       /*   0x: Normal processing operation */
#define TAnMR_TCK_ECMUL4   0x80       /*   1x: Multiply-by-4 processing operation */
                                      /* One Shot Mode: */
#define TAnMR_TCK_OSF1     0x00       /*   00: f1 or f2 */
#define TAnMR_TCK_OSF8     0x40       /*   01: f8 */
#define TAnMR_TCK_OSF32    0x80       /*   10: f32 */
#define TAnMR_TCK_OSFC32   0xc0       /*   11: fc32 */
                                      /* PWM Mode: */
#define TAnMR_TCK_PMF1     0x00       /*   00: f1 or f2 */
#define TAnMR_TCK_PMF8     0x40       /*   01: f8 */
#define TAnMR_TCK_PMF32    0x80       /*   10: f32 */
#define TAnMR_TCK_PMFC32   0xc0       /*   11: fc32 */

/* Timer B Mode Register (8-bit access) */

#define TBnMR_TMOD_MASK    0x03       /* Bits 0-1: Operation mode select */
#define TBnMR_TMOD_TIMER   0x00       /*   00 : Timer mode */
#define TBnMR_TMOD_EVENT   0x01       /*   01 : Event counter mode */
#define TBnMR_TMOD_PWM     0x02       /*   10 : Pulse period/pulse width measurement mode */
#define TBnMR_MR_MASK      0x3c       /* Bits 2-5: Mode function values */
                                      /* Timer Mode: */
#define TBnMR_MR_TM        0x00       /*   0000 : Required bit settings for timer mode */
                                      /* Event Counter Mode: */
#define TBnMR_MR_ECFALLING 0x00       /*   0000 : Counts external signal's falling edges */
#define TBnMR_MR_ECRISING  0x04       /*   0001 : Counts external signal's rising edges */
#define TBnMR_MR_ECXTFALL  0x00       /*   0010 : Counts external signal's falling and rising edges */
                                      /* Pulse period/pulse width measurement mode: */
#define TBnMR_MR_PMFALLING 0x00       /*   0000 : Period between falling edge to falling edge */
#define TBnMR_MR_PMRISING  0x08       /*   0001 : Period between rising edge to rising edge */
#define TBnMR_MR_PMSVAL    0x00       /*   0010 : Width between edge(s) to edge(s) */
#define TBnMR_TCK_MASK     0xc0       /* Bits 6-7: Count source select */
                                      /* Timer Mode: */
#define TBnMR_TCK_TMF1     0x00       /*   00: f1 or f2 */
#define TBnMR_TCK_TMF8     0x40       /*   01: f8 */
#define TBnMR_TCK_TMF32    0x80       /*   10: f32 */
#define TBnMR_TCK_TMFC32   0xc0       /*   11: fc32 */
                                      /* Event Counter Mode: */
#define TBnMR_TCK_ECTBIN   0x00       /*   00: Input from TBnIN pin */
#define TBnMR_TCK_ECTBOVF  0x80       /*   10: TBj overflow */
                                      /* Pulse period/pulse width measurement mode: */
#define TBnMR_TCK_PMF1     0x00       /*   00: f1 or f2 */
#define TBnMR_TCK_PMF8     0x40       /*   01: f8 */
#define TBnMR_TCK_PMF32    0x80       /*   10: f32 */
#define TBnMR_TCK_PMFC32   0xc0       /*   11: fc32 */

/************************************************************************************
 * Global Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_SH_SRC_M16C_M16C_TIMER_H */
