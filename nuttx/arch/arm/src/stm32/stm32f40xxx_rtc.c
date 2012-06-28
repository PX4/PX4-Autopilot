/************************************************************************************
 * arch/arm/src/stm32/stm32f40xxx_rtc.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/rtc.h>

#include <time.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "up_arch.h"

#include "stm32_rtc.h"

#ifdef CONFIG_RTC

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* This RTC implementation supports only date/time RTC hardware */

#ifndef CONFIG_RTC_DATETIME
#  error "CONFIG_RTC_DATETIME must be set to use this driver"
#endif

#ifdef CONFIG_RTC_HIRES
#  error "CONFIG_RTC_HIRES must NOT be set with this driver"
#endif

#ifndef CONFIG_STM32_PWR
#  error "CONFIG_STM32_PWR must selected to use this driver"
#endif

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_RTC
#endif

/* Constants ************************************************************************/

#define SYNCHRO_TIMEOUT  (0x00020000)
#define INITMODE_TIMEOUT (0x00010000)
#define RTC_MAGIC        (0xfacefeed)
#define RTC_PREDIV_S     (0xff)
#define RTC_PREDIV_A     (0x7f)

/* Debug ****************************************************************************/

#ifdef CONFIG_DEBUG_RTC
#  define rtcdbg    dbg
#  define rtcvdbg   vdbg
#  define rtclldbg  lldbg
#  define rtcllvdbg llvdbg
#else
#  define rtcdbg(x...)
#  define rtcvdbg(x...)
#  define rtclldbg(x...)
#  define rtcllvdbg(x...)
#endif

/************************************************************************************
 * Private Types
 ************************************************************************************/

/************************************************************************************
 * Private Data
 ************************************************************************************/

/* Callback to use when the alarm expires */

#ifdef CONFIG_RTC_ALARM
static alarmcb_t g_alarmcb;
#endif

/************************************************************************************
 * Public Data
 ************************************************************************************/

/* g_rtc_enabled is set true after the RTC has successfully initialized */

volatile bool g_rtc_enabled = false;

/************************************************************************************
 * Private Functions
 ************************************************************************************/
/************************************************************************************
 * Name: rtc_dumpregs
 *
 * Description:
 *    Disable RTC write protection
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_RTC
static void rtc_dumpregs(FAR const char *msg)
{
  rtclldbg("%s:\n", msg);
  rtclldbg("      TR: %08x\n", getreg32(STM32_RTC_TR));
  rtclldbg("      DR: %08x\n", getreg32(STM32_RTC_DR));
  rtclldbg("      CR: %08x\n", getreg32(STM32_RTC_CR));
  rtclldbg("     ISR: %08x\n", getreg32(STM32_RTC_ISR));
  rtclldbg("    PRER: %08x\n", getreg32(STM32_RTC_PRER));
  rtclldbg("    WUTR: %08x\n", getreg32(STM32_RTC_WUTR));
  rtclldbg("  CALIBR: %08x\n", getreg32(STM32_RTC_CALIBR));
  rtclldbg("  ALRMAR: %08x\n", getreg32(STM32_RTC_ALRMAR));
  rtclldbg("  ALRMBR: %08x\n", getreg32(STM32_RTC_ALRMBR));
  rtclldbg("  SHIFTR: %08x\n", getreg32(STM32_RTC_SHIFTR));
  rtclldbg("    TSTR: %08x\n", getreg32(STM32_RTC_TSTR));
  rtclldbg("    TSDR: %08x\n", getreg32(STM32_RTC_TSDR));
  rtclldbg("   TSSSR: %08x\n", getreg32(STM32_RTC_TSSSR));
  rtclldbg("    CALR: %08x\n", getreg32(STM32_RTC_CALR));
  rtclldbg("   TAFCR: %08x\n", getreg32(STM32_RTC_TAFCR));
  rtclldbg("ALRMASSR: %08x\n", getreg32(STM32_RTC_ALRMASSR));
  rtclldbg("ALRMBSSR: %08x\n", getreg32(STM32_RTC_ALRMBSSR));
  rtclldbg("     BK0: %08x\n", getreg32(STM32_RTC_BK0R));
}
#else
#  define rtc_dumpregs(msg)
#endif

/************************************************************************************
 * Name: rtc_dumptime
 *
 * Description:
 *    Disable RTC write protection
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

#ifdef CONFIG_DEBUG_RTC
static void rtc_dumptime(FAR struct tm *tp, FAR const char *msg)
{
  rtclldbg("%s:\n", msg);
  rtclldbg("  tm_sec: %08x\n", tp->tm_sec);
  rtclldbg("  tm_min: %08x\n", tp->tm_min);
  rtclldbg(" tm_hour: %08x\n", tp->tm_hour);
  rtclldbg(" tm_mday: %08x\n", tp->tm_mday);
  rtclldbg("  tm_mon: %08x\n", tp->tm_mon);
  rtclldbg(" tm_year: %08x\n", tp->tm_year);
}
#else
#  define rtc_dumptime(tp, msg)
#endif

/************************************************************************************
 * Name: rtc_wprunlock
 *
 * Description:
 *    Disable RTC write protection
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static void rtc_wprunlock(void)
{
  /* The following steps are required to unlock the write protection on all the
   * RTC registers (except for RTC_ISR[13:8], RTC_TAFCR, and RTC_BKPxR).
   *
   * 1. Write 0xCA into the RTC_WPR register.
   * 2. Write 0x53 into the RTC_WPR register.
   *
   * Writing a wrong key reactivates the write protection.
   */

  putreg32(0xca, STM32_RTC_WPR);
  putreg32(0x53, STM32_RTC_WPR);
}

/************************************************************************************
 * Name: rtc_wprunlock
 *
 * Description:
 *    Enable RTC write protection
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

static inline void rtc_wprlock(void)
{
  /* Writing any wrong key reactivates the write protection. */

  putreg32(0xff, STM32_RTC_WPR);
}

/************************************************************************************
 * Name: rtc_synchwait
 *
 * Description:
 *   Waits until the RTC Time and Date registers (RTC_TR and RTC_DR) are
 *   synchronized with RTC APB clock.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

static int rtc_synchwait(void)
{
  volatile uint32_t timeout;
  uint32_t regval;
  int ret;

  /* Disable the write protection for RTC registers */

  rtc_wprunlock();

  /* Clear Registers synchronization flag (RSF) */
 
  regval = getreg32(STM32_RTC_ISR);
  regval &= ~RTC_ISR_RSF;
  putreg32(regval, STM32_RTC_ISR);

  /* Now wait the registers to become synchronised */

  ret = -ETIMEDOUT;
  for (timeout = 0; timeout < SYNCHRO_TIMEOUT; timeout++)
    {
      regval = getreg32(STM32_RTC_ISR);
      if ((regval & RTC_ISR_RSF) != 0)
        {
          /* Synchronized */

          ret = OK;
          break;
        }
    }

  /* Re-enable the write protection for RTC registers */

  rtc_wprlock();
  return ret;
}

/************************************************************************************
 * Name: rtc_enterinit
 *
 * Description:
 *   Enter RTC initialization mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

static int rtc_enterinit(void)
{
  volatile uint32_t timeout;
  uint32_t regval;
  int ret;
     
  /* Check if the Initialization mode is already set */

  regval = getreg32(STM32_RTC_ISR);

  ret = OK;
  if ((regval & RTC_ISR_INITF) == 0)
    {
      /* Set the Initialization mode */

      putreg32(RTC_ISR_INIT, STM32_RTC_ISR);
    
      /* Wait until the RTC is in the INIT state (or a timeout occurs) */
 
      ret = -ETIMEDOUT;
      for (timeout = 0; timeout < INITMODE_TIMEOUT; timeout++)
        {
          regval = getreg32(STM32_RTC_ISR);
          if ((regval & RTC_ISR_INITF) != 0)
            {
              ret = OK;
              break;
            }
        }
    }

  return ret;  
}

/************************************************************************************
 * Name: rtc_exitinit
 *
 * Description:
 *   Exit RTC initialization mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

static void rtc_exitinit(void)
{ 
  uint32_t regval;

  regval = getreg32(STM32_RTC_ISR);
  regval &= ~(RTC_ISR_INIT);
  putreg32(regval, STM32_RTC_ISR);
}

/************************************************************************************
 * Name: rtc_bin2bcd
 *
 * Description:
 *   Converts a 2 digit binary to BCD format
 *
 * Input Parameters:
 *   value - The byte to be converted.
 *
 * Returned Value:
 *   The value in BCD representation
 *
 ************************************************************************************/

static uint32_t rtc_bin2bcd(int value)
{
  uint32_t msbcd = 0;
  
  while (value >= 10)
    {
      msbcd++;
      value -= 10;
    }
  
  return (msbcd << 4) | value;
}

/************************************************************************************
 * Name: rtc_bin2bcd
 *
 * Description:
 *   Convert from 2 digit BCD to binary.
 *
 * Input Parameters:
 *   value - The BCD value to be converted.
 *
 * Returned Value:
 *   The value in binary representation
 *
 ************************************************************************************/

static int rtc_bcd2bin(uint32_t value)
{
  uint32_t tens = (value >> 4) * 10;
  return (int)(tens + (value & 0x0f));
}

/************************************************************************************
 * Name: rtc_setup
 *
 * Description:
 *   Performs first time configuration of the RTC.  A special value written into
 *   back-up register 0 will prevent this function from being called on sub-sequent
 *   resets or power up.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

static int rtc_setup(void)
{
  uint32_t regval;
  int ret;

  /* Enable the External Low-Speed (LSE) Oscillator setup the LSE as the RTC clock\
   * source, and enable the RTC.
   */

  stm32_rcc_enablelse();

  /* Wait for the RTC Time and Date registers to be synchronized with RTC APB
   * clock.
   */

  ret = rtc_synchwait();
  if (ret == OK)
    {
      /* Disable the write protection for RTC registers */

      rtc_wprunlock();

      /* Set Initialization mode */

      ret = rtc_enterinit();
      if (ret == OK)
        {
          /* Set the 24 hour format by clearing the FMT bit in the RTC
           * control register
           */

          regval = getreg32(STM32_RTC_CR);
          regval &= ~RTC_CR_FMT;
          putreg32(regval, STM32_RTC_CR);

          /* Configure RTC pre-scaler to the required, default values for
           * use with the 32.768 KHz LSE clock:
           */

          putreg32(((uint32_t)0xff << RTC_PRER_PREDIV_S_SHIFT) |
                   ((uint32_t)0x7f << RTC_PRER_PREDIV_A_SHIFT),
                   STM32_RTC_PRER);

          /* Exit RTC initialization mode */

          rtc_exitinit();
        }

      /* Re-enable the write protection for RTC registers */

      rtc_wprlock();
    }
  return ret;
}

/************************************************************************************
 * Name: rtc_resume
 *
 * Description:
 *   Called when the RTC was already initialized on a previous power cycle.  This
 *   just brings the RTC back into full operation.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

static int rtc_resume(void)
{
#ifdef CONFIG_RTC_ALARM
  uint32_t regval;
#endif
  int ret;

  /* Wait for the RTC Time and Date registers to be syncrhonized with RTC APB
   * clock.
   */

  ret = rtc_synchwait();

  /* Clear the RTC alarm flags */

#ifdef CONFIG_RTC_ALARM
  regval  = getreg32(STM32_RTC_ISR);
  regval &= ~(RTC_ISR_ALRAF|RTC_ISR_ALRBF);
  putreg32(regval, STM32_RTC_ISR);

  /* Clear the EXTI Line 17 Pending bit (Connected internally to RTC Alarm) */

  putreg32((1 << 17), STM32_EXTI_PR);
#endif
  return ret;
}

/************************************************************************************
 * Name: rtc_interrupt
 *
 * Description:
 *    RTC interrupt service routine
 *
 * Input Parameters:
 *   irq - The IRQ number that generated the interrupt
 *   context - Architecture specific register save information.
 *
 * Returned Value:
 *   Zero (OK) on success; A negated errno value on failure.
 *
 ************************************************************************************/

#if CONFIG_RTC_ALARM
static int rtc_interrupt(int irq, void *context)
{
#warning "Missing logic"
  return OK;
}
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_rtcinitialize
 *
 * Description:
 *   Initialize the hardware RTC per the selected configuration.  This function is
 *   called once during the OS initialization sequence
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int up_rtcinitialize(void)
{
  uint32_t regval;
  int ret;

  rtc_dumpregs("On reset");

  /* Clocking for the PWR block must be provided.  However, this is done
   * unconditionally in stm32f40xxx_rcc.c on power up.  This done unconditionally
   * because the PWR block is also needed to set the internal voltage regulator for
   * maximum performance.
   */

  /* Enable access to the backup domain (RTC registers, RTC backup data registers
   * and backup SRAM).
   */

  stm32_pwr_enablebkp();

  /* Check if the one-time initialization of the RTC has already been performed.
   * We can determine this by checking if the magic number has been writing to
   * to back-up date register DR0.
   */

  regval = getreg32(STM32_RTC_BK0R);
  if (regval != RTC_MAGIC)
    {
      /* Perform the one-time setup of the LSE clocking to the RTC */

      ret = rtc_setup();

      /* Remember that the RTC is initialized */

      putreg32(RTC_MAGIC, STM32_RTC_BK0R);
    }
  else
    {
      /* RTC already set-up, just resume normal operation */

      ret = rtc_resume();
    }

  /* Configure RTC interrupt to catch alarm interrupts. All RTC interrupts are
   * connected to the EXTI controller.  To enable the RTC Alarm interrupt, the
   * following sequence is required:
   *
   * 1. Configure and enable the EXTI Line 17 in interrupt mode and select the
   *    rising edge sensitivity.
   * 2. Configure and enable the RTC_Alarm IRQ channel in the NVIC.
   * 3. Configure the RTC to generate RTC alarms (Alarm A or Alarm B).
   */

#ifdef CONFIG_RTC_ALARM
#  warning "Missing EXTI setup logic"

  /* Then attach the ALARM interrupt handler */

  irq_attach(STM32_IRQ_RTC, rtc_interrupt);
  up_enable_irq(STM32_IRQ_RTC);
#endif

  g_rtc_enabled = true;
  rtc_dumpregs("After Initialization"); 
  return OK;
}

/************************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used by the RTOS during
 *   intialization to set up the system time when CONFIG_RTC and CONFIG_RTC_DATETIME
 *   are selected (and CONFIG_RTC_HIRES is not).
 *
 *   NOTE: Some date/time RTC hardware is capability of sub-second accuracy.  That
 *   sub-second accuracy is lost in this interface.  However, since the system time
 *   is reinitialized on each power-up/reset, there will be no timing inaccuracy in
 *   the long run.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int up_rtc_getdatetime(FAR struct tm *tp)
{
  uint32_t dr;
  uint32_t tr;
  uint32_t tmp;

  /* Sample the data time registers.  There is a race condition here... If we sample
   * the time just before midnight on December 31, the date could be wrong because
   * the day rolled over while were sampling.
   */

  do
    {
      dr  = getreg32(STM32_RTC_DR);
      tr  = getreg32(STM32_RTC_TR);
      tmp = getreg32(STM32_RTC_DR);
    }
  while (tmp != dr);

  rtc_dumpregs("Reading Time");

  /* Convert the RTC time to fields in struct tm format.  All of the STM32
   * All of the ranges of values correspond between struct tm and the time
   * register.
   */

  tmp = (tr & (RTC_TR_SU_MASK|RTC_TR_ST_MASK)) >> RTC_TR_SU_SHIFT;
  tp->tm_sec = rtc_bcd2bin(tmp);

  tmp = (tr & (RTC_TR_MNU_MASK|RTC_TR_MNT_MASK)) >> RTC_TR_MNU_SHIFT;
  tp->tm_min = rtc_bcd2bin(tmp);

  tmp = (tr & (RTC_TR_HU_MASK|RTC_TR_HT_MASK)) >> RTC_TR_HU_SHIFT;
  tp->tm_hour = rtc_bcd2bin(tmp);

  /* Now convert the RTC date to fields in struct tm format:
   * Days: 1-31 match in both cases.
   * Month: STM32 is 1-12, struct tm is 0-11.
   * Years: STM32 is 00-99, struct tm is years since 1900.
   *
   * Issue:  I am not sure what the STM32 years mean.  Are these the
   * years 2000-2099?  I'll assume so.
   */

  tmp = (dr & (RTC_DR_DU_MASK|RTC_DR_DT_MASK)) >> RTC_DR_DU_SHIFT;
  tp->tm_mday = rtc_bcd2bin(tmp);

  tmp = (dr & (RTC_DR_MU_MASK|RTC_DR_MT)) >> RTC_DR_MU_SHIFT;
  tp->tm_mon = rtc_bcd2bin(tmp) - 1;

  tmp = (dr & (RTC_DR_YU_MASK|RTC_DR_YT_MASK)) >> RTC_DR_YU_SHIFT;
  tp->tm_year = rtc_bcd2bin(tmp) + 100;

  rtc_dumptime(tp, "Returning");
  return OK;
}

/************************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be able to
 *   set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int up_rtc_settime(FAR const struct timespec *tp)
{
  FAR struct tm newtime;
  uint32_t tr;
  uint32_t dr;
  int ret;

  /* Break out the time values (not that the time is set only to units of seconds) */

  (void)gmtime_r(&tp->tv_sec, &newtime);
  rtc_dumptime(&newtime, "Setting time");

  /* Then write the broken out values to the RTC */

  /* Convert the struct tm format to RTC time register fields.  All of the STM32
   * All of the ranges of values correspond between struct tm and the time
   * register.
   */

  tr = (rtc_bin2bcd(newtime.tm_sec)  << RTC_TR_SU_SHIFT) |
       (rtc_bin2bcd(newtime.tm_min)  << RTC_TR_MNU_SHIFT) |
       (rtc_bin2bcd(newtime.tm_hour) << RTC_TR_HU_SHIFT);
  tr &= ~RTC_TR_RESERVED_BITS;

  /* Now convert the fields in struct tm format to the RTC date register fields:
   * Days: 1-31 match in both cases.
   * Month: STM32 is 1-12, struct tm is 0-11.
   * Years: STM32 is 00-99, struct tm is years since 1900.
   *
   * Issue:  I am not sure what the STM32 years mean.  Are these the
   * years 2000-2099?  I'll assume so.
   */

  dr = (rtc_bin2bcd(newtime.tm_mday) << RTC_DR_DU_SHIFT) |
       ((rtc_bin2bcd(newtime.tm_mon + 1))  << RTC_DR_MU_SHIFT) |
       ((rtc_bin2bcd(newtime.tm_year - 100)) << RTC_DR_YU_SHIFT);
  dr &= ~RTC_DR_RESERVED_BITS;

  /* Disable the write protection for RTC registers */

  rtc_wprunlock();

  /* Set Initialization mode */

  ret = rtc_enterinit();
  if (ret == OK)
    {
      /* Set the RTC TR and DR registers */

      putreg32(tr, STM32_RTC_TR);
      putreg32(dr, STM32_RTC_DR);

      /* Exit Initialization mode and wait for the RTC Time and Date
       * registers to be synchronized with RTC APB clock.
       */

      rtc_exitinit();
      ret = rtc_synchwait();
    }

  /* Re-enable the write protection for RTC registers */

  rtc_wprlock();
  rtc_dumpregs("New time setting");
  return ret;
}

/************************************************************************************
 * Name: up_rtc_setalarm
 *
 * Description:
 *   Set up an alarm.  Up to two alarms can be supported (ALARM A and ALARM B).
 *
 * Input Parameters:
 *   tp - the time to set the alarm
 *   callback - the function to call when the alarm expires.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

#ifdef CONFIG_RTC_ALARM
int up_rtc_setalarm(FAR const struct timespec *tp, alarmcb_t callback);
{
  irqstate_t flags;
  int ret = -EBUSY;

  /* Is there already something waiting on the ALARM? */

  if (g_alarmcb == NULL)
    {
      /* No.. Save the callback function pointer */

      g_alarmcb = callback;

      /* Break out the time values */
#warning "Missing logic"

      /* The set the alarm */
#warning "Missing logic"

      ret = OK;
    }
  return ret;
}
#endif

#endif /* CONFIG_RTC */

