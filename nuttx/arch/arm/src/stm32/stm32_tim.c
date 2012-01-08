/************************************************************************************
 * arm/arm/src/stm32/stm32_tim.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
 *
 * With modifications and updates by:
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_internal.h"
#include "up_arch.h"

#include "stm32_internal.h"
#include "stm32_gpio.h"
#include "stm32_tim.h"

/************************************************************************************
 * Private Types
 ************************************************************************************/
/* Configuration ********************************************************************/
/* Timer devices may be used for different purposes.  Such special purposes include:
 *
 * - To generate modulated outputs for such things as motor control.  If CONFIG_STM32_TIMn
 *   is defined then the CONFIG_STM32_TIMn_PWM may also be defined to indicate that
 *   the timer is intended to be used for pulsed output modulation.
 *
 * - To control periodic ADC input sampling.  If CONFIG_STM32_TIMn is defined then 
 *   CONFIG_STM32_TIMn_ADC may also be defined to indicate that timer "n" is intended
 *   to be used for that purpose.
 *
 * - To control periodic DAC outputs.  If CONFIG_STM32_TIMn is defined then 
 *   CONFIG_STM32_TIMn_DAC may also be defined to indicate that timer "n" is intended
 *   to be used for that purpose.
 *
 * In any of these cases, the timer will not be used by this timer module.
 */

#if defined(CONFIG_STM32_TIM1_PWM) || defined (CONFIG_STM32_TIM1_ADC) || defined(CONFIG_STM32_TIM1_DAC)
#  undef CONFIG_STM32_TIM1
#endif
#if defined(CONFIG_STM32_TIM2_PWM) || defined (CONFIG_STM32_TIM2_ADC) || defined(CONFIG_STM32_TIM2_DAC)
#  undef CONFIG_STM32_TIM2
#endif
#if defined(CONFIG_STM32_TIM3_PWM) || defined (CONFIG_STM32_TIM3_ADC) || defined(CONFIG_STM32_TIM3_DAC)
#  undef CONFIG_STM32_TIM3
#endif
#if defined(CONFIG_STM32_TIM4_PWM) || defined (CONFIG_STM32_TIM4_ADC) || defined(CONFIG_STM32_TIM4_DAC)
#  undef CONFIG_STM32_TIM4
#endif
#if defined(CONFIG_STM32_TIM5_PWM) || defined (CONFIG_STM32_TIM5_ADC) || defined(CONFIG_STM32_TIM5_DAC)
#  undef CONFIG_STM32_TIM5
#endif
#if defined(CONFIG_STM32_TIM6_PWM) || defined (CONFIG_STM32_TIM6_ADC) || defined(CONFIG_STM32_TIM6_DAC)
#  undef CONFIG_STM32_TIM6
#endif
#if defined(CONFIG_STM32_TIM7_PWM) || defined (CONFIG_STM32_TIM7_ADC) || defined(CONFIG_STM32_TIM7_DAC)
#  undef CONFIG_STM32_TIM7
#endif
#if defined(CONFIG_STM32_TIM8_PWM) || defined (CONFIG_STM32_TIM8_ADC) || defined(CONFIG_STM32_TIM8_DAC)
#  undef CONFIG_STM32_TIM8
#endif
#if defined(CONFIG_STM32_TIM9_PWM) || defined (CONFIG_STM32_TIM9_ADC) || defined(CONFIG_STM32_TIM9_DAC)
#  undef CONFIG_STM32_TIM9
#endif
#if defined(CONFIG_STM32_TIM10_PWM) || defined (CONFIG_STM32_TIM10_ADC) || defined(CONFIG_STM32_TIM10_DAC)
#  undef CONFIG_STM32_TIM10
#endif
#if defined(CONFIG_STM32_TIM11_PWM) || defined (CONFIG_STM32_TIM11_ADC) || defined(CONFIG_STM32_TIM11_DAC)
#  undef CONFIG_STM32_TIM11
#endif
#if defined(CONFIG_STM32_TIM12_PWM) || defined (CONFIG_STM32_TIM12_ADC) || defined(CONFIG_STM32_TIM12_DAC)
#  undef CONFIG_STM32_TIM12
#endif
#if defined(CONFIG_STM32_TIM13_PWM) || defined (CONFIG_STM32_TIM13_ADC) || defined(CONFIG_STM32_TIM13_DAC)
#  undef CONFIG_STM32_TIM13
#endif
#if defined(CONFIG_STM32_TIM14_PWM) || defined (CONFIG_STM32_TIM14_ADC) || defined(CONFIG_STM32_TIM14_DAC)
#  undef CONFIG_STM32_TIM14
#endif

/* This module then only compiles if there are enabled timers that are not intended for
 * some other purpose.
 */

#if defined(CONFIG_STM32_TIM1) || defined(CONFIG_STM32_TIM2) || defined(CONFIG_STM32_TIM3) || \
    defined(CONFIG_STM32_TIM4) || defined(CONFIG_STM32_TIM5) || defined(CONFIG_STM32_TIM6) || \
    defined(CONFIG_STM32_TIM7) || defined(CONFIG_STM32_TIM8)

/************************************************************************************
 * Private Types
 ************************************************************************************/

/** TIM Device Structure 
 */
struct stm32_tim_priv_s {
    struct stm32_tim_ops_s *ops;
    stm32_tim_mode_t        mode;
    uint32_t                base;   /** TIMn base address */
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/** Get register value by offset */
static inline uint16_t stm32_tim_getreg(FAR struct stm32_tim_dev_s *dev, uint8_t offset)
{
    return getreg16( ((struct stm32_tim_priv_s *)dev)->base + offset);
}

/** Put register value by offset */
static inline void stm32_tim_putreg(FAR struct stm32_tim_dev_s *dev, uint8_t offset, uint16_t value)
{
    //printf("putreg(%8x)=%4x\n", ((struct stm32_tim_priv_s *)dev)->base + offset, value );
    putreg16(value, ((struct stm32_tim_priv_s *)dev)->base + offset);
}

/** Modify register value by offset */
static inline void stm32_tim_modifyreg(FAR struct stm32_tim_dev_s *dev, uint8_t offset, uint16_t clearbits, uint16_t setbits)
{
    modifyreg16( ((struct stm32_tim_priv_s *)dev)->base + offset, clearbits, setbits);
}

static void stm32_tim_reload_counter(FAR struct stm32_tim_dev_s *dev)
{
    uint16_t val = stm32_tim_getreg(dev, STM32_BTIM_EGR_OFFSET);
    val |= ATIM_EGR_UG;
    stm32_tim_putreg(dev, STM32_BTIM_EGR_OFFSET, val);    
}

static void stm32_tim_enable(FAR struct stm32_tim_dev_s *dev)
{
    uint16_t val = stm32_tim_getreg(dev, STM32_BTIM_CR1_OFFSET);
    val |= ATIM_CR1_CEN;
    stm32_tim_reload_counter(dev);
    stm32_tim_putreg(dev, STM32_BTIM_CR1_OFFSET, val);
}

static void stm32_tim_disable(FAR struct stm32_tim_dev_s *dev)
{
    uint16_t val = stm32_tim_getreg(dev, STM32_BTIM_CR1_OFFSET);
    val &= ~ATIM_CR1_CEN;
    stm32_tim_putreg(dev, STM32_BTIM_CR1_OFFSET, val);
}

/** Reset timer into system default state, but do not affect output/input pins */
static void stm32_tim_reset(FAR struct stm32_tim_dev_s *dev)
{
    ((struct stm32_tim_priv_s *)dev)->mode = STM32_TIM_MODE_DISABLED;
    stm32_tim_disable(dev);
}

static void stm32_tim_gpioconfig(uint32_t cfg, stm32_tim_channel_t mode)
{
    /** \todo Added support for input capture and bipolar dual outputs for TIM8 */

    if (mode & STM32_TIM_CH_MODE_MASK) {
        stm32_configgpio(cfg);
    } 
    else {
        stm32_unconfiggpio(cfg);
    }
}

/************************************************************************************
 * Basic Functions
 ************************************************************************************/

static int stm32_tim_setclock(FAR struct stm32_tim_dev_s *dev, uint32_t freq)
{
    int prescaler;
    
    ASSERT(dev);
    
    /* Disable Timer? */
    if (freq == 0) {
        stm32_tim_disable(dev);
        return 0;
    }
    
#if STM32_NATIM > 0    
    if (((struct stm32_tim_priv_s *)dev)->base == STM32_TIM1_BASE || 
        ((struct stm32_tim_priv_s *)dev)->base == STM32_TIM8_BASE)
        prescaler = STM32_TIM18_FREQUENCY / freq;
    else 
#endif
        prescaler = STM32_TIM27_FREQUENCY / freq;
        
    /* we need to decrement value for '1', but only, if we are allowed to
     * not to cause underflow. Check for overflow.
     */
    if (prescaler > 0) prescaler--;
    if (prescaler > 0xFFFF) prescaler = 0xFFFF;
    
    stm32_tim_putreg(dev, STM32_BTIM_PSC_OFFSET, prescaler);    
    stm32_tim_enable(dev);
    
    return prescaler;
}

static void stm32_tim_setperiod(FAR struct stm32_tim_dev_s *dev, uint16_t period)
{
    ASSERT(dev);
    stm32_tim_putreg(dev, STM32_BTIM_ARR_OFFSET, period);
}

static int stm32_tim_setisr(FAR struct stm32_tim_dev_s *dev, int (*handler)(int irq, void *context), int source)
{
    int vectorno;

    ASSERT(dev);
    ASSERT(source==0);
        
    switch( ((struct stm32_tim_priv_s *)dev)->base ) {
#if CONFIG_STM32_TIM2
        case STM32_TIM2_BASE: vectorno = STM32_IRQ_TIM2;    break;
#endif
#if CONFIG_STM32_TIM3
        case STM32_TIM3_BASE: vectorno = STM32_IRQ_TIM3;    break;
#endif
#if CONFIG_STM32_TIM4
        case STM32_TIM4_BASE: vectorno = STM32_IRQ_TIM4;    break;
#endif
#if CONFIG_STM32_TIM5
        case STM32_TIM5_BASE: vectorno = STM32_IRQ_TIM5;    break;
#endif
#if STM32_NBTIM > 0
#if CONFIG_STM32_TIM6
        case STM32_TIM6_BASE: vectorno = STM32_IRQ_TIM6;    break;
#endif
#endif
#if STM32_NBTIM > 1
#if CONFIG_STM32_TIM7
        case STM32_TIM7_BASE: vectorno = STM32_IRQ_TIM7;    break;
#endif
#endif
#if STM32_NATIM > 0
        /** \todo add support for multiple sources and callbacks */
#if CONFIG_STM32_TIM1
        case STM32_TIM1_BASE: vectorno = STM32_IRQ_TIM1UP;  break;
#endif
#if CONFIG_STM32_TIM8
        case STM32_TIM8_BASE: vectorno = STM32_IRQ_TIM8UP;  break;
#endif
#endif
        default: return ERROR;
    }
    
    /* Disable interrupt when callback is removed */
    
    if (!handler) {
        up_disable_irq(vectorno);
        irq_detach(vectorno);
        return OK;
    }
    
    /* Otherwise set callback and enable interrupt */

    irq_attach(vectorno, handler);
    up_enable_irq(vectorno);
//  up_prioritize_irq(vectorno, NVIC_SYSH_PRIORITY_DEFAULT);
    return OK;
}

static void stm32_tim_enableint(FAR struct stm32_tim_dev_s *dev, int source)
{
    ASSERT(dev);
    stm32_tim_modifyreg(dev, STM32_BTIM_DIER_OFFSET, 0, ATIM_DIER_UIE);
}

static void stm32_tim_disableint(FAR struct stm32_tim_dev_s *dev, int source)
{
    ASSERT(dev);
    stm32_tim_modifyreg(dev, STM32_BTIM_DIER_OFFSET, ATIM_DIER_UIE, 0);
}

static void stm32_tim_ackint(FAR struct stm32_tim_dev_s *dev, int source)
{
    stm32_tim_putreg(dev, STM32_BTIM_SR_OFFSET, ~ATIM_SR_UIF);
}

/************************************************************************************
 * General Functions
 ************************************************************************************/

static int stm32_tim_setmode(FAR struct stm32_tim_dev_s *dev, stm32_tim_mode_t mode)
{
    uint16_t val = ATIM_CR1_CEN | ATIM_CR1_ARPE;
    
    ASSERT(dev);
    
    /* This function is not supported on basic timers. To enable or 
     * disable it, simply set its clock to valid frequency or zero.
     */
    
#if STM32_NBTIM > 0
    if (((struct stm32_tim_priv_s *)dev)->base == STM32_TIM6_BASE 
#endif
#if STM32_NBTIM > 1
        ||  ((struct stm32_tim_priv_s *)dev)->base == STM32_TIM7_BASE
#endif
#if STM32_NBTIM > 0
    ) return ERROR;
#endif

    /* Decode operational modes */
    
    switch(mode & STM32_TIM_MODE_MASK) {
    
        case STM32_TIM_MODE_DISABLED:
            val = 0;
            break;
        
        case STM32_TIM_MODE_DOWN:
            val |= ATIM_CR1_DIR;

        case STM32_TIM_MODE_UP:
            break;
            
        case STM32_TIM_MODE_UPDOWN:
            val |= ATIM_CR1_CENTER1;
            // Our default: Interrupts are generated on compare, when counting down
            break;

        case STM32_TIM_MODE_PULSE:
            val |= ATIM_CR1_OPM;
            break;
        
        default: return ERROR;
    }
    
    stm32_tim_reload_counter(dev);    
    stm32_tim_putreg(dev, STM32_BTIM_CR1_OFFSET, val);
    
#if STM32_NATIM > 0
    /* Advanced registers require Main Output Enable */
    
    if (((struct stm32_tim_priv_s *)dev)->base == STM32_TIM1_BASE ||
        ((struct stm32_tim_priv_s *)dev)->base == STM32_TIM8_BASE) {
        stm32_tim_modifyreg(dev, STM32_ATIM_BDTR_OFFSET, 0, ATIM_BDTR_MOE);
    }
#endif
    
    return OK;
}

static int stm32_tim_setchannel(FAR struct stm32_tim_dev_s *dev, uint8_t channel, stm32_tim_channel_t mode)
{
    uint16_t ccmr_val = 0;
    uint16_t ccer_val = stm32_tim_getreg(dev, STM32_GTIM_CCER_OFFSET);
    uint8_t  ccmr_offset = STM32_GTIM_CCMR1_OFFSET;
    
    ASSERT(dev);
    
    /* Further we use range as 0..3; if channel=0 it will also overflow here */
    
    if (--channel > 4) return ERROR;
        
    /* Assume that channel is disabled and polarity is active high */
    
    ccer_val &= ~(3 << (channel << 2));

    /* This function is not supported on basic timers. To enable or 
     * disable it, simply set its clock to valid frequency or zero.
     */
    
#if STM32_NBTIM > 0
    if (   ((struct stm32_tim_priv_s *)dev)->base == STM32_TIM6_BASE 
#endif
#if STM32_NBTIM > 1
        || ((struct stm32_tim_priv_s *)dev)->base == STM32_TIM7_BASE
#endif
#if STM32_NBTIM > 0
       ) return ERROR;
#endif

    /* Decode configuration */
    
    switch(mode & STM32_TIM_CH_MODE_MASK) {
    
        case STM32_TIM_CH_DISABLED:
            break;
    
        case STM32_TIM_CH_OUTPWM:
            ccmr_val  =  (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC1M_SHIFT) + ATIM_CCMR1_OC1PE;
            ccer_val |= ATIM_CCER_CC1E << (channel << 2);
            break;
            
        default:
            return ERROR;
    }
    
    /* Set polarity */
    
    if (mode & STM32_TIM_CH_POLARITY_NEG)
        ccer_val |= ATIM_CCER_CC1P << (channel << 2);
    
    /* define its position (shift) and get register offset */
    
    if (channel & 1) ccmr_val <<= 8;
    if (channel > 1) ccmr_offset = STM32_GTIM_CCMR2_OFFSET;    

    stm32_tim_putreg(dev, ccmr_offset, ccmr_val);
    stm32_tim_putreg(dev, STM32_GTIM_CCER_OFFSET, ccer_val);
    
    /* set GPIO */
    
    switch( ((struct stm32_tim_priv_s *)dev)->base ) {
#if CONFIG_STM32_TIM2
        case STM32_TIM2_BASE:
            switch(channel) {
#if defined(GPIO_TIM2_CH1OUT)
                case 0: stm32_tim_gpioconfig(GPIO_TIM2_CH1OUT, mode); break;
#endif
#if defined(GPIO_TIM2_CH2OUT)
                case 1: stm32_tim_gpioconfig(GPIO_TIM2_CH2OUT, mode); break;
#endif
#if defined(GPIO_TIM2_CH3OUT)
                case 2: stm32_tim_gpioconfig(GPIO_TIM2_CH3OUT, mode); break;
#endif
#if defined(GPIO_TIM2_CH4OUT)
                case 3: stm32_tim_gpioconfig(GPIO_TIM2_CH4OUT, mode); break;
#endif
                default: return ERROR;
            }
            break;
#endif
#if CONFIG_STM32_TIM3
        case STM32_TIM3_BASE:
            switch(channel) {
#if defined(GPIO_TIM3_CH1OUT)
                case 0: stm32_tim_gpioconfig(GPIO_TIM3_CH1OUT, mode); break;
#endif
#if defined(GPIO_TIM3_CH2OUT)
                case 1: stm32_tim_gpioconfig(GPIO_TIM3_CH2OUT, mode); break;
#endif
#if defined(GPIO_TIM3_CH3OUT)
                case 2: stm32_tim_gpioconfig(GPIO_TIM3_CH3OUT, mode); break;
#endif
#if defined(GPIO_TIM3_CH4OUT)
                case 3: stm32_tim_gpioconfig(GPIO_TIM3_CH4OUT, mode); break;
#endif
                default: return ERROR;
            }
            break;
#endif
#if CONFIG_STM32_TIM4
        case STM32_TIM4_BASE:
            switch(channel) {
#if defined(GPIO_TIM4_CH1OUT)
                case 0: stm32_tim_gpioconfig(GPIO_TIM4_CH1OUT, mode); break;
#endif
#if defined(GPIO_TIM4_CH2OUT)
                case 1: stm32_tim_gpioconfig(GPIO_TIM4_CH2OUT, mode); break;
#endif
#if defined(GPIO_TIM4_CH3OUT)
                case 2: stm32_tim_gpioconfig(GPIO_TIM4_CH3OUT, mode); break;
#endif
#if defined(GPIO_TIM4_CH4OUT)
                case 3: stm32_tim_gpioconfig(GPIO_TIM4_CH4OUT, mode); break;
#endif
                default: return ERROR;
            }
            break;
#endif
#if CONFIG_STM32_TIM5
        case STM32_TIM5_BASE:
            switch(channel) {
#if defined(GPIO_TIM5_CH1OUT)
                case 0: stm32_tim_gpioconfig(GPIO_TIM5_CH1OUT, mode); break;
#endif
#if defined(GPIO_TIM5_CH2OUT)
                case 1: stm32_tim_gpioconfig(GPIO_TIM5_CH2OUT, mode); break;
#endif
#if defined(GPIO_TIM5_CH3OUT)
                case 2: stm32_tim_gpioconfig(GPIO_TIM5_CH3OUT, mode); break;
#endif
#if defined(GPIO_TIM5_CH4OUT)
                case 3: stm32_tim_gpioconfig(GPIO_TIM5_CH4OUT, mode); break;
#endif
                default: return ERROR;
            }
            break;
#endif

#if STM32_NATIM > 0
#if CONFIG_STM32_TIM1
        case STM32_TIM1_BASE:
            switch(channel) {
#if defined(GPIO_TIM1_CH1OUT)
                case 0: stm32_tim_gpioconfig(GPIO_TIM1_CH1OUT, mode); break;
#endif
#if defined(GPIO_TIM1_CH2OUT)
                case 1: stm32_tim_gpioconfig(GPIO_TIM1_CH2OUT, mode); break;
#endif
#if defined(GPIO_TIM1_CH3OUT)
                case 2: stm32_tim_gpioconfig(GPIO_TIM1_CH3OUT, mode); break;
#endif
#if defined(GPIO_TIM1_CH4OUT)
                case 3: stm32_tim_gpioconfig(GPIO_TIM1_CH4OUT, mode); break;
#endif
                default: return ERROR;
            }
            break;
#endif
#if CONFIG_STM32_TIM8
        case STM32_TIM8_BASE:
            switch(channel) {
#if defined(GPIO_TIM8_CH1OUT)
                case 0: stm32_tim_gpioconfig(GPIO_TIM8_CH1OUT, mode); break;
#endif
#if defined(GPIO_TIM8_CH2OUT)
                case 1: stm32_tim_gpioconfig(GPIO_TIM8_CH2OUT, mode); break;
#endif
#if defined(GPIO_TIM8_CH3OUT)
                case 2: stm32_tim_gpioconfig(GPIO_TIM8_CH3OUT, mode); break;
#endif
#if defined(GPIO_TIM8_CH4OUT)
                case 3: stm32_tim_gpioconfig(GPIO_TIM8_CH4OUT, mode); break;
#endif
                default: return ERROR;
            }
            break;
#endif
#endif
        default: return ERROR;
    }
    
    return OK;
}

static int stm32_tim_setcompare(FAR struct stm32_tim_dev_s *dev, uint8_t channel, uint16_t compare)
{
    ASSERT(dev);
    
    switch(channel) {
        case 1: stm32_tim_putreg(dev, STM32_GTIM_CCR1_OFFSET, compare); break;
        case 2: stm32_tim_putreg(dev, STM32_GTIM_CCR2_OFFSET, compare); break;
        case 3: stm32_tim_putreg(dev, STM32_GTIM_CCR3_OFFSET, compare); break;
        case 4: stm32_tim_putreg(dev, STM32_GTIM_CCR4_OFFSET, compare); break;
        default: return ERROR;
    }
    return OK;
}

static int stm32_tim_getcapture(FAR struct stm32_tim_dev_s *dev, uint8_t channel)
{
    ASSERT(dev);
    
    switch(channel) {
        case 1: return stm32_tim_getreg(dev, STM32_GTIM_CCR1_OFFSET);
        case 2: return stm32_tim_getreg(dev, STM32_GTIM_CCR2_OFFSET);
        case 3: return stm32_tim_getreg(dev, STM32_GTIM_CCR3_OFFSET);
        case 4: return stm32_tim_getreg(dev, STM32_GTIM_CCR4_OFFSET);
    }
    return ERROR;
}

/************************************************************************************
 * Advanced Functions
 ************************************************************************************/

/** \todo Advanced functions for the STM32_ATIM */
	

/************************************************************************************
 * Device Structures, Instantiation
 ************************************************************************************/

struct stm32_tim_ops_s stm32_tim_ops = {
    .setmode        = &stm32_tim_setmode,
    .setclock       = &stm32_tim_setclock,
    .setperiod      = &stm32_tim_setperiod,
    .setchannel     = &stm32_tim_setchannel,
    .setcompare     = &stm32_tim_setcompare,
    .getcapture     = &stm32_tim_getcapture,
    .setisr         = &stm32_tim_setisr,
    .enableint      = &stm32_tim_enableint,
    .disableint     = &stm32_tim_disableint,
    .ackint         = &stm32_tim_ackint
};

#if CONFIG_STM32_TIM2
struct stm32_tim_priv_s stm32_tim2_priv = {
    .ops        = &stm32_tim_ops,
    .mode       = STM32_TIM_MODE_UNUSED,
    .base       = STM32_TIM2_BASE,
};
#endif

#if CONFIG_STM32_TIM3
struct stm32_tim_priv_s stm32_tim3_priv = {
    .ops        = &stm32_tim_ops,
    .mode       = STM32_TIM_MODE_UNUSED,
    .base       = STM32_TIM3_BASE,
};
#endif

#if CONFIG_STM32_TIM4
struct stm32_tim_priv_s stm32_tim4_priv = {
    .ops        = &stm32_tim_ops,
    .mode       = STM32_TIM_MODE_UNUSED,
    .base       = STM32_TIM4_BASE,
};
#endif

#if CONFIG_STM32_TIM5
struct stm32_tim_priv_s stm32_tim5_priv = {
    .ops        = &stm32_tim_ops,
    .mode       = STM32_TIM_MODE_UNUSED,
    .base       = STM32_TIM5_BASE,
};
#endif

#if STM32_NBTIM > 0
#if CONFIG_STM32_TIM6
struct stm32_tim_priv_s stm32_tim6_priv = {
    .ops        = &stm32_tim_ops,
    .mode       = STM32_TIM_MODE_UNUSED,
    .base       = STM32_TIM6_BASE,
};
#endif
#endif

#if STM32_NBTIM > 1
#if CONFIG_STM32_TIM7
struct stm32_tim_priv_s stm32_tim7_priv = {
    .ops        = &stm32_tim_ops,
    .mode       = STM32_TIM_MODE_UNUSED,
    .base       = STM32_TIM7_BASE,
};
#endif
#endif

#if STM32_NATIM > 0

#if CONFIG_STM32_TIM1
struct stm32_tim_priv_s stm32_tim1_priv = {
    .ops        = &stm32_tim_ops,
    .mode       = STM32_TIM_MODE_UNUSED,
    .base       = STM32_TIM1_BASE,
};
#endif

#if CONFIG_STM32_TIM8
struct stm32_tim_priv_s stm32_tim8_priv = {
    .ops        = &stm32_tim_ops,
    .mode       = STM32_TIM_MODE_UNUSED,
    .base       = STM32_TIM8_BASE,
};
#endif

#endif


/************************************************************************************
 * Public Function - Initialization
 ************************************************************************************/

FAR struct stm32_tim_dev_s * stm32_tim_init(int timer)
{
    struct stm32_tim_dev_s * dev = NULL;
        
    /* Get structure and enable power */
    
    switch(timer) {
#if CONFIG_STM32_TIM2
        case 2: 
            dev = (struct stm32_tim_dev_s *)&stm32_tim2_priv; 
            modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM2EN);
            break;
#endif
#if CONFIG_STM32_TIM3
        case 3: 
            dev = (struct stm32_tim_dev_s *)&stm32_tim3_priv; 
            modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM3EN);
            break;
#endif
#if CONFIG_STM32_TIM4
        case 4: 
            dev = (struct stm32_tim_dev_s *)&stm32_tim4_priv; 
            modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM4EN);
            break;
#endif
#if CONFIG_STM32_TIM5
        case 5: 
            dev = (struct stm32_tim_dev_s *)&stm32_tim5_priv; 
            modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM5EN);
            break;
#endif

#if STM32_NBTIM > 0
#if CONFIG_STM32_TIM6
        case 6: 
            dev = (struct stm32_tim_dev_s *)&stm32_tim6_priv; 
            modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM6EN);
            break;
#endif
#endif
#if STM32_NBTIM > 1
#if CONFIG_STM32_TIM7
        case 7: 
            dev = (struct stm32_tim_dev_s *)&stm32_tim7_priv; 
            modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM7EN);
            break;
#endif
#endif

#if STM32_NATIM > 0
#if CONFIG_STM32_TIM1
        case 1: 
            dev = (struct stm32_tim_dev_s *)&stm32_tim1_priv; 
            modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM1EN);
            break;
#endif
#if CONFIG_STM32_TIM8
        case 8: 
            dev = (struct stm32_tim_dev_s *)&stm32_tim8_priv; 
            modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM8EN);
            break;
#endif
#endif
        default: return NULL;
    }
    
    /* Is device already allocated */
    
    if ( ((struct stm32_tim_priv_s *)dev)->mode != STM32_TIM_MODE_UNUSED)
        return NULL;
        
    stm32_tim_reset(dev);
        
    return dev;
}


/** \todo Detach interrupts, and close down all TIM Channels */
int stm32_tim_deinit(FAR struct stm32_tim_dev_s * dev)
{
    ASSERT(dev);
    
    /* Disable power */
    
    switch( ((struct stm32_tim_priv_s *)dev)->base ) {
#if CONFIG_STM32_TIM2
        case STM32_TIM2_BASE: modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM2EN, 0);  break;
#endif
#if CONFIG_STM32_TIM3
        case STM32_TIM3_BASE: modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM3EN, 0);  break;
#endif
#if CONFIG_STM32_TIM4
        case STM32_TIM4_BASE: modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM4EN, 0);  break;
#endif
#if CONFIG_STM32_TIM5
        case STM32_TIM5_BASE: modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM5EN, 0);  break;
#endif
#if STM32_NBTIM > 0
#if CONFIG_STM32_TIM6
        case STM32_TIM6_BASE: modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM6EN, 0);  break;
#endif
#endif
#if STM32_NBTIM > 1
#if CONFIG_STM32_TIM7
        case STM32_TIM7_BASE: modifyreg32(STM32_RCC_APB1ENR, RCC_APB1ENR_TIM7EN, 0);  break;
#endif
#endif
        
#if STM32_NATIM > 0
#if CONFIG_STM32_TIM1
        case STM32_TIM1_BASE: modifyreg32(STM32_RCC_APB2ENR, RCC_APB2ENR_TIM1EN, 0);  break;
#endif
#if CONFIG_STM32_TIM8
        case STM32_TIM8_BASE: modifyreg32(STM32_RCC_APB2ENR, RCC_APB2ENR_TIM8EN, 0);  break;
#endif
#endif
        default: return ERROR;
    }
    
    /* Mark it as free */
    
    ((struct stm32_tim_priv_s *)dev)->mode = STM32_TIM_MODE_UNUSED;
        
    return OK;
}

#endif /* defined(CONFIG_STM32_TIM1 || ... || TIM8) */
