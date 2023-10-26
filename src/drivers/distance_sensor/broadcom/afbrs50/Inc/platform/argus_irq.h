/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     This file provides an interface for enabling/disabling interrupts.
 *
 * @copyright
 *
 * Copyright (c) 2023, Broadcom Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef ARGUS_IRQ_H
#define ARGUS_IRQ_H
#ifdef __cplusplus
extern "C" {
#endif

/*!***************************************************************************
 * @defgroup    argus_irq IRQ: Global Interrupt Control Layer
 * @ingroup     argus_hal
 *
 * @brief       Global Interrupt Control Layer
 *
 * @details     This module provides functionality to globally enable/disable
 *              interrupts in a nested way.
 *
 *              Here is a simple example implementation using the CMSIS functions
 *              "__enable_irq()" and "__disable_irq()". An integer counter is
 *              used to achieve nested interrupt disabling:
 *
 *              @code
 *
 *              // Global lock level counter value.
 *              static volatile int g_irq_lock_ct;
 *
 *              // Global unlock all interrupts using CMSIS function "__enable_irq()".
 *              void IRQ_UNLOCK(void)
 *              {
 *                  assert(g_irq_lock_ct > 0);
 *                  if (--g_irq_lock_ct <= 0)
 *                  {
 *                      g_irq_lock_ct = 0;
 *                      __enable_irq();
 *                  }
 *              }
 *
 *              // Global lock all interrupts using CMSIS function "__disable_irq()".
 *              void IRQ_LOCK(void)
 *              {
 *                  __disable_irq();
 *                  g_irq_lock_ct++;
 *              }
 *
 *              @endcode
 *
 * @note        The IRQ locking mechanism is used to create atomic sections
 *              (within the scope of the AFBR-S50 API) that are very few processor
 *              instruction only. It does NOT lock interrupts for considerable
 *              amounts of time.
 *
 * @note        The IRQ_LOCK might get called multiple times. Therefore, the
 *              API expects that the IRQ_UNLOCK must be called as many times as
 *              the IRQ_LOCK was called before the interrupts are enabled.
 *
 * @note        The interrupts utilized by the AFBR-S50 API can be interrupted
 *              by other, higher prioritized interrupts, e.g. some system
 *              critical interrupts. In this case, the IRQ_LOCK/IRQ_UNLOCK
 *              mechanism can be implemented such that only the interrupts
 *              required for the AFBR-S50 API are locked. The above example is
 *              dedicated to a ARM Corex-M0 architecture, where interrupts
 *              can only disabled at a global scope. Other architectures like
 *              ARM Cortex-M4 allow selective disabling of interrupts.
 *
 * @addtogroup  argus_irq
 * @{
 *****************************************************************************/

/*!***************************************************************************
 * @brief   Enable IRQ Interrupts
 *
 * @details Enables IRQ interrupts and enters an atomic or critical section.
 *
 * @note    The IRQ_LOCK might get called multiple times. Therefore, the
 *          API expects that the IRQ_UNLOCK must be called as many times as
 *          the IRQ_LOCK was called before the interrupts are enabled.
 *****************************************************************************/
void IRQ_UNLOCK(void);

/*!***************************************************************************
 * @brief   Disable IRQ Interrupts
 *
 * @details Disables IRQ interrupts and leaves the atomic or critical section.
 *
 * @note    The IRQ_LOCK might get called multiple times. Therefore, the
 *          API expects that the IRQ_UNLOCK must be called as many times as
 *          the IRQ_LOCK was called before the interrupts are enabled.
 *****************************************************************************/
void IRQ_LOCK(void);

/*! @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // ARGUS_IRQ_H
