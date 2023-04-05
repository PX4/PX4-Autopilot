/*************************************************************************//**
 * @file
 * @brief       This file is part of the AFBR-S50 API.
 * @details     This file provides an interface for the required timer modules.
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

#ifndef ARGUS_TIMER_H
#define ARGUS_TIMER_H
#ifdef __cplusplus
extern "C" {
#endif

/*!***************************************************************************
 * @defgroup    argus_timer Timer: Hardware Timer Interface
 * @ingroup     argus_hal
 *
 * @brief       Timer implementations for lifetime counting as well as periodic
 *              callback.
 *
 * @details     The module provides an interface to the timing utilities that
 *              are required by the AFBR-S50 time-of-flight sensor API.
 *
 *              Two essential features have to be provided by the user code:
 *              1. Time Measurement Capability: In order to keep track of outgoing
 *                signals, the API needs to measure elapsed time. In order to
 *                provide optimum device performance, the granularity should be
 *                around 10 to 100 microseconds.
 *              2. Periodic Callback: The API provides an automatic starting of
 *                measurement cycles at a fixed frame rate via a periodic
 *                interrupt timer. If this feature is not used, implementation
 *                of the periodic interrupts can be skipped. An weak default
 *                implementation is provide in the API.
 *              .
 *
 *              The time measurement feature is simply implemented by the function
 *              #Timer_GetCounterValue. Whenever the function is called, the
 *              provided counter values must be written with the values obtained
 *              by the current time.
 *
 *              The periodic interrupt timer is a simple callback interface.
 *              After installing the callback function pointer via #Timer_SetCallback,
 *              the timer can be started by setting interval via #Timer_SetInterval.
 *              From then, the callback is invoked periodically as the corresponding
 *              interval may specify. The timer is stopped by setting the interval
 *              to 0 using the #Timer_SetInterval function. The interval can be
 *              updated at any time by updating the interval via the #Timer_SetInterval
 *              function. To any of these functions, an abstract parameter pointer
 *              must be passed. This parameter is passed back to the callback any
 *              time it is invoked.
 *
 *              In order to provide the usage of multiple devices, an mechanism is
 *              introduced to allow the installation of multiple callback interval
 *              at the same time. Therefore, the abstract parameter pointer is used
 *              to identify the corresponding callback interval. For example, there
 *              are two callbacks for two intervals, t1 and t2, required. The user
 *              can start two timers by calling the #Timer_SetInterval method twice,
 *              but with an individual parameter pointer, ptr1 and ptr2, each:
 *              \code
 *                  Timer_SetInterval(100000, ptr1); // 10 ms callback w/ parameter ptr1
 *                  Timer_SetInterval(200000, ptr2); // 20 ms callback w/ parameter ptr1
 *              \endcode
 *
 *              Note that the implemented timer module must therefore support
 *              as many different intervals as instances of the AFBR-S50 device are
 *              used.
 *
 * @addtogroup  argus_timer
 * @{
 *****************************************************************************/

#include "utility/status.h"

/*******************************************************************************
 * Lifetime Counter Timer Interface
 ******************************************************************************/

/*!***************************************************************************
 * @brief   Obtains the lifetime counter value from the timers.
 *
 * @details The function is required to get the current time relative to any
 *          point in time, e.g. the startup time. The returned values \p hct and
 *          \p lct are given in seconds and microseconds respectively. The current
 *          elapsed time since the reference time is then calculated from:
 *
 *          t_now [µsec] = hct * 1000000 µsec + lct * 1 µsec
 *
 *          Note that the accuracy/granularity of the lifetime counter does
 *          not need to be 1 µsec. Usually, a granularity of approximately
 *          100 µsec is sufficient. However, in case of very high frame rates
 *          (above 1000 frames per second), it is recommended to implement
 *          an even lower granularity (somewhere in the 10 µsec regime).
 *
 *          It must be guaranteed, that each call of the #Timer_GetCounterValue
 *          function must provide a value that is greater or equal, but never lower,
 *          than the value returned from the previous call.
 *
 *          A hardware based implementation of the lifetime counter functionality
 *          would be to chain two distinct timers such that counter 2 increases
 *          its value when counter 1 wraps to 0. The easiest way is to setup
 *          counter 1 to wrap exactly every second. Counter 1 would than count
 *          the sub-seconds (i.e. µsec) value (\p lct) and counter 2 the seconds
 *          (\p hct) value. A 16-bit counter is sufficient in case of counter 1
 *          while counter 2 must be a 32-bit version.
 *
 *          In case of a lack of available hardware timers, a software solution
 *          can be used that requires only a 16-bit timer. In a simple scenario,
 *          the timer is configured to wrap around every second and increase
 *          a software counter value in its interrupt service routine (triggered
 *          with the wrap around event) every time the wrap around occurs.
 *
 *
 * @note    The implementation of this function is mandatory for the correct
 *          execution of the API.
 *
 * @param   hct A pointer to the high counter value bits representing current
 *              time in seconds.
 *
 * @param   lct A pointer to the low counter value bits representing current
 *              time in microseconds. Range: 0, .., 999999 µsec
 *****************************************************************************/
void Timer_GetCounterValue(uint32_t *hct, uint32_t *lct);

/*******************************************************************************
 * Periodic Interrupt Timer Interface
 ******************************************************************************/

/*!***************************************************************************
 * @brief   The callback function type for periodic interrupt timer.
 *
 * @details The function that is invoked every time a specified interval elapses.
 *          An abstract parameter is passed to the function whenever it is called.
 *
 * @param   param An abstract parameter to be passed to the callback. This is
 *                  also the identifier of the given interval.
 *****************************************************************************/
typedef void (*timer_cb_t)(void *param);

/*!***************************************************************************
 * @brief   Installs an periodic timer callback function.
 *
 * @details Installs an periodic timer callback function that is invoked whenever
 *          an interval elapses. The callback is the same for any interval,
 *          however, the single intervals can be identified by the passed
 *          parameter.
 *          Passing a zero-pointer removes and disables the callback.
 *
 * @note    The implementation of this function is optional for the correct
 *          execution of the API. If not implemented, a weak implementation
 *          within the API will be used that disable the periodic timer callback
 *          and thus the automatic starting of measurements from the background.
 *
 * @param   f The timer callback function.
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Timer_SetCallback(timer_cb_t f);

/*!***************************************************************************
 * @brief   Sets the timer interval for a specified callback parameter.
 *
 * @details Sets the callback interval for the specified parameter and starts
 *          the timer with a new interval. If there is already an interval with
 *          the given parameter, the timer is restarted with the given interval.
 *          If the same time interval as already set is passed, nothing happens.
 *          Passing a interval of 0 disables the timer.
 *
 *          When enabling the timer (or resetting by applying another interval),
 *          the first timer interrupt must happen after the specified interval.
 *
 *          Note that a microsecond granularity for the timer interrupt period is
 *          not required. Usually a milliseconds granularity is sufficient.
 *          The required granularity depends on the targeted frame rate, e.g. in
 *          case of more than 1 kHz measurement rate, a granularity of less than
 *          a millisecond is required to achieve the given frame rate.
 *
 * @note    The implementation of this function is optional for the correct
 *          execution of the API. If not implemented, a weak implementation
 *          within the API will be used that disable the periodic timer callback
 *          and thus the automatic starting of measurements from the background.
 *
 * @param   dt_microseconds The callback interval in microseconds.
 *
 * @param   param An abstract parameter to be passed to the callback. This is
 *                  also the identifier of the given interval.
 *
 * @return  Returns the \link #status_t status\endlink (#STATUS_OK on success).
 *****************************************************************************/
status_t Timer_SetInterval(uint32_t dt_microseconds, void *param);

/*! @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif /* ARGUS_TIMER_H */
