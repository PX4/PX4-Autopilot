
#include "timer.h"

#include <drivers/drv_hrt.h>

#include <stdio.h>

#include <board_config.h>

static struct hrt_call broadcom_hrt_call = {};

static timer_cb_t timer_callback_; /*! Callback function for PIT timer */

static uint32_t period_us_;

/*! Storage for the callback parameter */
static void *callback_param_;

static void broadcom_hrt_callout(void *arg)
{
	if (timer_callback_ != 0) {
		//timer_callback_(arg);
		timer_callback_(callback_param_);
		hrt_call_after(&broadcom_hrt_call, period_us_, broadcom_hrt_callout, callback_param_);
	}
}

void Timer_Init(void)
{
	hrt_cancel(&broadcom_hrt_call);
}

/*!***************************************************************************
* @brief Obtains the lifetime counter value from the timers.
*
* @details The function is required to get the current time relative to any
* point in time, e.g. the startup time. The returned values \p hct and
* \p lct are given in seconds and microseconds respectively. The current
* elapsed time since the reference time is then calculated from:
*
* t_now [usec] = hct * 1000000 usec + lct * 1 usec
*
* @param hct A pointer to the high counter value bits representing current
* time in seconds.
* @param lct A pointer to the low counter value bits representing current
* time in microseconds. Range: 0, .., 999999 usec
* @return -
*****************************************************************************/

void Timer_GetCounterValue(uint32_t *hct, uint32_t *lct)
{
	hrt_abstime time = hrt_absolute_time();
	*hct = time / 1000000ULL;
	*lct = time - (*hct * 1000000ULL);
}

/*!***************************************************************************
* @brief Starts the timer for a specified callback parameter.
* @details Sets the callback interval for the specified parameter and starts
* the timer with a new interval. If there is already an interval with
* the given parameter, the timer is restarted with the given interval.
* Passing a interval of 0 disables the timer.
* @param dt_microseconds The callback interval in microseconds.
* @param param An abstract parameter to be passed to the callback. This is
* also the identifier of the given interval.
* @return Returns the \link #status_t status\endlink (#STATUS_OK on success).
*****************************************************************************/

status_t Timer_Start(uint32_t period, void *param)
{
	callback_param_ = param;
	period_us_ = period;

	if (period != 0) {
		hrt_call_after(&broadcom_hrt_call, period, broadcom_hrt_callout, param);

	} else {
		hrt_cancel(&broadcom_hrt_call);
	}

	return STATUS_OK;
}

/*!***************************************************************************
* @brief Stops the timer for a specified callback parameter.
* @details Stops a callback interval for the specified parameter.
* @param param An abstract parameter that identifies the interval to be stopped.
* @return Returns the \link #status_t status\endlink (#STATUS_OK on success).
*****************************************************************************/
status_t Timer_Stop(void *param)
{
	period_us_ = 0;
	callback_param_ = 0;
	hrt_cancel(&broadcom_hrt_call);
	return STATUS_OK;
}

/*!***************************************************************************
* @brief Sets the timer interval for a specified callback parameter.
* @details Sets the callback interval for the specified parameter and starts
* the timer with a new interval. If there is already an interval with
* the given parameter, the timer is restarted with the given interval.
* If the same time interval as already set is passed, nothing happens.
* Passing a interval of 0 disables the timer.
* @param dt_microseconds The callback interval in microseconds.
* @param param An abstract parameter to be passed to the callback. This is
* also the identifier of the given interval.
* @return Returns the \link #status_t status\endlink (#STATUS_OK on success).
*****************************************************************************/
status_t Timer_SetInterval(uint32_t dt_microseconds, void *param)
{
	if (dt_microseconds != 0) {
		period_us_ = dt_microseconds;
		hrt_call_after(&broadcom_hrt_call, dt_microseconds, broadcom_hrt_callout, param);

	} else {
		hrt_cancel(&broadcom_hrt_call);
	}

	return STATUS_OK;
}

/*!***************************************************************************
* @brief Installs an periodic timer callback function.
* @details Installs an periodic timer callback function that is invoked whenever
* an interval elapses. The callback is the same for any interval,
* however, the single intervals can be identified by the passed
* parameter.
* Passing a zero-pointer removes and disables the callback.
* @param f The timer callback function.
* @return Returns the \link #status_t status\endlink (#STATUS_OK on success).
*****************************************************************************/

status_t Timer_SetCallback(timer_cb_t f)
{
	timer_callback_ = f;
	return STATUS_OK;
}
