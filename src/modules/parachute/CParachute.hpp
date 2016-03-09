/*
 * CParachute.hpp
 *
 *  Created on: Jun 20, 2015
 *      Author: friedrich
 */

#ifndef CPARACHUTE_HPP_
#define CPARACHUTE_HPP_

/*
 * include system configuration
 */
#include <nuttx/config.h>

/*
 * basic includes
 */
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <mathlib/mathlib.h>

#include <poll.h>
#include <sys/types.h>
#include <assert.h>
#include <nuttx/analog/adc.h>
#include <assert.h>
#include <sched.h>

/*
 * include drivers
 */
#include <drivers/drv_hrt.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_airspeed.h>
#include <drivers/drv_px4flow.h>

/*
 * include systemlib
 */
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/airspeed.h>

#include <conversion/rotation.h>


#define PARACHUTE_TASK_DELAY_MS 150

#include "CStateMachine.hpp"

/*
 * define OK to handle compiler errors
 */
#ifndef OK
#define OK 0
#endif


/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;



class CParachute
{
public:
	/**
	 * Constructor
	 */
	CParachute();

	/**
	 * Destructor, also kills the parachute task.
	 */
	~CParachute();

	/**
	 * Start the parachute task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	/**
	 * getState of state machine
	 *
	 * @return		[0-3] as state id
	 */
	inline int	getState()
	{
		return m_cStateMachine->getState();
	}


private:
	bool	m_task_should_exit;			/**< if true, parachute task should exit */
	int 	m_parachute_task;			/**< task handle for parachute task */
	bool	m_hil_enabled;				/**< if true, HIL is active */

	perf_counter_t	m_loop_perf;			/**< loop performance counter */

	CStateMachine* 	m_cStateMachine;


	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main parachute task.
	 */
	void		task_main();
};


#endif /* CPARACHUTE_HPP_ */
