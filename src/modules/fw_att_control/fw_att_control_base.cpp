/*
 * fw_att_control_base.cpp
 *
 *  Created on: Sep 24, 2014
 *      Author: roman
 */

#include "fw_att_control_base.h"


FixedwingAttitudeControlBase::FixedwingAttitudeControlBase() :

	_task_should_exit(false),
	_task_running(false),
	_control_task(-1),


/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fw att control")),
	_nonfinite_input_perf(perf_alloc(PC_COUNT, "fw att control nonfinite input")),
	_nonfinite_output_perf(perf_alloc(PC_COUNT, "fw att control nonfinite output")),
/* states */
	_setpoint_valid(false),
	_debug(false)
{
	/* safely initialize structs */
	_att = {};
	_accel = {};
	_att_sp = {};
	_manual = {};
	_airspeed = {};
	_vcontrol_mode = {};
	_actuators = {};
	_actuators_airframe = {};
	_global_pos = {};
	_vehicle_status = {};

}



FixedwingAttitudeControlBase::~FixedwingAttitudeControlBase()
{

}

void FixedwingAttitudeControlBase::control_attitude()
{

}
