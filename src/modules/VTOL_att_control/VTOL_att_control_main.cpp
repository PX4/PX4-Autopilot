/*
 * VTOL_att_control.cpp
 *
 *  Created on: Aug 29, 2014
 *      Author: roman
 */
//@author: Roman Bapst <bapstr@ethz.ch>



//take over headers from mc_att_controller for now but check if really all are needed later
#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>


extern "C" __EXPORT int VTOL_att_control_main(int argc, char *argv[]);



class VtolAttitudeControl
{
public:

	VtolAttitudeControl();
	~VtolAttitudeControl();

	int start();	//start the task and return OK on success


private:
//-----------------flags & handles----------------------------------------------
	bool _task_should_exit;
	int _control_task;		//task handle for VTOL attitude controller

	//handlers for subscriptions
	int		_v_att_sub;				//vehicle attitude subscription
	int		_v_att_sp_sub;			//vehicle attitude setpoint subscription
	int		_v_rates_sp_sub;		//vehicle rates setpoint subscription
	int		_v_control_mode_sub;	//vehicle control mode subscription
	int		_params_sub;			//parameter updates subscription
	int		_manual_control_sp_sub;	//manual control setpoint subscription
	int		_armed_sub;				//arming status subscription

	int 	_actuator_inputs_mc;	//topic on which the mc_att_controller publishes actuator inputs
	int 	_actuator_inputs_fw;	//topic on which the fw_att_controller publishes actuator inputs

	//handlers for publishers
	orb_advert_t	_actuators_0_pub;		//input for the mixer (roll,pitch,yaw,thrust)
	orb_advert_t 	_actuators_1_pub;
//----------------data containers-------------------------------------------------------
	struct vehicle_attitude_s			_v_att;				//vehicle attitude
	struct vehicle_attitude_setpoint_s	_v_att_sp;			//vehicle attitude setpoint
	struct vehicle_rates_setpoint_s		_v_rates_sp;		//vehicle rates setpoint
	struct manual_control_setpoint_s	_manual_control_sp; //manual control setpoint
	struct vehicle_control_mode_s		_v_control_mode;	//vehicle control mode
	struct actuator_controls_s			_actuators_out_0;	//actuator controls going to the mc mixer
	struct actuator_controls_s			_actuators_out_1;	//actuator controls going to the fw mixer (used for elevons)
	struct actuator_controls_s			_actuators_mc_in;	//actuator controls from mc_att_control
	struct actuator_controls_s			_actuators_fw_in;	//actuator controls from fw_att_control
	struct actuator_armed_s				_armed;				//actuator arming status

	perf_counter_t	_loop_perf;			/**< loop performance counter */


//----------------Member functions---------------------------------------------------------

	void 		task_main();	//main task
	static void	task_main_trampoline(int argc, char *argv[]);	//Shim for calling task_main from task_create.

	void		vehicle_control_mode_poll();	//Check for changes in vehicle control mode.
	void		vehicle_manual_poll();			//Check for changes in manual inputs.
	void		arming_status_poll();			//Check for arming status updates.
	void 		actuator_controls_mc_poll();	//Check for changes in mc_attitude_control output
	void 		actuator_controls_fw_poll();	//Check for changes in fw_attitude_control output
	void  		fill_mc_att_control_output();	//write mc_att_control results to actuator message
	void		fill_fw_att_control_output();	//write fw_att_control results to actuator message


};



namespace VTOL_att_control {

VtolAttitudeControl *g_control;

}

VtolAttitudeControl::VtolAttitudeControl() :
	_task_should_exit(false),
	_control_task(-1),

	//init subscription handlers
	_v_att_sub(-1),
	_v_att_sp_sub(-1),
	_v_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sp_sub(-1),
	_armed_sub(-1),

	//init publication handlers
	_actuators_0_pub(-1),
	_actuators_1_pub(-1),

	_loop_perf(perf_alloc(PC_ELAPSED, "mc_att_control"))
{

}

VtolAttitudeControl::~VtolAttitudeControl()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	VTOL_att_control::g_control = nullptr;
}


/**
* Check for changes in vehicle control mode.
*/
void VtolAttitudeControl::vehicle_control_mode_poll()
{
	bool updated;

	/* Check HIL state if vehicle status has changed */
	orb_check(_v_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _v_control_mode_sub, &_v_control_mode);
	}
}

/**
* Check for changes in manual inputs.
*/
void VtolAttitudeControl::vehicle_manual_poll()
{
	bool updated;

	/* get pilots inputs */
	orb_check(_manual_control_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sp_sub, &_manual_control_sp);
	}
}
/**
* Check for arming status updates.
*/
void VtolAttitudeControl::arming_status_poll()
{
	/* check if there is a new setpoint */
	bool updated;
	orb_check(_armed_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
	}
}

void VtolAttitudeControl::actuator_controls_mc_poll()
{
	bool updated;
	orb_check(_actuator_inputs_mc, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_controls_virtual_mc),_actuator_inputs_mc , &_actuators_mc_in);
	}
}

void VtolAttitudeControl::actuator_controls_fw_poll()
{
	bool updated;
	orb_check(_actuator_inputs_fw, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_controls_virtual_fw),_actuator_inputs_fw , &_actuators_fw_in);
	}
}

void VtolAttitudeControl::fill_mc_att_control_output()
{
	_actuators_out_0.control[0] = _actuators_mc_in.control[0];
	_actuators_out_0.control[1] = _actuators_mc_in.control[1];
	_actuators_out_0.control[2] = _actuators_mc_in.control[2];
	_actuators_out_0.control[3] = _actuators_mc_in.control[3];
	//set neutral position for elevons
	_actuators_out_1.control[0] = 0;	//roll elevon
	_actuators_out_1.control[1] = 0;	//pitch elevon
}

void VtolAttitudeControl::fill_fw_att_control_output()
{

	_actuators_out_0.control[0] = _actuators_fw_in.control[2];	//fw roll is mc yaw
	//warnx("roll %.5f",(double)_actuators_out_0.control[0]);
	_actuators_out_0.control[1] = _actuators_fw_in.control[1];	//fw pitch is mc pitch
	_actuators_out_0.control[2] = _actuators_fw_in.control[0];	//fw yaw is mc roll
	_actuators_out_0.control[3] = _actuators_fw_in.control[3];	//throttle stays throttle
	//controls for the elevons
	_actuators_out_1.control[0] = _actuators_fw_in.control[0];	//roll elevon
	_actuators_out_1.control[1] = _actuators_fw_in.control[1];	//pitch elevon
}


void
VtolAttitudeControl::task_main_trampoline(int argc, char *argv[])
{
	VTOL_att_control::g_control->task_main();
}


void VtolAttitudeControl::task_main()
{
	warnx("started");
	fflush(stdout);
//
	//do subscriptions
	_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));

//	//check if these topics are declared
	_actuator_inputs_mc = orb_subscribe(ORB_ID(actuator_controls_virtual_mc));
	_actuator_inputs_fw = orb_subscribe(ORB_ID(actuator_controls_virtual_fw));
//
	/* wakeup source: vehicle attitude */
	struct pollfd fds[2];

	fds[0].fd = _actuator_inputs_mc;
	fds[0].events = POLLIN;
	fds[1].fd = _actuator_inputs_fw;
	fds[1].events = POLLIN;

	while(!_task_should_exit)
	{

//		/* wait for up to 100ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);
//
//		/* timed out - periodic check for _task_should_exit */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

//
// 	got data from mc_att_controller
		if (fds[0].revents & POLLIN) {
			vehicle_manual_poll();	//update remote input
			orb_copy(ORB_ID(actuator_controls_virtual_mc), _actuator_inputs_mc, &_actuators_mc_in);
			if(_manual_control_sp.aux1 <= 0.0f)
			{
				fill_mc_att_control_output();

				if (_actuators_0_pub > 0) {
					orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators_out_0);
				} else
				{
					_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators_out_0);
				}
				if (_actuators_1_pub > 0) {
					orb_publish(ORB_ID(actuator_controls_1), _actuators_1_pub, &_actuators_out_1);
				}
				else
				{
					_actuators_1_pub = orb_advertise(ORB_ID(actuator_controls_1), &_actuators_out_1);
				}
			}

		}
//	got data from fw_att_controller
		if(fds[1].revents & POLLIN)
		{
			orb_copy(ORB_ID(actuator_controls_virtual_fw), _actuator_inputs_fw, &_actuators_fw_in);
			vehicle_manual_poll();	//update remote input
			if(_manual_control_sp.aux1 >= 0.0f)
				{
					fill_fw_att_control_output();

					if (_actuators_0_pub > 0) {
						orb_publish(ORB_ID(actuator_controls_0), _actuators_0_pub, &_actuators_out_0);
					} else
					{
						_actuators_0_pub = orb_advertise(ORB_ID(actuator_controls_0), &_actuators_out_0);
					}
					if (_actuators_1_pub > 0) {
						orb_publish(ORB_ID(actuator_controls_1), _actuators_1_pub, &_actuators_out_1);
					} else
					{
						_actuators_1_pub = orb_advertise(ORB_ID(actuator_controls_1), &_actuators_out_1);
					}
				}
		}

	}
	warnx("exit");
	_control_task = -1;
	_exit(0);




	}

int
VtolAttitudeControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = task_spawn_cmd("VTOL_att_control",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2048,
				       (main_t)&VtolAttitudeControl::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}


int VTOL_att_control_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: VTOL_att_control {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (VTOL_att_control::g_control != nullptr)
			errx(1, "already running");

		VTOL_att_control::g_control = new VtolAttitudeControl;

		if (VTOL_att_control::g_control == nullptr)
			errx(1, "alloc failed");

		if (OK != VTOL_att_control::g_control->start()) {
			delete VTOL_att_control::g_control;
			VTOL_att_control::g_control = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (VTOL_att_control::g_control == nullptr)
			errx(1, "not running");

		delete VTOL_att_control::g_control;
		VTOL_att_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (VTOL_att_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
