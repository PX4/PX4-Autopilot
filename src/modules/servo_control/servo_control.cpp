/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 ****************************************************************************/

/**
 * @file servo_control.cpp
 *
 * @author bdai<bdai1412@gmail.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/task_status_monitor_m2p.h>
#include <uORB/topics/actuator_controls.h>

extern "C" __EXPORT int servo_control_main(int argc, char *argv[]);

class ServoControl;

namespace servo_control
{
	ServoControl *instance;
}

class ServoControl
{
public:
	/**
	 * Constructor
	 */
	ServoControl();

	/**
	 * Destructor, also kills task.
	 */
	~ServoControl();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	static void	task_main_trampoline(int argc, char *argv[]);

	void		task_main();

private:
	bool 	_task_should_exit;	/**< if true, task_main() should exit */
	int		_control_task;	/**< task handle */

	int 	_servo_sub;
	orb_advert_t _actuator_2_pub;
	struct {
		float duration;
		bool tretch;
		bool isspraying;
		bool isready;
	} _servo;
	int _pre_status;

};

ServoControl::ServoControl():
		_task_should_exit(false),
		_control_task(-1),
		_servo_sub(-1),
		_actuator_2_pub(nullptr),
		_pre_status(0)
{
	memset(&_servo, 0, sizeof(_servo));
}

ServoControl::~ServoControl()
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
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	servo_control::instance = nullptr;
}

int
ServoControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("servo_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_DEFAULT - 5,
					   1500,
					   (px4_main_t)&ServoControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void
ServoControl::task_main_trampoline(int argc, char *argv[])
{
	servo_control::instance->task_main();
}

void
ServoControl::task_main()
{
	int task_status_sub = orb_subscribe(ORB_ID(task_status_monitor_m2p));
	struct pollfd fds[1];
	fds[0].fd = task_status_sub;
	fds[0].events = POLLIN;

	struct task_status_monitor_m2p_s task_status;
	struct actuator_controls_s actuators_2;
	hrt_abstime spray_finish_time = 0;

	/*initialized -bdai<23 Sep 2016>*/
	actuators_2.control[4] = 1;
	actuators_2.control[5] = 0.20;
	_actuator_2_pub = orb_advertise(ORB_ID(actuator_controls_2), &actuators_2);

	while (!_task_should_exit){
		int ret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 200);

		/* timed out - periodic check for _task_should_exit */
		if (ret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (ret < 0) {
			warn("poll error %d, %d", ret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		orb_copy(ORB_ID(task_status_monitor_m2p), task_status_sub, &task_status);
		_servo.duration = task_status.spray_duration;

		if (task_status.task_status == 12) {
			_servo.tretch = true;
			_servo.isready = false;
			spray_finish_time = 0;
		} else if (task_status.task_status == 13) {
			_servo.tretch = true;
			_servo.isready = true;
			if (_pre_status != 13) {
				spray_finish_time = (hrt_abstime)_servo.duration*1e6 + hrt_absolute_time();
			}
		} else {
			_servo.tretch = false;
			_servo.isready = false;
			spray_finish_time = 0;
		}

		_pre_status = task_status.task_status;

		if (_servo.tretch) {
			actuators_2.control[4] = -1; /*tretch out -bdai<23 Sep 2016>*/
		} else {
			actuators_2.control[4] = 1;
		}

		if (_servo.isready && hrt_absolute_time() < spray_finish_time){
			actuators_2.control[5] = 0.55;	/*start injection -bdai<23 Sep 2016>*/
		} else {
			actuators_2.control[5] = 0.2;	/*stop injection -bdai<23 Sep 2016>*/
		}
		orb_publish(ORB_ID(actuator_controls_2), _actuator_2_pub, &actuators_2);

	}
}


int
servo_control_main(int argc, char *argv[])
{
	if(argc < 2){
		warnx("usage: servo_control {start|stop|status}");
		return 1;
	}

	if(!strcmp(argv[1],"start")){
		if(servo_control::instance !=nullptr){
			warnx("already running");
			return 1;
		}

		servo_control::instance = new ServoControl;

		if(servo_control::instance == nullptr){
			warnx("alloc failed");
			return 1;
		}

		if(OK != servo_control::instance->start()){
			delete servo_control::instance;
			servo_control::instance = nullptr;
			warn("start failed");
			return 1;
		}

		return 0;
	}

	if(!strcmp(argv[1], "stop")){
		if(servo_control::instance == nullptr){
			warnx("not running");
			return 1;
		}

		delete servo_control::instance;
		servo_control::instance = nullptr;
		return 0;
	}

	if(!strcmp(argv[1],"status")){
		if(servo_control::instance){
			warnx("running");
			return 0;
		}else{
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}














