/****************************************************************************
*
* Copyright (c) 2015 Mark Charlebois. All rights reserved.
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

#include <stdint.h>

#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <errno.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/parameter_update.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/param/param.h>
#include <uart_esc.h>

/** driver 'main' command */
extern "C" { __EXPORT int uart_esc_main(int argc, char *argv[]); }

namespace uart_esc
{
	volatile bool _task_should_exit = false; // flag indicating if uart_esc task should exit
	static const char* _device = NULL;       // SPI device path that uart_esc is connected to
	static bool _is_running = false;         // flag indicating if uart_esc app is running
	static px4_task_t _task_handle = -1;     // handle to the task main thread
	static struct esc_feedback_s _feedback;  // esc feedback
	UartEsc* esc;                            // esc instance
	void uart_esc_rotate_motors(int* motor_rpm,int num_rotors); // motor re-mapping

	// subscriptions
	int		_controls_sub;
	int		_armed_sub;
	int		_param_sub;

	// publications
	orb_advert_t	_outputs_pub;

	// topic structures
	actuator_controls_s     _controls;
	actuator_armed_s	_armed;
	parameter_update_s      _param_update;
	actuator_outputs_s      _outputs;

	/** Print out the usage information */
	static void usage();

	/** uart_esc start */
	static void start(const char* device);

	/** uart_esc stop */
	static void stop();

	/** task main trampoline function */
	static void	task_main_trampoline(int argc, char *argv[]);

	/** uart_esc thread primary entry point */
	static void task_main(int argc, char *argv[]);

	/** mixer initialization */
	static MultirotorMixer* mixer;
	static int initialize_mixer(const char* mixer_filename);
	static int mixer_control_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input);
	unsigned int _num_mixer_outputs;

int mixer_control_callback(uintptr_t handle,
			 uint8_t control_group,
			 uint8_t control_index,
			 float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls[control_group].control[control_index];

	return 0;
}


int initialize_mixer(const char* mixer_filename)
{
	int mixer_initialized = -1;

	static const char *buf=
		"R: 4x 10000 10000 10000 0\n"
		"M: 1\n"
		"O: 10000 10000 0 -10000 10000\n"
		"S: 0 4 10000 10000 0 -10000 10000\n"
		"M: 1\n"
		"O: 10000 10000 0 -10000 10000\n"
		"S: 0 5 10000 10000 0 -10000 10000\n"
		"M: 1\n"
		"O: 10000 10000 0 -10000 10000\n"
		"S: 0 6 10000 10000 0 -10000 10000\n"
		"M: 1\n"
		"O: 10000 10000 0 -10000 10000\n"
		"S: 0 7 10000 10000 0 -10000 10000\n";
		/*"Multirotor mixer for PX4FMU\n"
		"===========================\n"
		"\n"
		"This file defines a single mixer for a quadrotor in the X configuration.  All controls\n"
		"are mixed 100%.\n"
		"\n"
		"R: 4x 10000 10000 10000 0\n"
		"\n"
		"Gimbal / payload mixer for last four channels\n"
		"-----------------------------------------------------\n"
		"\n"
		"M: 1\n"
		"O:      10000  10000      0 -10000  10000\n"
		"S: 0 4  10000  10000      0 -10000  10000\n"
		"\n"
		"M: 1\n"
		"O:      10000  10000      0 -10000  10000\n"
		"S: 0 5  10000  10000      0 -10000  10000\n"
		"\n"
		"M: 1\n"
		"O:      10000  10000      0 -10000  10000\n"
		"S: 0 6  10000  10000      0 -10000  10000\n"
		"\n"
		"M: 1\n"
		"O:      10000  10000      0 -10000  10000\n"
		"S: 0 7  10000  10000      0 -10000  10000\n\n"
		;*/
	unsigned int buflen = strlen(buf);
	_num_mixer_outputs  = 4;

	// This is the intended way to read the charater description of the mixer, and should be re-enabled
	// when file reading is supported
#if 0
	int fd_load = open(mixer_filename, O_RDONLY, 0);
	char buf[512];
	unsigned int buflen = sizeof(buf);

	if(fd_load != -1)
	{
		int nRead = read(fd_load, buf, buflen);
		if(nRead > 0)
		{
			if(mixer_load_from_buf(buf, nRead))
			{
				PX4_INFO("Successfully initialized mixer from config file in /mnt/persist/flightparams/");
				mixer_initialized = 1;
			}
		}
		close(fd_load);
	}
	if(!mixer_initialized)
	{
		float pitch_scale = 1;
		float yaw_scale = 1;
		float deadband = 0;
		if(!mixer_initialize_quad_x(roll_scale, pitch_scale, yaw_scale, deadband))
		{
			PX4_ERR("mixer initialization failed");
			mixer_initialized = -1;
			return mixer_initialized;
		}
		PX4_WARN("mixer config file not found, successfully initialized default quad x mixer");
		mixer_initialized = true;
	}
#endif

	mixer = MultirotorMixer::from_text(mixer_control_callback, (uintptr_t)&_controls, buf, buflen);
	if(!(mixer == nullptr)) mixer_initialized = true;
	return mixer_initialized;
}

/**
* Rotate the motor rpm values based on the motor mappings configuration stored
* in motor_mapping
*/
void uart_esc_rotate_motors(int* motor_rpm,int num_rotors)
{
	ASSERT(num_rotors==4);
	int i;
	int motor_mapping[4] = {2,4,1,3};
	int motor_rpm_copy[4];

	memcpy(motor_rpm_copy, motor_rpm, sizeof(int)*num_rotors);

	for (i = 0; i < num_rotors; i++)
	{
		// motor_rpm[i] = motor_rpm_copy[motor_mapping[i]-1];
		motor_rpm[motor_mapping[i]-1] = motor_rpm_copy[i];
	}
}

void task_main(int argc, char *argv[])
{
	PX4_INFO("enter task_main");

	_outputs_pub = nullptr;

	// Hard coded options for now
	enum esc_model_t model = ESC_200QX;
	int baud_rate          = 250000;

	esc = UartEsc::get_instance();
	if (esc == NULL)
	{
		PX4_ERR("failed to new UartEsc instance");
	}
	else if (esc->initialize(model, _device, baud_rate) < 0)
	{
		PX4_ERR("failed to initialize UartEsc");
	}
	else
	{
		// Subscribe for orb topics
		_controls_sub = orb_subscribe(ORB_ID(actuator_controls_0)); // single group for now
		_armed_sub    = orb_subscribe(ORB_ID(actuator_armed));
		_param_sub    = orb_subscribe(ORB_ID(parameter_update));

		// initialize publication structures
		memset(&_outputs, 0, sizeof(_outputs));

		// set up poll topic and limit poll interval
		px4_pollfd_struct_t fds[1];
		fds[0].fd     = _controls_sub;
		fds[0].events = POLLIN;
		//orb_set_interval(_controls_sub, 10);  // max actuator update period, ms

		// set up mixer
		const char* mixer_filename = "/mnt/persist/flightparams/mixer_config.txt";
		if(initialize_mixer(mixer_filename) < 0)
		{
			PX4_ERR("Mixer initialization failed.");
			_task_should_exit = true;
		}

		// Main loop
		while(!_task_should_exit)
		{
			int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

			/* timed out - periodic check for _task_should_exit */
			if (pret == 0)
				continue;

			/* this is undesirable but not much we can do - might want to flag unhappy status */
			if (pret < 0) {
				PX4_WARN("poll error %d, %d", pret, errno);
				/* sleep a bit before next try */
				usleep(100000);
				continue;
			}

			// Handle new actuator controls data
			if (fds[0].revents & POLLIN) {

				// Grab new controls data
				orb_copy(ORB_ID(actuator_controls_0), _controls_sub, &_controls);
				// Mix to the outputs
				_outputs.timestamp = hrt_absolute_time();
				int motor_rpms[4]; //not yet supporting variable numbers

				if(_armed.armed)
				{
					_outputs.noutputs = mixer->mix(&_outputs.output[0], _num_mixer_outputs, NULL);

					// Send outputs to the ESCs
					for (unsigned outIdx=0; outIdx < _num_mixer_outputs; outIdx++)
					{
						// map -1.0 - 1.0 outputs to RPMs
						motor_rpms[outIdx] = ( ( _outputs.output[outIdx] + 1.0 ) / 2.0 ) *
							( esc->max_rpm()-esc->min_rpm() ) + esc->min_rpm();

					}
					uart_esc_rotate_motors(motor_rpms,_num_mixer_outputs);
				} else {
					for (unsigned outIdx=0; outIdx < _num_mixer_outputs; outIdx++)
					{
						motor_rpms[outIdx] = 0;
						_outputs.output[outIdx] = -1.0;
					}
				}
				esc->send_rpms(motor_rpms, _num_mixer_outputs, false);

				/*
				static int count=0;
				count++;
				if (!(count % 1)) {

					PX4_DEBUG(" ");
					PX4_DEBUG("Time       t: %13lu, Armed: %d",(unsigned long)_outputs.timestamp,_armed.armed);
					PX4_DEBUG("Act Controls: 0: %+8.4f, 1: %+8.4f,   2: %+8.4f, 3: %+8.4f",_controls.control[0],_controls.control[1],_controls.control[2],_controls.control[3]);
					PX4_DEBUG("Act Outputs : 0: %+8.4f, 1: %+8.4f,   2: %+8.4f, 3: %+8.4f",_outputs.output[0],_outputs.output[1],_outputs.output[2],_outputs.output[3]);
				}
				*/

				/* Publish mixed control outputs */
				if (_outputs_pub != nullptr) {
					orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &_outputs);
				} else {
					_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &_outputs);
				}
			}

			// Check for updates in other subscripions
			bool updated = false;
			orb_check(_armed_sub, &updated);
			if (updated) {
				orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
			}

			orb_check(_param_sub, &updated);
			if (updated) {
				orb_copy(ORB_ID(parameter_update), _param_sub, &_param_update);
				// The param update struct only contains a timestamp. You can use or not.
				// Update parameters relevant to this task (TBD)
			}
		}
	}

	PX4_WARN("closing uart_esc");

	delete esc;
}

/** uart_esc main entrance */
void task_main_trampoline(int argc, char *argv[])
{
	PX4_WARN("task_main_trampoline");
	task_main(argc, argv);
}

void start(const char* device)
{
	ASSERT(_task_handle == -1);

	_device = device;

	/* start the task */
	_task_handle = px4_task_spawn_cmd("uart_esc_main",
					SCHED_DEFAULT,
					SCHED_PRIORITY_MAX,
					1500,
					(px4_main_t)&task_main_trampoline,
					nullptr);

	if (_task_handle < 0) {
		warn("task start failed");
		return;
	}

	_is_running = true;
}

void stop()
{
	// TODO: set thread exit signal to terminate the task main thread

	_is_running = false;
	_device = NULL;
	_task_handle = -1;
}

void usage()
{
	PX4_WARN("missing command: try 'start', 'stop', 'status'");
	PX4_WARN("options:");
	PX4_WARN("    -D device");
}

}; // namespace uart_esc

int uart_esc_main(int argc, char *argv[])
{
	const char* device = NULL;
	int ch;
	int myoptind = 1;
	const char *myoptarg = NULL;

	// TODO: need to obtain the two parameters from command line arguments.
	// Should add this feature later
	device = "/dev/tty-2";

	// /* jump over start/off/etc and look at options first */
	// while ((ch = px4_getopt(argc, argv, "R:D:", &myoptind, &myoptarg)) != EOF) {
	// 	switch (ch) {
	// 	case 'D':
	// 		device = optarg;
	// 		break;
	// 	default:
	// 		uart_esc::usage();
	// 		exit(0);
	// 	}
	// }

	// Check on required arguments
	if (device == NULL)
	{
		uart_esc::usage();
		return 1;
	}

	const char *verb = argv[1];
	PX4_WARN("verb = %s", verb);
	PX4_WARN("result = %d", strcmp(verb, "start"));

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		if (uart_esc::_is_running) {
			PX4_WARN("uart_esc already running");
			return 1;
		}
		uart_esc::start(device);
	}

  else if (!strcmp(verb, "stop")) {
		if (uart_esc::_is_running) {
			PX4_WARN("uart_esc is not running");
			return 1;
		}
		uart_esc::stop();
	}

	else if (!strcmp(verb, "status")) {
		PX4_WARN("uart_esc is %s", uart_esc::_is_running ? "running":"stopped");
		return 0;
	}
	else {
		uart_esc::usage();
		return 1;
	}

	return 0;
}
