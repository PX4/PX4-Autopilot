/****************************************************************************
*
*   Copyright (c) 2015 Mark Charlebois. All rights reserved.
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
#include <uORB/topics/input_rc.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/mixer/mixer_multirotor.generated.h>
#include <systemlib/param/param.h>
#include <dev_fs_lib_serial.h>
#include <v1.0/checksum.h>

#define MAX_LEN_DEV_PATH 32
#define RC_MAGIC 	0xf6

namespace uart_esc
{
#define UART_ESC_MAX_MOTORS  4

volatile bool _task_should_exit = false; // flag indicating if uart_esc task should exit
static char _device[MAX_LEN_DEV_PATH];
static bool _is_running = false;         // flag indicating if uart_esc app is running
static px4_task_t _task_handle = -1;     // handle to the task main thread
void uart_esc_rotate_motors(int16_t *motor_rpm, int num_rotors); // motor re-mapping

// subscriptions
int		_controls_sub;
int		_armed_sub;
int		_param_sub;
int 	_fd;
// filenames
// /dev/fs/ is mapped to /usr/share/data/adsp/
static const char *MIXER_FILENAME = "/dev/fs/quad_x.main.mix";


// publications
orb_advert_t        	_outputs_pub;
orb_advert_t 			_rc_pub;

// topic structures
actuator_controls_s     _controls;
actuator_armed_s        _armed;
parameter_update_s      _param_update;
actuator_outputs_s      _outputs;
input_rc_s 				_rc;

/** Print out the usage information */
void usage();

/** uart_esc start */
void start(const char *device);

/** uart_esc stop */
void stop();

/** task main trampoline function */
void	task_main_trampoline(int argc, char *argv[]);

/** uart_esc thread primary entry point */
void task_main(int argc, char *argv[]);

/** mixer initialization */
MultirotorMixer *mixer;
int initialize_mixer(const char *mixer_filename);
int mixer_control_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input);

void parameters_init();
void parameters_update();

struct {
	int model;
	int baudrate;
	int px4_motor_mapping[UART_ESC_MAX_MOTORS];
} _parameters;

struct {
	param_t model;
	param_t baudrate;
	param_t px4_motor_mapping[UART_ESC_MAX_MOTORS];
} _parameter_handles;

void parameters_init()
{
	_parameter_handles.model 		= param_find("UART_ESC_MODEL");
	_parameter_handles.baudrate = param_find("UART_ESC_BAUDRATE");

	/* PX4 motor mapping parameters */
	for (unsigned int i = 0; i < UART_ESC_MAX_MOTORS; i++) {
		char nbuf[20];

		/* min values */
		sprintf(nbuf, "UART_ESC_PX4MOTOR%d", i + 1);
		_parameter_handles.px4_motor_mapping[i] = param_find(nbuf);
	}

	parameters_update();
}

void parameters_update()
{
	PX4_WARN("uart_esc_main parameters_update");
	int v_int;

	if (param_get(_parameter_handles.model, &v_int) == 0) {
		_parameters.model = v_int;
		PX4_WARN("UART_ESC_MODEL %d", _parameters.model);
	}

	if (param_get(_parameter_handles.baudrate, &v_int) == 0) {
		_parameters.baudrate = v_int;
		PX4_WARN("UART_ESC_BAUDRATE %d", _parameters.baudrate);
	}

	for (unsigned int i = 0; i < UART_ESC_MAX_MOTORS; i++) {
		if (param_get(_parameter_handles.px4_motor_mapping[i], &v_int) == 0) {
			_parameters.px4_motor_mapping[i] = v_int;
			PX4_WARN("UART_ESC_PX4MOTOR%d %d", i + 1, _parameters.px4_motor_mapping[i]);
		}
	}
}

int mixer_control_callback(uintptr_t handle,
			   uint8_t control_group,
			   uint8_t control_index,
			   float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls[control_group].control[control_index];

	/* motor spinup phase - lock throttle to zero *
	if (_pwm_limit.state == PWM_LIMIT_STATE_RAMP) {
		if (control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE &&
			control_index == actuator_controls_s::INDEX_THROTTLE) {
			* limit the throttle output to zero during motor spinup,
			 * as the motors cannot follow any demand yet
			 *
			input = 0.0f;
		}
	}
	*/
	return 0;
}


int initialize_mixer(const char *mixer_filename)
{
	mixer = nullptr;

	int mixer_initialized = -1;

	char buf[2048];
	unsigned int buflen = sizeof(buf);

	PX4_INFO("Initializing mixer from config file in %s", mixer_filename);

	int fd_load = ::open(mixer_filename, O_RDONLY);

	if (fd_load != -1) {
		int nRead = read(fd_load, buf, buflen);
		close(fd_load);

		if (nRead > 0) {
			mixer = MultirotorMixer::from_text(mixer_control_callback, (uintptr_t)&_controls, buf, buflen);

			if (mixer != nullptr) {
				PX4_INFO("Successfully initialized mixer from config file");
				mixer_initialized = 0;

			} else {
				PX4_WARN("Unable to parse from mixer config file");
			}

		} else {
			PX4_WARN("Unable to read from mixer config file");
		}

	} else {
		PX4_WARN("Unable to open mixer config file");
	}

	// mixer file loading failed, fall back to default mixer configuration for
	// QUAD_X airframe
	if (mixer_initialized < 0) {
		float roll_scale = 1;
		float pitch_scale = 1;
		float yaw_scale = 1;
		float deadband = 0;

		mixer = new MultirotorMixer(mixer_control_callback, (uintptr_t)&_controls,
					    MultirotorGeometry::QUAD_X,
					    roll_scale, pitch_scale, yaw_scale, deadband);

		if (mixer == nullptr) {
			PX4_ERR("mixer initialization failed");
			mixer_initialized = -1;
			return mixer_initialized;
		}

		PX4_WARN("mixer config file not found, successfully initialized default quad x mixer");
		mixer_initialized = 0;
	}

	return mixer_initialized;
}

/**
* Rotate the motor rpm values based on the motor mappings configuration stored
* in motor_mapping
*/
void uart_esc_rotate_motors(int16_t *motor_rpm, int num_rotors)
{
	ASSERT(num_rotors <= UART_ESC_MAX_MOTORS);
	int i;
	int16_t motor_rpm_copy[UART_ESC_MAX_MOTORS];

	for (i = 0; i < num_rotors; i++) {
		motor_rpm_copy[i] = motor_rpm[i];
	}

	for (i = 0; i < num_rotors; i++) {
		motor_rpm[_parameters.px4_motor_mapping[i] - 1] = motor_rpm_copy[i];
	}
}

int uart_initialize(const char *device, int baud)
{

	int fd = ::open(device, O_RDWR | O_NONBLOCK);

	if (fd == -1) {
		PX4_ERR("failed in open");
		return -1;
	}

	struct dspal_serial_open_options options;

	options.bit_rate = DSPAL_SIO_BITRATE_57600;

	options.tx_flow = DSPAL_SIO_FCTL_OFF;

	options.rx_flow = DSPAL_SIO_FCTL_OFF;

	options.rx_data_callback = nullptr;

	options.tx_data_callback = nullptr;

	options.is_tx_data_synchronous = false;

	int ret = ::ioctl(fd, SERIAL_IOCTL_OPEN_OPTIONS, (void *)&options);

	if (ret != 0) {
		PX4_ERR("Failed to setup UART flow control options");
	}

	_fd = fd;
	return 0;
}

void handleRC(int uart_fd, struct input_rc_s *rc)
{
	// read uart until we get the magic number
	char buf[10];

	while (true) {
		int ret = ::read(uart_fd, &buf, sizeof(buf));
		PX4_ERR("got %d", ret);

		if (ret < 1) {
			break;
		}
	}
}

void task_main(int argc, char *argv[])
{
	PX4_INFO("enter task_main");

	_outputs_pub = nullptr;

	parameters_init();

	if (uart_initialize(_device, _parameters.baudrate) < 0) {
		PX4_ERR("failed to initialize UartEsc");

	} else {
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
		if (initialize_mixer(MIXER_FILENAME) < 0) {
			PX4_ERR("Mixer initialization failed.");
			_task_should_exit = true;
		}

		// Main loop
		while (!_task_should_exit) {
			int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

			/* timed out - periodic check for _task_should_exit */
			if (pret == 0) {
				continue;
			}

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
				int16_t motor_rpms[UART_ESC_MAX_MOTORS];

				if (_armed.armed) {
					_outputs.noutputs = mixer->mix(&_outputs.output[0],
								       actuator_controls_0_s::NUM_ACTUATOR_CONTROLS,
								       NULL);

					// Make sure we support only up to UART_ESC_MAX_MOTORS motors
					if (_outputs.noutputs > UART_ESC_MAX_MOTORS) {
						PX4_ERR("Unsupported motors %d, up to %d motors supported",
							_outputs.noutputs, UART_ESC_MAX_MOTORS);
						continue;
					}

					// iterate actuators
					for (unsigned i = 0; i < _outputs.noutputs; i++) {
						// last resort: catch NaN, INF and out-of-band errors
						if (i < _outputs.noutputs &&
						    PX4_ISFINITE(_outputs.output[i]) &&
						    _outputs.output[i] >= -1.0f &&
						    _outputs.output[i] <= 1.0f) {
							// scale for PWM output 1000 - 2000us
							_outputs.output[i] = 1500 + (500 * _outputs.output[i]);

						} else {
							//
							// Value is NaN, INF or out of band - set to the minimum value.
							// This will be clearly visible on the servo status and will limit the risk of accidentally
							// spinning motors. It would be deadly in flight.
							//
							_outputs.output[i] = 900;
						}
					}

					uart_esc_rotate_motors(motor_rpms, _outputs.noutputs);

				} else {
					_outputs.noutputs = UART_ESC_MAX_MOTORS;

					for (unsigned outIdx = 0; outIdx < _outputs.noutputs; outIdx++) {
						_outputs.output[outIdx] = 900;
					}
				}

				uint8_t data[11];
				struct PACKED {
					uint8_t magic = 0xF7;
					uint16_t period[4];
					uint16_t crc;
				} frame;

				for (uint8_t i = 0; i < 4; i++) {
					frame.period[i] = _outputs.output[i];
				}

				frame.crc = crc_calculate((uint8_t *)frame.period, 4 * 2);

				data[0] = frame.magic;
				memcpy(&data[1], (uint8_t *)frame.period, sizeof(frame.period));
				memcpy(&data[9], (uint8_t *)&frame.crc, sizeof(frame.crc));

				::write(_fd, data, sizeof(data));

				/*
				static int count=0;
				count++;
				if (!(count % 1)) {
					PX4_DEBUG("                                                                  ");
					PX4_DEBUG("Time       t: %13lu, Armed: %d                                  ",(unsigned long)_outputs.timestamp,_armed.armed);
					PX4_DEBUG("Act Controls: 0: %+8.4f, 1: %+8.4f,   2: %+8.4f, 3: %+8.4f  ",_controls.control[0],_controls.control[1],_controls.control[2],_controls.control[3]);
					PX4_DEBUG("Act Outputs : 0: %+8.4f, 1: %+8.4f,   2: %+8.4f, 3: %+8.4f  ",_outputs.output[0],_outputs.output[1],_outputs.output[2],_outputs.output[3]);
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
}

/** uart_esc main entrance */
void task_main_trampoline(int argc, char *argv[])
{
	PX4_WARN("task_main_trampoline");
	task_main(argc, argv);
}

void start()
{
	ASSERT(_task_handle == -1);

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
	// TODO - set thread exit signal to terminate the task main thread

	_is_running = false;
	_task_handle = -1;
}

void usage()
{
	PX4_WARN("missing command: try 'start', 'stop', 'status'");
	PX4_WARN("options:");
	PX4_WARN("    -d device");
}

} // namespace uart_esc

/** driver 'main' command */
extern "C" __EXPORT int uart_esc_main(int argc, char *argv[]);

int uart_esc_main(int argc, char *argv[])
{
	const char *device = NULL;
	int ch;
	int myoptind = 1;
	const char *myoptarg = NULL;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		default:
			uart_esc::usage();
			return 1;
		}
	}

	// Check on required arguments
	if (device == NULL || strlen(device) == 0) {
		uart_esc::usage();
		return 1;
	}

	memset(uart_esc::_device, 0, MAX_LEN_DEV_PATH);
	strncpy(uart_esc::_device, device, strlen(device));

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		if (uart_esc::_is_running) {
			PX4_WARN("uart_esc already running");
			return 1;
		}

		uart_esc::start();
	}

	else if (!strcmp(verb, "stop")) {
		if (uart_esc::_is_running) {
			PX4_WARN("uart_esc is not running");
			return 1;
		}

		uart_esc::stop();
	}

	else if (!strcmp(verb, "status")) {
		PX4_WARN("uart_esc is %s", uart_esc::_is_running ? "running" : "stopped");
		return 0;

	} else {
		uart_esc::usage();
		return 1;
	}

	return 0;
}