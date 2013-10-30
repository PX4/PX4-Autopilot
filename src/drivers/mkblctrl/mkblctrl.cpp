/****************************************************************************
 *
 *   Copyright (C) 2012,2013 PX4 Development Team. All rights reserved.
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
 * @file mkblctrl.cpp
 *
 * Driver/configurator for the Mikrokopter BL-Ctrl.
 * Marco Bauer
 *
 */

#include <nuttx/config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/i2c.h>

#include <board_config.h>

#include <drivers/device/device.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_rc_input.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/mixer/mixer.h>
#include <drivers/drv_mixer.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_effective.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/esc_status.h>

#include <systemlib/err.h>

#define I2C_BUS_SPEED					400000
#define UPDATE_RATE						400
#define MAX_MOTORS 						8
#define BLCTRL_BASE_ADDR 				0x29
#define BLCTRL_OLD 						0
#define BLCTRL_NEW 						1
#define BLCTRL_MIN_VALUE				-0.920F
#define MOTOR_STATE_PRESENT_MASK		0x80
#define MOTOR_STATE_ERROR_MASK		0x7F
#define MOTOR_SPINUP_COUNTER			2500
#define ESC_UORB_PUBLISH_DELAY			200

class MK : public device::I2C
{
public:
	enum Mode {
		MODE_NONE,
		MODE_2PWM,
		MODE_4PWM,
		MODE_6PWM,
	};

	enum MappingMode {
		MAPPING_MK = 0,
		MAPPING_PX4,
	};

	enum FrameType {
		FRAME_PLUS = 0,
		FRAME_X,
	};

	MK(int bus);
	~MK();

	virtual int	ioctl(file *filp, int cmd, unsigned long arg);
	virtual int	init(unsigned motors);
	virtual ssize_t	write(file *filp, const char *buffer, size_t len);

	int		set_mode(Mode mode);
	int		set_pwm_rate(unsigned rate);
	int		set_motor_count(unsigned count);
	int		set_motor_test(bool motortest);
	int		set_overrideSecurityChecks(bool overrideSecurityChecks);
	int		set_px4mode(int px4mode);
	int		set_frametype(int frametype);
	unsigned int		mk_check_for_blctrl(unsigned int count, bool showOutput);

private:
	static const unsigned _max_actuators = MAX_MOTORS;
	static const bool	showDebug = false;

	Mode		_mode;
	int 		_update_rate;
	int 		_current_update_rate;
	int		_task;
	int		_t_actuators;
	int		_t_actuator_armed;
	unsigned int		_motor;
	int    _px4mode;
	int    _frametype;

	orb_advert_t	_t_outputs;
	orb_advert_t	_t_actuators_effective;
	orb_advert_t	_t_esc_status;

	unsigned int	_num_outputs;
	bool		_primary_pwm_device;
	bool     _motortest;
	bool 		_overrideSecurityChecks;

	volatile bool	_task_should_exit;
	bool		_armed;

	unsigned long	debugCounter;

	MixerGroup	*_mixers;

	actuator_controls_s _controls;

	static void	task_main_trampoline(int argc, char *argv[]);
	void		task_main() __attribute__((noreturn));

	static int	control_callback(uintptr_t handle,
					 uint8_t control_group,
					 uint8_t control_index,
					 float &input);

	int		pwm_ioctl(file *filp, int cmd, unsigned long arg);

	struct GPIOConfig {
		uint32_t	input;
		uint32_t	output;
		uint32_t	alt;
	};

	static const GPIOConfig	_gpio_tab[];
	static const unsigned	_ngpio;

	void		gpio_reset(void);
	void		gpio_set_function(uint32_t gpios, int function);
	void		gpio_write(uint32_t gpios, int function);
	uint32_t	gpio_read(void);
	int		gpio_ioctl(file *filp, int cmd, unsigned long arg);
	int			mk_servo_arm(bool status);

	int 		mk_servo_set(unsigned int chan, short val);
	int 		mk_servo_set_value(unsigned int chan, short val);
	int 		mk_servo_test(unsigned int chan);
	short		scaling(float val, float inMin, float inMax, float outMin, float outMax);


};

const MK::GPIOConfig MK::_gpio_tab[] = {
	{GPIO_GPIO0_INPUT, GPIO_GPIO0_OUTPUT, 0},
	{GPIO_GPIO1_INPUT, GPIO_GPIO1_OUTPUT, 0},
	{GPIO_GPIO2_INPUT, GPIO_GPIO2_OUTPUT, GPIO_USART2_CTS_1},
	{GPIO_GPIO3_INPUT, GPIO_GPIO3_OUTPUT, GPIO_USART2_RTS_1},
	{GPIO_GPIO4_INPUT, GPIO_GPIO4_OUTPUT, GPIO_USART2_TX_1},
	{GPIO_GPIO5_INPUT, GPIO_GPIO5_OUTPUT, GPIO_USART2_RX_1},
	{GPIO_GPIO6_INPUT, GPIO_GPIO6_OUTPUT, GPIO_CAN2_TX_2},
	{GPIO_GPIO7_INPUT, GPIO_GPIO7_OUTPUT, GPIO_CAN2_RX_2},
};

const unsigned MK::_ngpio = sizeof(MK::_gpio_tab) / sizeof(MK::_gpio_tab[0]);

const int blctrlAddr_quad_plus[] = { 2, 2, -2, -2, 0, 0, 0, 0 };	// Addresstranslator for Quad + configuration
const int blctrlAddr_hexa_plus[] = { 0, 2, 2, -2, 1, -3, 0, 0 };	// Addresstranslator for Hexa + configuration
const int blctrlAddr_octo_plus[] = { 0, 3, -1, 0, 3, 0, 0, -5 };	// Addresstranslator for Octo + configuration

const int blctrlAddr_quad_x[] = { 2, 2, -2, -2, 0, 0, 0, 0 };	// Addresstranslator for Quad X configuration
const int blctrlAddr_hexa_x[] = { 2, 4, -2, 0, -3, -1, 0, 0 };	// Addresstranslator for Hexa X configuration
const int blctrlAddr_octo_x[] = { 1, 4, 0, 1, -4, 1, 1, -4 };	// Addresstranslator for Octo X configuration

const int blctrlAddr_px4[]  = { 0, 0, 0, 0, 0, 0, 0, 0};

int addrTranslator[] = {0, 0, 0, 0, 0, 0, 0, 0};

struct MotorData_t {
	unsigned int Version;                        // the version of the BL (0 = old)
	unsigned int SetPoint;                       // written by attitude controller
	unsigned int SetPointLowerBits;      // for higher Resolution of new BLs
	float SetPoint_PX4; 			     // Values from PX4
	unsigned int State;                          // 7 bit for I2C error counter, highest bit indicates if motor is present
	unsigned int ReadMode;                       // select data to read
	unsigned short RawPwmValue;							// length of PWM pulse
	// the following bytes must be exactly in that order!
	unsigned int Current;                        // in 0.1 A steps, read back from BL
	unsigned int MaxPWM;                         // read back from BL is less than 255 if BL is in current limit
	unsigned int Temperature;            // old BL-Ctrl will return a 255 here, the new version the temp. in
	unsigned int RoundCount;
};

MotorData_t Motor[MAX_MOTORS];


namespace
{

MK	*g_mk;

} // namespace

MK::MK(int bus) :
	I2C("mkblctrl", "/dev/mkblctrl", bus, 0, I2C_BUS_SPEED),
	_mode(MODE_NONE),
	_update_rate(50),
	_task(-1),
	_t_actuators(-1),
	_t_actuator_armed(-1),
	_t_outputs(0),
	_t_actuators_effective(0),
	_t_esc_status(0),
	_num_outputs(0),
	_motortest(false),
	_overrideSecurityChecks(false),
	_motor(-1),
	_px4mode(MAPPING_MK),
	_frametype(FRAME_PLUS),
	_primary_pwm_device(false),
	_task_should_exit(false),
	_armed(false),
	_mixers(nullptr)
{
	_debug_enabled = true;
}

MK::~MK()
{
	if (_task != -1) {
		/* tell the task we want it to go away */
		_task_should_exit = true;

		unsigned i = 10;

		do {
			/* wait 50ms - it should wake every 100ms or so worst-case */
			usleep(50000);

			/* if we have given up, kill it */
			if (--i == 0) {
				task_delete(_task);
				break;
			}

		} while (_task != -1);
	}

	/* clean up the alternate device node */
	if (_primary_pwm_device)
		unregister_driver(PWM_OUTPUT_DEVICE_PATH);

	g_mk = nullptr;
}

int
MK::init(unsigned motors)
{
	_num_outputs = motors;
	debugCounter = 0;
	int ret;
	ASSERT(_task == -1);

	ret = I2C::init();

	if (ret != OK) {
		warnx("I2C init failed");
		return ret;
	}

	usleep(500000);

	/* try to claim the generic PWM output device node as well - it's OK if we fail at this */
	ret = register_driver(PWM_OUTPUT_DEVICE_PATH, &fops, 0666, (void *)this);

	if (ret == OK) {
		log("default PWM output device");
		_primary_pwm_device = true;
	}

	/* reset GPIOs */
	gpio_reset();

	/* start the IO interface task */
	_task = task_spawn_cmd("mkblctrl",
			   SCHED_DEFAULT,
			   SCHED_PRIORITY_MAX - 20,
			   2048,
			   (main_t)&MK::task_main_trampoline,
			   nullptr);


	if (_task < 0) {
		debug("task start failed: %d", errno);
		return -errno;
	}

	return OK;
}

void
MK::task_main_trampoline(int argc, char *argv[])
{
	g_mk->task_main();
}

int
MK::set_mode(Mode mode)
{
	/*
	 * Configure for PWM output.
	 *
	 * Note that regardless of the configured mode, the task is always
	 * listening and mixing; the mode just selects which of the channels
	 * are presented on the output pins.
	 */
	switch (mode) {
	case MODE_2PWM:
		up_pwm_servo_deinit();
		_update_rate = UPDATE_RATE;	/* default output rate */
		break;

	case MODE_4PWM:
		up_pwm_servo_deinit();
		_update_rate = UPDATE_RATE;	/* default output rate */
		break;

	case MODE_NONE:
		debug("MODE_NONE");
		/* disable servo outputs and set a very low update rate */
		up_pwm_servo_deinit();
		_update_rate = UPDATE_RATE;
		break;

	default:
		return -EINVAL;
	}

	_mode = mode;
	return OK;
}

int
MK::set_pwm_rate(unsigned rate)
{
	if ((rate > 500) || (rate < 10))
		return -EINVAL;

	_update_rate = rate;
	return OK;
}

int
MK::set_px4mode(int px4mode)
{
	_px4mode = px4mode;
}

int
MK::set_frametype(int frametype)
{
	_frametype = frametype;
}


int
MK::set_motor_count(unsigned count)
{
	if (count > 0) {

		_num_outputs = count;

		if (_px4mode == MAPPING_MK) {
			if (_frametype == FRAME_PLUS) {
				fprintf(stderr, "[mkblctrl] addresstanslator for Mikrokopter addressing used. Frametype: +\n");

			} else if (_frametype == FRAME_X) {
				fprintf(stderr, "[mkblctrl] addresstanslator for Mikrokopter addressing used. Frametype: X\n");
			}

			if (_num_outputs == 4) {
				if (_frametype == FRAME_PLUS) {
					memcpy(&addrTranslator, &blctrlAddr_quad_plus, sizeof(blctrlAddr_quad_plus));

				} else if (_frametype == FRAME_X) {
					memcpy(&addrTranslator, &blctrlAddr_quad_x, sizeof(blctrlAddr_quad_x));
				}

			} else if (_num_outputs == 6) {
				if (_frametype == FRAME_PLUS) {
					memcpy(&addrTranslator, &blctrlAddr_hexa_plus, sizeof(blctrlAddr_hexa_plus));

				} else if (_frametype == FRAME_X) {
					memcpy(&addrTranslator, &blctrlAddr_hexa_x, sizeof(blctrlAddr_hexa_x));
				}

			} else if (_num_outputs == 8) {
				if (_frametype == FRAME_PLUS) {
					memcpy(&addrTranslator, &blctrlAddr_octo_plus, sizeof(blctrlAddr_octo_plus));

				} else if (_frametype == FRAME_X) {
					memcpy(&addrTranslator, &blctrlAddr_octo_x, sizeof(blctrlAddr_octo_x));
				}
			}

		} else {
			fprintf(stderr, "[mkblctrl] PX4 native addressing used.\n");
			memcpy(&addrTranslator, &blctrlAddr_px4, sizeof(blctrlAddr_px4));
		}

		if (_num_outputs == 4) {
			fprintf(stderr, "[mkblctrl] Quadrocopter Mode (4)\n");

		} else 	if (_num_outputs == 6) {
			fprintf(stderr, "[mkblctrl] Hexacopter Mode (6)\n");

		} else 	if (_num_outputs == 8) {
			fprintf(stderr, "[mkblctrl] Octocopter Mode (8)\n");
		}

		return OK;

	} else {
		return -1;
	}

}

int
MK::set_motor_test(bool motortest)
{
	_motortest = motortest;
	return OK;
}

int
MK::set_overrideSecurityChecks(bool overrideSecurityChecks)
{
	_overrideSecurityChecks = overrideSecurityChecks;
	return OK;
}

short
MK::scaling(float val, float inMin, float inMax, float outMin, float outMax)
{
	short retVal = 0;

	retVal = (val - inMin) / (inMax - inMin) * (outMax - outMin) + outMin;

	if (retVal < outMin) {
		retVal = outMin;

	} else if (retVal > outMax) {
		retVal = outMax;
	}

	return retVal;
}

void
MK::task_main()
{
	long update_rate_in_us = 0;
	float tmpVal = 0;

	/*
	 * Subscribe to the appropriate PWM output topic based on whether we are the
	 * primary PWM output or not.
	 */
	_t_actuators = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);

	/* force a reset of the update rate */
	_current_update_rate = 0;

	_t_actuator_armed = orb_subscribe(ORB_ID(actuator_armed));
	orb_set_interval(_t_actuator_armed, 200);		/* 5Hz update rate */

	/* advertise the mixed control outputs */
	actuator_outputs_s outputs;
	memset(&outputs, 0, sizeof(outputs));
	/* advertise the mixed control outputs */
	_t_outputs = orb_advertise(_primary_pwm_device ? ORB_ID_VEHICLE_CONTROLS : ORB_ID(actuator_outputs_1),
				   &outputs);

	/* advertise the effective control inputs */
	actuator_controls_effective_s controls_effective;
	memset(&controls_effective, 0, sizeof(controls_effective));
	/* advertise the effective control inputs */
	_t_actuators_effective = orb_advertise(_primary_pwm_device ? ORB_ID_VEHICLE_ATTITUDE_CONTROLS_EFFECTIVE : ORB_ID(actuator_controls_effective_1),
				   &controls_effective);

	/* advertise the blctrl status */
	esc_status_s esc;
	memset(&esc, 0, sizeof(esc));
	_t_esc_status = orb_advertise(ORB_ID(esc_status), &esc);



	pollfd fds[2];
	fds[0].fd = _t_actuators;
	fds[0].events = POLLIN;
	fds[1].fd = _t_actuator_armed;
	fds[1].events = POLLIN;

	log("starting");

	/* loop until killed */
	while (!_task_should_exit) {

		/* handle update rate changes */
		if (_current_update_rate != _update_rate) {
			int update_rate_in_ms = int(1000 / _update_rate);
			update_rate_in_us = long(1000000 / _update_rate);

			/* reject faster than 500 Hz updates */
			if (update_rate_in_ms < 2) {
				update_rate_in_ms = 2;
				_update_rate = 500;
			}

			/* reject slower than 50 Hz updates */
			if (update_rate_in_ms > 20) {
				update_rate_in_ms = 20;
				_update_rate = 50;
			}

			orb_set_interval(_t_actuators, update_rate_in_ms);
			up_pwm_servo_set_rate(_update_rate);
			_current_update_rate = _update_rate;
		}

		/* sleep waiting for data max 100ms */
		int ret = ::poll(&fds[0], 2, 100);

		/* this would be bad... */
		if (ret < 0) {
			log("poll error %d", errno);
			usleep(1000000);
			continue;
		}

		/* do we have a control update? */
		if (fds[0].revents & POLLIN) {

			/* get controls - must always do this to avoid spinning */
			orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, _t_actuators, &_controls);

			/* can we mix? */
			if (_mixers != nullptr) {

				/* do mixing */
				outputs.noutputs = _mixers->mix(&outputs.output[0], _num_outputs);
				outputs.timestamp = hrt_absolute_time();

				// XXX output actual limited values
				memcpy(&controls_effective, &_controls, sizeof(controls_effective));

				/* iterate actuators */
				for (unsigned int i = 0; i < _num_outputs; i++) {

					/* last resort: catch NaN, INF and out-of-band errors */
					if (i < outputs.noutputs &&
					    isfinite(outputs.output[i]) &&
					    outputs.output[i] >= -1.0f &&
					    outputs.output[i] <= 1.0f) {
						/* scale for PWM output 900 - 2100us */
						/* nothing to do here */
					} else {
						/*
						 * Value is NaN, INF or out of band - set to the minimum value.
						 * This will be clearly visible on the servo status and will limit the risk of accidentally
						 * spinning motors. It would be deadly in flight.
						 */
						if (outputs.output[i] < -1.0f) {
							outputs.output[i] = -1.0f;

						} else if (outputs.output[i] > 1.0f) {
							outputs.output[i] = 1.0f;

						} else {
							outputs.output[i] = -1.0f;
						}
					}

					if(!_overrideSecurityChecks) {
						/* don't go under BLCTRL_MIN_VALUE */
						if (outputs.output[i] < BLCTRL_MIN_VALUE) {
							outputs.output[i] = BLCTRL_MIN_VALUE;
						}
					}

					/* output to BLCtrl's */
					if (_motortest == true) {
						mk_servo_test(i);

					} else {
						//mk_servo_set_value(i, scaling(outputs.output[i], -1.0f, 1.0f, 0, 1024));	// scale the output to 0 - 1024 and sent to output routine
						// 11 Bit
						Motor[i].SetPoint_PX4 = outputs.output[i];
						mk_servo_set(i, scaling(outputs.output[i], -1.0f, 1.0f, 0, 2047));	// scale the output to 0 - 2047 and sent to output routine
					}

				}

			}



		}

		/* how about an arming update? */
		if (fds[1].revents & POLLIN) {
			actuator_armed_s aa;

			/* get new value */
			orb_copy(ORB_ID(actuator_armed), _t_actuator_armed, &aa);

			/* update PWM servo armed status if armed and not locked down */
			mk_servo_arm(aa.armed && !aa.lockdown);
		}



		/*
		 * Only update esc topic every half second.
		 */

		if (hrt_absolute_time() - esc.timestamp > 500000) {
			esc.counter++;
			esc.timestamp = hrt_absolute_time();
			esc.esc_count = (uint8_t) _num_outputs;
			esc.esc_connectiontype = ESC_CONNECTION_TYPE_I2C;

			for (unsigned int i = 0; i < _num_outputs; i++) {
				esc.esc[i].esc_address = (uint8_t) BLCTRL_BASE_ADDR + i;
				esc.esc[i].esc_vendor = ESC_VENDOR_MIKROKOPTER;
				esc.esc[i].esc_version = (uint16_t) Motor[i].Version;
				esc.esc[i].esc_voltage = (uint16_t) 0;
				esc.esc[i].esc_current = (uint16_t) Motor[i].Current;
				esc.esc[i].esc_rpm = (uint16_t) 0;
				esc.esc[i].esc_setpoint = (float) Motor[i].SetPoint_PX4;
				if (Motor[i].Version == 1) {
					// BLCtrl 2.0 (11Bit)
					esc.esc[i].esc_setpoint_raw = (uint16_t) (Motor[i].SetPoint<<3) | Motor[i].SetPointLowerBits;
				} else {
					// BLCtrl < 2.0 (8Bit)
					esc.esc[i].esc_setpoint_raw = (uint16_t) Motor[i].SetPoint;
				}
				esc.esc[i].esc_temperature = (uint16_t) Motor[i].Temperature;
				esc.esc[i].esc_state = (uint16_t) Motor[i].State;
				esc.esc[i].esc_errorcount = (uint16_t) 0;
			}

			orb_publish(ORB_ID(esc_status), _t_esc_status, &esc);
		}

	}

	//::close(_t_esc_status);
	::close(_t_actuators);
	::close(_t_actuators_effective);
	::close(_t_actuator_armed);


	/* make sure servos are off */
	up_pwm_servo_deinit();

	log("stopping");

	/* note - someone else is responsible for restoring the GPIO config */

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}


int
MK::mk_servo_arm(bool status)
{
	_armed = status;
	return 0;
}


unsigned int
MK::mk_check_for_blctrl(unsigned int count, bool showOutput)
{
	_retries = 50;
	uint8_t foundMotorCount = 0;

	for (unsigned i = 0; i < MAX_MOTORS; i++) {
		Motor[i].Version = 0;
		Motor[i].SetPoint = 0;
		Motor[i].SetPointLowerBits = 0;
		Motor[i].State = 0;
		Motor[i].ReadMode = 0;
		Motor[i].RawPwmValue = 0;
		Motor[i].Current = 0;
		Motor[i].MaxPWM = 0;
		Motor[i].Temperature = 0;
		Motor[i].RoundCount = 0;
	}

	uint8_t msg = 0;
	uint8_t result[3];

	for (unsigned i = 0; i < count; i++) {
		result[0] = 0;
		result[1] = 0;
		result[2] = 0;

		set_address(BLCTRL_BASE_ADDR + i);

		if (OK == transfer(&msg, 1, &result[0], 3)) {
			Motor[i].Current = result[0];
			Motor[i].MaxPWM = result[1];
			Motor[i].Temperature = result[2];
			Motor[i].State |= MOTOR_STATE_PRESENT_MASK; // set present bit;
			foundMotorCount++;

			if (Motor[i].MaxPWM == 250) {
				Motor[i].Version = BLCTRL_NEW;

			} else {
				Motor[i].Version = BLCTRL_OLD;
			}
		}
	}

	if (showOutput) {
		fprintf(stderr, "[mkblctrl] MotorsFound: %i\n", foundMotorCount);

		for (unsigned i = 0; i < foundMotorCount; i++) {
			fprintf(stderr, "[mkblctrl] blctrl[%i] : found=%i\tversion=%i\tcurrent=%i\tmaxpwm=%i\ttemperature=%i\n", i, Motor[i].State, Motor[i].Version, Motor[i].Current, Motor[i].MaxPWM, Motor[i].Temperature);
		}

		
		if(!_overrideSecurityChecks) {
			if (foundMotorCount != 4 && foundMotorCount != 6 && foundMotorCount != 8) {
				_task_should_exit = true;
			}
		}
	}

	return foundMotorCount;
}

int
MK::mk_servo_set(unsigned int chan, short val)
{
	short tmpVal = 0;
	_retries = 0;
	uint8_t result[3] = { 0, 0, 0 };
	uint8_t msg[2] = { 0, 0 };
	uint8_t rod = 0;
	uint8_t bytesToSendBL2 = 2;

	tmpVal = val;

	if (tmpVal > 2047) {
		tmpVal = 2047;

	} else if (tmpVal < 0) {
		tmpVal = 0;
	}

	Motor[chan].SetPoint = (uint8_t)(tmpVal>>3)&0xff;
	Motor[chan].SetPointLowerBits = ((uint8_t)tmpVal%8)&0x07;

	if (_armed == false) {
		Motor[chan].SetPoint = 0;
		Motor[chan].SetPointLowerBits = 0;
	}

	//if(Motor[chan].State & MOTOR_STATE_PRESENT_MASK) {
	set_address(BLCTRL_BASE_ADDR + (chan + addrTranslator[chan]));

	if (Motor[chan].Version == BLCTRL_OLD) {
		/*
		*	Old BL-Ctrl 8Bit served. Version < 2.0
		*/
		msg[0] = Motor[chan].SetPoint;

		if (Motor[chan].RoundCount >= 16) {
			// on each 16th cyle we read out the status messages from the blctrl
			if (OK == transfer(&msg[0], 1, &result[0], 2)) {
				Motor[chan].Current = result[0];
				Motor[chan].MaxPWM = result[1];
				Motor[chan].Temperature = 255;;

			} else {
				if ((Motor[chan].State & MOTOR_STATE_ERROR_MASK) < MOTOR_STATE_ERROR_MASK) Motor[chan].State++;	// error
			}

			Motor[chan].RoundCount = 0;

		} else {
			if (OK != transfer(&msg[0], 1, nullptr, 0)) {
				if ((Motor[chan].State & MOTOR_STATE_ERROR_MASK) < MOTOR_STATE_ERROR_MASK) Motor[chan].State++;	// error
			}
		}

	} else {
		/*
		*	New BL-Ctrl 11Bit served. Version >= 2.0
		*/
		msg[0] = Motor[chan].SetPoint;
		msg[1] = Motor[chan].SetPointLowerBits;

		if (Motor[chan].SetPointLowerBits == 0) {
			bytesToSendBL2 = 1;	// if setpoint lower bits are zero, we send only the higher bits - this saves time
		}

		if (Motor[chan].RoundCount >= 16) {
			// on each 16th cyle we read out the status messages from the blctrl
			if (OK == transfer(&msg[0], bytesToSendBL2, &result[0], 3)) {
				Motor[chan].Current = result[0];
				Motor[chan].MaxPWM = result[1];
				Motor[chan].Temperature = result[2];

			} else {
				if ((Motor[chan].State & MOTOR_STATE_ERROR_MASK) < MOTOR_STATE_ERROR_MASK) Motor[chan].State++;	// error
			}

			Motor[chan].RoundCount = 0;

		} else {
			if (OK != transfer(&msg[0], bytesToSendBL2, nullptr, 0)) {
				if ((Motor[chan].State & MOTOR_STATE_ERROR_MASK) < MOTOR_STATE_ERROR_MASK) Motor[chan].State++;	// error
			}
		}

	}

	Motor[chan].RoundCount++;
	//}

	if (showDebug == true) {
		debugCounter++;

		if (debugCounter == 2000) {
			debugCounter = 0;

			for (int i = 0; i < _num_outputs; i++) {
				if (Motor[i].State & MOTOR_STATE_PRESENT_MASK) {
					fprintf(stderr, "[mkblctrl] #%i:\tVer: %i\tVal: %i\tCurr: %i\tMaxPWM: %i\tTemp: %i\tState: %i\n", i, Motor[i].Version, Motor[i].SetPoint, Motor[i].Current, Motor[i].MaxPWM, Motor[i].Temperature, Motor[i].State);
				}
			}

			fprintf(stderr, "\n");
		}
	}

	return 0;
}

int
MK::mk_servo_set_value(unsigned int chan, short val)
{
	_retries = 0;
	int ret;
	short tmpVal = 0;
	uint8_t msg[2] = { 0, 0 };

	tmpVal = val;

	if (tmpVal > 1024) {
		tmpVal = 1024;

	} else if (tmpVal < 0) {
		tmpVal = 0;
	}

	Motor[chan].SetPoint = (uint8_t)(tmpVal / 4);

	if (_armed == false) {
		Motor[chan].SetPoint = 0;
		Motor[chan].SetPointLowerBits = 0;
	}

	msg[0] = Motor[chan].SetPoint;

	set_address(BLCTRL_BASE_ADDR + (chan + addrTranslator[chan]));
	ret = transfer(&msg[0], 1, nullptr, 0);

	ret = OK;
	return ret;
}


int
MK::mk_servo_test(unsigned int chan)
{
	int ret = 0;
	float tmpVal = 0;
	float val = -1;
	_retries = 0;
	uint8_t msg[2] = { 0, 0 };

	if (debugCounter >= MOTOR_SPINUP_COUNTER) {
		debugCounter = 0;
		_motor++;

		if (_motor < _num_outputs) {
			fprintf(stderr, "[mkblctrl] Motortest - #%i:\tspinup\n", _motor);
		}

		if (_motor >= _num_outputs) {
			_motor = -1;
			_motortest = false;
		}
	}

	debugCounter++;

	if (_motor == chan) {
		val = BLCTRL_MIN_VALUE;

	} else {
		val = -1;
	}

	tmpVal = (511 + (511 * val));

	if (tmpVal > 1024) {
		tmpVal = 1024;
	}

	Motor[chan].SetPoint = (uint8_t)(tmpVal / 4);

	if (_motor != chan) {
		Motor[chan].SetPoint = 0;
		Motor[chan].SetPointLowerBits = 0;
	}

	if (Motor[chan].Version == BLCTRL_OLD) {
		msg[0] = Motor[chan].SetPoint;

	} else {
		msg[0] = Motor[chan].SetPoint;
		msg[1] = Motor[chan].SetPointLowerBits;
	}

	set_address(BLCTRL_BASE_ADDR + (chan + addrTranslator[chan]));

	if (Motor[chan].Version == BLCTRL_OLD) {
		ret = transfer(&msg[0], 1, nullptr, 0);

	} else {
		ret = transfer(&msg[0], 2, nullptr, 0);
	}

	return ret;
}


int
MK::control_callback(uintptr_t handle,
		     uint8_t control_group,
		     uint8_t control_index,
		     float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls->control[control_index];
	return 0;
}

int
MK::ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret;

	// XXX disabled, confusing users

	/* try it as a GPIO ioctl first */
	ret = gpio_ioctl(filp, cmd, arg);

	if (ret != -ENOTTY)
		return ret;

	/* if we are in valid PWM mode, try it as a PWM ioctl as well */
	/*
	switch (_mode) {
	case MODE_2PWM:
	case MODE_4PWM:
	case MODE_6PWM:
		ret = pwm_ioctl(filp, cmd, arg);
		break;

	default:
		debug("not in a PWM mode");
		break;
	}
	*/
	ret = pwm_ioctl(filp, cmd, arg);

	/* if nobody wants it, let CDev have it */
	if (ret == -ENOTTY)
		ret = CDev::ioctl(filp, cmd, arg);

	return ret;
}

int
MK::pwm_ioctl(file *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	lock();

	switch (cmd) {
	case PWM_SERVO_ARM:
		mk_servo_arm(true);
		break;

	case PWM_SERVO_SET_ARM_OK:
	case PWM_SERVO_CLEAR_ARM_OK:
		// these are no-ops, as no safety switch
		break;

	case PWM_SERVO_DISARM:
		mk_servo_arm(false);
		break;

	case PWM_SERVO_SET_UPDATE_RATE:
		ret = OK;
		break;

	case PWM_SERVO_SET_SELECT_UPDATE_RATE:
		ret = OK;
		break;


	case PWM_SERVO_SET(0) ... PWM_SERVO_SET(_max_actuators - 1):
		if (arg < 2150) {
			Motor[cmd - PWM_SERVO_SET(0)].RawPwmValue = (unsigned short)arg;
			mk_servo_set(cmd - PWM_SERVO_SET(0), scaling(arg, 1010, 2100, 0, 2047));
		} else {
			ret = -EINVAL;
		}

		break;

	case PWM_SERVO_GET(0) ... PWM_SERVO_GET(_max_actuators - 1):
		/* copy the current output value from the channel */
		*(servo_position_t *)arg = Motor[cmd - PWM_SERVO_SET(0)].RawPwmValue;

		break;

	case PWM_SERVO_GET_RATEGROUP(0):
	case PWM_SERVO_GET_RATEGROUP(1):
	case PWM_SERVO_GET_RATEGROUP(2):
	case PWM_SERVO_GET_RATEGROUP(3):
		//*(uint32_t *)arg = up_pwm_servo_get_rate_group(cmd - PWM_SERVO_GET_RATEGROUP(0));
		break;

	case PWM_SERVO_GET_COUNT:
	case MIXERIOCGETOUTPUTCOUNT:
		*(unsigned *)arg = _num_outputs;
		break;

	case MIXERIOCRESET:
		if (_mixers != nullptr) {
			delete _mixers;
			_mixers = nullptr;
		}

		break;

	case MIXERIOCADDSIMPLE: {
			mixer_simple_s *mixinfo = (mixer_simple_s *)arg;

			SimpleMixer *mixer = new SimpleMixer(control_callback,
							     (uintptr_t)&_controls, mixinfo);

			if (mixer->check()) {
				delete mixer;
				ret = -EINVAL;

			} else {
				if (_mixers == nullptr)
					_mixers = new MixerGroup(control_callback,
								 (uintptr_t)&_controls);

				_mixers->add_mixer(mixer);
			}

			break;
		}

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strnlen(buf, 1024);

			if (_mixers == nullptr)
				_mixers = new MixerGroup(control_callback, (uintptr_t)&_controls);

			if (_mixers == nullptr) {
				ret = -ENOMEM;

			} else {

				ret = _mixers->load_from_buf(buf, buflen);

				if (ret != 0) {
					debug("mixer load failed with %d", ret);
					delete _mixers;
					_mixers = nullptr;
					ret = -EINVAL;
				}
			}

			break;
		}

	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	return ret;
}

/*
  this implements PWM output via a write() method, for compatibility
  with px4io
 */
ssize_t
MK::write(file *filp, const char *buffer, size_t len)
{
	unsigned count = len / 2;
	uint16_t values[8];

	if (count > _num_outputs) {
		// we only have 8 I2C outputs in the driver
		count = _num_outputs;
	}

	// allow for misaligned values
	memcpy(values, buffer, count * 2);

	for (uint8_t i = 0; i < count; i++) {
		Motor[i].RawPwmValue = (unsigned short)values[i];
		mk_servo_set(i, scaling(values[i], 1010, 2100, 0, 2047));
	}

	return count * 2;
}

void
MK::gpio_reset(void)
{
	/*
	 * Setup default GPIO config - all pins as GPIOs, GPIO driver chip
	 * to input mode.
	 */
	for (unsigned i = 0; i < _ngpio; i++)
		stm32_configgpio(_gpio_tab[i].input);

	stm32_gpiowrite(GPIO_GPIO_DIR, 0);
	stm32_configgpio(GPIO_GPIO_DIR);
}

void
MK::gpio_set_function(uint32_t gpios, int function)
{
	/*
	 * GPIOs 0 and 1 must have the same direction as they are buffered
	 * by a shared 2-port driver.  Any attempt to set either sets both.
	 */
	if (gpios & 3) {
		gpios |= 3;

		/* flip the buffer to output mode if required */
		if (GPIO_SET_OUTPUT == function)
			stm32_gpiowrite(GPIO_GPIO_DIR, 1);
	}

	/* configure selected GPIOs as required */
	for (unsigned i = 0; i < _ngpio; i++) {
		if (gpios & (1 << i)) {
			switch (function) {
			case GPIO_SET_INPUT:
				stm32_configgpio(_gpio_tab[i].input);
				break;

			case GPIO_SET_OUTPUT:
				stm32_configgpio(_gpio_tab[i].output);
				break;

			case GPIO_SET_ALT_1:
				if (_gpio_tab[i].alt != 0)
					stm32_configgpio(_gpio_tab[i].alt);

				break;
			}
		}
	}

	/* flip buffer to input mode if required */
	if ((GPIO_SET_INPUT == function) && (gpios & 3))
		stm32_gpiowrite(GPIO_GPIO_DIR, 0);
}

void
MK::gpio_write(uint32_t gpios, int function)
{
	int value = (function == GPIO_SET) ? 1 : 0;

	for (unsigned i = 0; i < _ngpio; i++)
		if (gpios & (1 << i))
			stm32_gpiowrite(_gpio_tab[i].output, value);
}

uint32_t
MK::gpio_read(void)
{
	uint32_t bits = 0;

	for (unsigned i = 0; i < _ngpio; i++)
		if (stm32_gpioread(_gpio_tab[i].input))
			bits |= (1 << i);

	return bits;
}

int
MK::gpio_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int	ret = OK;

	lock();

	switch (cmd) {

	case GPIO_RESET:
		gpio_reset();
		break;

	case GPIO_SET_OUTPUT:
	case GPIO_SET_INPUT:
	case GPIO_SET_ALT_1:
		gpio_set_function(arg, cmd);
		break;

	case GPIO_SET_ALT_2:
	case GPIO_SET_ALT_3:
	case GPIO_SET_ALT_4:
		ret = -EINVAL;
		break;

	case GPIO_SET:
	case GPIO_CLEAR:
		gpio_write(arg, cmd);
		break;

	case GPIO_GET:
		*(uint32_t *)arg = gpio_read();
		break;

	default:
		ret = -ENOTTY;
	}

	unlock();

	return ret;
}

namespace
{

enum PortMode {
	PORT_MODE_UNSET = 0,
	PORT_FULL_GPIO,
	PORT_FULL_SERIAL,
	PORT_FULL_PWM,
	PORT_GPIO_AND_SERIAL,
	PORT_PWM_AND_SERIAL,
	PORT_PWM_AND_GPIO,
};

enum MappingMode {
	MAPPING_MK = 0,
	MAPPING_PX4,
};

enum FrameType {
	FRAME_PLUS = 0,
	FRAME_X,
};

PortMode g_port_mode;

int
mk_new_mode(PortMode new_mode, int update_rate, int motorcount, bool motortest, int px4mode, int frametype, bool overrideSecurityChecks)
{
	uint32_t gpio_bits;
	int shouldStop = 0;
	MK::Mode servo_mode;

	/* reset to all-inputs */
	g_mk->ioctl(0, GPIO_RESET, 0);

	gpio_bits = 0;
	servo_mode = MK::MODE_NONE;

	/* native PX4 addressing) */
	g_mk->set_px4mode(px4mode);

	/* set frametype (geometry) */
	g_mk->set_frametype(frametype);

	/* motortest if enabled */
	g_mk->set_motor_test(motortest);

	/* ovveride security checks if enabled */
	g_mk->set_overrideSecurityChecks(overrideSecurityChecks);


	/* count used motors */
	do {
		if (g_mk->mk_check_for_blctrl(8, false) != 0) {
			shouldStop = 4;

		} else {
			shouldStop++;
		}

		sleep(1);
	} while (shouldStop < 3);

	g_mk->set_motor_count(g_mk->mk_check_for_blctrl(8, true));

	/* (re)set the PWM output mode */
	g_mk->set_mode(servo_mode);


	if ((servo_mode != MK::MODE_NONE) && (update_rate != 0))
		g_mk->set_pwm_rate(update_rate);

	return OK;
}

int
mk_start(unsigned bus, unsigned motors)
{
	int ret = OK;

	if (g_mk == nullptr) {

		g_mk = new MK(bus);

		if (g_mk == nullptr) {
			ret = -ENOMEM;

		} else {
			ret = g_mk->init(motors);

			if (ret != OK) {
				delete g_mk;
				g_mk = nullptr;
			}
		}
	}

	return ret;
}


} // namespace

extern "C" __EXPORT int mkblctrl_main(int argc, char *argv[]);

int
mkblctrl_main(int argc, char *argv[])
{
	PortMode port_mode = PORT_FULL_PWM;
	int pwm_update_rate_in_hz = UPDATE_RATE;
	int motorcount = 8;
	int bus = 1;
	int px4mode = MAPPING_PX4;
	int frametype = FRAME_PLUS;	// + plus is default
	bool motortest = false;
	bool overrideSecurityChecks = false;
	bool showHelp = false;
	bool newMode = false;

	/*
	 * optional parameters
	 */
	for (int i = 1; i < argc; i++) {

		/* look for the optional i2c bus parameter */
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--bus") == 0) {
			if (argc > i + 1) {
				bus = atoi(argv[i + 1]);
				newMode = true;

			} else {
				errx(1, "missing argument for i2c bus (-b)");
				return 1;
			}
		}

		/* look for the optional frame parameter */
		if (strcmp(argv[i], "-mkmode") == 0 || strcmp(argv[i], "--mkmode") == 0) {
			if (argc > i + 1) {
				if (strcmp(argv[i + 1], "+") == 0 || strcmp(argv[i + 1], "x") == 0 || strcmp(argv[i + 1], "X") == 0) {
					px4mode = MAPPING_MK;
					newMode = true;

					if (strcmp(argv[i + 1], "+") == 0) {
						frametype = FRAME_PLUS;

					} else {
						frametype = FRAME_X;
					}

				} else {
					errx(1, "only + or x for frametype supported !");
				}

			} else {
				errx(1, "missing argument for mkmode (-mkmode)");
				return 1;
			}
		}

		/* look for the optional test parameter */
		if (strcmp(argv[i], "-t") == 0) {
			motortest = true;
			newMode = true;
		}

		/* look for the optional -h --help parameter */
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			showHelp = true;
		}

		/* look for the optional --override-security-checks  parameter */
		if (strcmp(argv[i], "--override-security-checks") == 0) {
			overrideSecurityChecks = true;
			newMode = true;
		}

	}

	if (showHelp) {
		fprintf(stderr, "mkblctrl: help:\n");
		fprintf(stderr, "  [-mkmode frame{+/x}] [-b i2c_bus_number] [-t motortest] [--override-security-checks] [-h / --help]\n\n");
		fprintf(stderr, "\t -mkmode frame {+/x} \t\t Type of frame, if Mikrokopter motor order is used.\n");
		fprintf(stderr, "\t -b i2c_bus_number \t\t Set the i2c bus where the ESCs are connected to (default 1).\n");
		fprintf(stderr, "\t -t motortest \t\t\t Spin up once every motor in order of motoraddress. (DANGER !!!)\n");
		fprintf(stderr, "\t --override-security-checks \t\t Disable all security checks (arming and number of ESCs). Used to test single Motors etc. (DANGER !!!)\n");
		exit(1);
	}


	if (g_mk == nullptr) {
		if (mk_start(bus, motorcount) != OK) {
			errx(1, "failed to start the MK-BLCtrl driver");

		} else {
			newMode = true;
		}
	}


	/* parameter set ? */
	if (newMode) {
		/* switch parameter */
		return mk_new_mode(port_mode, pwm_update_rate_in_hz, motorcount, motortest, px4mode, frametype, overrideSecurityChecks);
	}

	/* test, etc. here g*/

	exit(1);
}
