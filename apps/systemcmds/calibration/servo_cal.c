/****************************************************************************
 * servo_cal.c
 *
 *   Copyright (C) 2012 Nils Wenzler. All rights reserved.
 *   Authors: Nils Wenzler <wenzlern@ethz.ch>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <arch/board/drv_pwm_servo.h>
#include <fcntl.h>
#include "calibration.h"

/****************************************************************************
 * Defines
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
//Store the values here before writing them to global_data_rc_channels
uint16_t max_values_servo[PWM_SERVO_MAX_CHANNELS];
uint16_t min_values_servo[PWM_SERVO_MAX_CHANNELS];
uint16_t mid_values_servo[PWM_SERVO_MAX_CHANNELS];

// Servo loop thread

pthread_t servo_calib_thread;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**This sets the middle values
 */
uint8_t set_mid_s(void)
{
	if (0 == global_data_trylock(&global_data_rc_channels->access_conf)) {
		uint8_t i;

		for (i = 0; i < PWM_SERVO_MAX_CHANNELS; i++) {
			if (i == global_data_rc_channels->function[ROLL] ||
			    i == global_data_rc_channels->function[YAW] ||
			    i == global_data_rc_channels->function[PITCH]) {

				mid_values_servo[i] = global_data_rc_channels->chan[i].raw;

			} else {
				mid_values_servo[i] = (max_values_servo[i] + min_values_servo[i]) / 2;
			}

		}

		global_data_unlock(&global_data_rc_channels->access_conf);
		return 0;

	} else
		return -1;
}

/**This gets all current values and writes them to min or max
 */
uint8_t get_value_s(void)
{
	if (0 == global_data_trylock(&global_data_rc_channels->access_conf)) {
		uint8_t i;

		for (i = 0; i < PWM_SERVO_MAX_CHANNELS; i++) {
			//see if the value is bigger or smaller than 1500 (roughly neutral)
			//and write to the appropriate array
			if (global_data_rc_channels->chan[i].raw != 0 &&
			    global_data_rc_channels->chan[i].raw < 1500) {
				min_values_servo[i] = global_data_rc_channels->chan[i].raw;

			} else if (global_data_rc_channels->chan[i].raw != 0 &&
				   global_data_rc_channels->chan[i].raw > 1500) {
				max_values_servo[i] = global_data_rc_channels->chan[i].raw;

			} else {
				max_values_servo[i] = 0;
				min_values_servo[i] = 0;
			}
		}

		global_data_unlock(&global_data_rc_channels->access_conf);
		return 0;

	} else
		return -1;
}


void write_data_s(void)
{
	//  global_data_lock(&global_data_rc_channels->access_conf);
	//  uint8_t i;
	//  for(i=0; i < NR_CHANNELS; i++){
	//    //Write the data to global_data_rc_channels (if not 0)
	//    if(mid_values_servo[i]!=0 && min_values_servo[i]!=0 && max_values_servo[i]!=0){
	//      global_data_rc_channels->chan[i].scaling_factor =
	//        10000/((max_values_servo[i] - min_values_servo[i])/2);
	//
	//      global_data_rc_channels->chan[i].mid = mid_values_servo[i];
	//    }
	//
	//    printf("Channel %i\t Function %s \t\t Min %i\t\t Max %i\t\t Scaling Factor: %i\t Middle Value %i\n",
	//        i,
	//        global_data_rc_channels->function_name[global_data_rc_channels->function[i]],
	//        min_values_servo[i], max_values_servo[i],
	//        global_data_rc_channels->chan[i].scaling_factor,
	//        global_data_rc_channels->chan[i].mid);
	//  }
	//  global_data_unlock(&global_data_rc_channels->access_conf);

	//Write to the Parameter storage



	global_data_lock(&global_data_parameter_storage->access_conf);

	global_data_parameter_storage->pm.param_values[PARAM_SERVO1_MIN] = min_values_servo[0];
	global_data_parameter_storage->pm.param_values[PARAM_SERVO2_MIN] = min_values_servo[1];
	global_data_parameter_storage->pm.param_values[PARAM_SERVO3_MIN] = min_values_servo[2];
	global_data_parameter_storage->pm.param_values[PARAM_SERVO4_MIN] = min_values_servo[3];

	global_data_parameter_storage->pm.param_values[PARAM_SERVO1_MAX] = max_values_servo[0];
	global_data_parameter_storage->pm.param_values[PARAM_SERVO2_MAX] = max_values_servo[1];
	global_data_parameter_storage->pm.param_values[PARAM_SERVO3_MAX] = max_values_servo[2];
	global_data_parameter_storage->pm.param_values[PARAM_SERVO4_MAX] = max_values_servo[3];

	global_data_parameter_storage->pm.param_values[PARAM_SERVO1_TRIM] = mid_values_servo[0];
	global_data_parameter_storage->pm.param_values[PARAM_SERVO2_TRIM] = mid_values_servo[1];
	global_data_parameter_storage->pm.param_values[PARAM_SERVO3_TRIM] = mid_values_servo[2];
	global_data_parameter_storage->pm.param_values[PARAM_SERVO4_TRIM] = mid_values_servo[3];

	global_data_unlock(&global_data_parameter_storage->access_conf);

	usleep(3e6);
	uint8_t i;

	for (i = 0; i < NR_CHANNELS; i++) {
		printf("Channel %i:\t\t Min %i\t\t Max %i\t\t Scaling Factor: %i\t Middle Value %i\n",
		       i,
		       min_values_servo[i], max_values_servo[i],
		       global_data_rc_channels->chan[i].scaling_factor,
		       global_data_rc_channels->chan[i].mid);
	}

}

static void *servo_loop(void *arg)
{

	// Set thread name
	prctl(1, "calibration servo", getpid());

	// initialize servos
	int fd;
	servo_position_t data[PWM_SERVO_MAX_CHANNELS];

	fd = open("/dev/pwm_servo", O_RDWR);

	if (fd < 0) {
		printf("failed opening /dev/pwm_servo\n");
	}

	ioctl(fd, PWM_SERVO_ARM, 0);

	while (1) {
		int i;

		for (i = 0; i < 4; i++) {
			data[i] = global_data_rc_channels->chan[global_data_rc_channels->function[THROTTLE]].raw;
		}

		int result = write(fd, &data, sizeof(data));

		if (result != sizeof(data)) {
			printf("failed bulk-reading channel values\n");
		}

		// 5Hz loop
		usleep(200000);
	}

	return NULL;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/
int servo_cal(int argc, char *argv[])
{
	//	pthread_attr_t servo_loop_attr;
	//	pthread_attr_init(&servo_loop_attr);
	//	pthread_attr_setstacksize(&servo_loop_attr, 1024);
	pthread_create(&servo_calib_thread, NULL, servo_loop, NULL);
	pthread_join(servo_calib_thread, NULL);

	printf("The servo calibration routine assumes you already did the channel\n");
	printf("assignment with 'calibration channels'\n");
	printf("This routine chooses the minimum, maximum and middle\n");
	printf("value for each channel separately. The ROLL, PITCH and YAW function\n");
	printf("get their middle value from the RC direct, for the rest it is\n");
	printf("calculated out of the min and max values.\n");
	press_enter();

	printf("Hold both sticks in lower left corner and continue\n ");
	press_enter();
	usleep(500000);

	while (get_value_s());

	printf("Hold both sticks in upper right corner and continue\n");
	press_enter();
	usleep(500000);

	while (get_value_s());

	printf("Set the trim to 0 and leave the sticks in the neutral position\n");
	press_enter();

	//Loop until successfull
	while (set_mid_s());

	//write the values to global_data_rc_channels
	write_data_s();

	return 0;

}

