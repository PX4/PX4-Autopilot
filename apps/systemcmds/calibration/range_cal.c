/****************************************************************************
 * range_cal.c
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
#include "calibration.h"

/****************************************************************************
 * Defines
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
//Store the values here before writing them to global_data_rc_channels
uint16_t max_values[NR_CHANNELS];
uint16_t min_values[NR_CHANNELS];
uint16_t mid_values[NR_CHANNELS];


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**This sets the middle values
 */
uint8_t set_mid(void)
{
	if (0 == global_data_trylock(&global_data_rc_channels->access_conf)) {
		uint8_t i;

		for (i = 0; i < NR_CHANNELS; i++) {
			if (i == global_data_rc_channels->function[ROLL] ||
			    i == global_data_rc_channels->function[YAW] ||
			    i == global_data_rc_channels->function[PITCH]) {

				mid_values[i] = global_data_rc_channels->chan[i].raw;

			} else {
				mid_values[i] = (max_values[i] + min_values[i]) / 2;
			}

		}

		global_data_unlock(&global_data_rc_channels->access_conf);
		return 0;

	} else
		return -1;
}

/**This gets all current values and writes them to min or max
 */
uint8_t get_value(void)
{
	if (0 == global_data_trylock(&global_data_rc_channels->access_conf)) {
		uint8_t i;

		for (i = 0; i < NR_CHANNELS; i++) {
			//see if the value is bigger or smaller than 1500 (roughly neutral)
			//and write to the appropriate array
			if (global_data_rc_channels->chan[i].raw != 0 &&
			    global_data_rc_channels->chan[i].raw < 1500) {
				min_values[i] = global_data_rc_channels->chan[i].raw;

			} else if (global_data_rc_channels->chan[i].raw != 0 &&
				   global_data_rc_channels->chan[i].raw > 1500) {
				max_values[i] = global_data_rc_channels->chan[i].raw;

			} else {
				max_values[i] = 0;
				min_values[i] = 0;
			}
		}

		global_data_unlock(&global_data_rc_channels->access_conf);
		return 0;

	} else
		return -1;
}


void write_data(void)
{
	//  global_data_lock(&global_data_rc_channels->access_conf);
	//  uint8_t i;
	//  for(i=0; i < NR_CHANNELS; i++){
	//    //Write the data to global_data_rc_channels (if not 0)
	//    if(mid_values[i]!=0 && min_values[i]!=0 && max_values[i]!=0){
	//      global_data_rc_channels->chan[i].scaling_factor =
	//        10000/((max_values[i] - min_values[i])/2);
	//
	//      global_data_rc_channels->chan[i].mid = mid_values[i];
	//    }
	//
	//    printf("Channel %i\t Function %s \t\t Min %i\t\t Max %i\t\t Scaling Factor: %i\t Middle Value %i\n",
	//        i,
	//        global_data_rc_channels->function_name[global_data_rc_channels->function[i]],
	//        min_values[i], max_values[i],
	//        global_data_rc_channels->chan[i].scaling_factor,
	//        global_data_rc_channels->chan[i].mid);
	//  }
	//  global_data_unlock(&global_data_rc_channels->access_conf);

	//Write to the Parameter storage

	global_data_parameter_storage->pm.param_values[PARAM_RC1_MIN] = min_values[0];
	global_data_parameter_storage->pm.param_values[PARAM_RC2_MIN] = min_values[1];
	global_data_parameter_storage->pm.param_values[PARAM_RC3_MIN] = min_values[2];
	global_data_parameter_storage->pm.param_values[PARAM_RC4_MIN] = min_values[3];
	global_data_parameter_storage->pm.param_values[PARAM_RC5_MIN] = min_values[4];
	global_data_parameter_storage->pm.param_values[PARAM_RC6_MIN] = min_values[5];
	global_data_parameter_storage->pm.param_values[PARAM_RC7_MIN] = min_values[6];
	global_data_parameter_storage->pm.param_values[PARAM_RC8_MIN] = min_values[7];


	global_data_parameter_storage->pm.param_values[PARAM_RC1_MAX] = max_values[0];
	global_data_parameter_storage->pm.param_values[PARAM_RC2_MAX] = max_values[1];
	global_data_parameter_storage->pm.param_values[PARAM_RC3_MAX] = max_values[2];
	global_data_parameter_storage->pm.param_values[PARAM_RC4_MAX] = max_values[3];
	global_data_parameter_storage->pm.param_values[PARAM_RC5_MAX] = max_values[4];
	global_data_parameter_storage->pm.param_values[PARAM_RC6_MAX] = max_values[5];
	global_data_parameter_storage->pm.param_values[PARAM_RC7_MAX] = max_values[6];
	global_data_parameter_storage->pm.param_values[PARAM_RC8_MAX] = max_values[7];


	global_data_parameter_storage->pm.param_values[PARAM_RC1_TRIM] = mid_values[0];
	global_data_parameter_storage->pm.param_values[PARAM_RC2_TRIM] = mid_values[1];
	global_data_parameter_storage->pm.param_values[PARAM_RC3_TRIM] = mid_values[2];
	global_data_parameter_storage->pm.param_values[PARAM_RC4_TRIM] = mid_values[3];
	global_data_parameter_storage->pm.param_values[PARAM_RC5_TRIM] = mid_values[4];
	global_data_parameter_storage->pm.param_values[PARAM_RC6_TRIM] = mid_values[5];
	global_data_parameter_storage->pm.param_values[PARAM_RC7_TRIM] = mid_values[6];
	global_data_parameter_storage->pm.param_values[PARAM_RC8_TRIM] = mid_values[7];

	usleep(3e6);
	uint8_t i;

	for (i = 0; i < NR_CHANNELS; i++) {
		printf("Channel %i:\t\t Min %i\t\t Max %i\t\t Scaling Factor: %i\t Middle Value %i\n",
		       i,
		       min_values[i], max_values[i],
		       global_data_rc_channels->chan[i].scaling_factor,
		       global_data_rc_channels->chan[i].mid);
	}


}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
int range_cal(int argc, char *argv[])
{

	printf("The range calibration routine assumes you already did the channel\n");
	printf("assignment\n");
	printf("This routine chooses the minimum, maximum and middle\n");
	printf("value for each channel separatly. The ROLL, PITCH and YAW function\n");
	printf("get their middle value from the RC direct, for the rest it is\n");
	printf("calculated out of the min and max values.\n");
	press_enter();

	printf("Hold both sticks in lower left corner and continue\n ");
	press_enter();
	usleep(500000);

	while (get_value());

	printf("Hold both sticks in upper right corner and continue\n");
	press_enter();
	usleep(500000);

	while (get_value());

	printf("Set the trim to 0 and leave the sticks in the neutral position\n");
	press_enter();

	//Loop until successfull
	while (set_mid());

	//write the values to global_data_rc_channels
	write_data();

	return 0;

}

