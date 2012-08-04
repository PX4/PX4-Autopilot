/****************************************************************************
 * channels_cal.c
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
#include <string.h>
#include <stdlib.h>
#include "calibration.h"

/****************************************************************************
 * Defines
 ****************************************************************************/
#define ABS(a)    (((a) < 0) ? -(a) : (a))
#define MIN(a,b)  (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

/****************************************************************************
 * Private Data
 ****************************************************************************/
//Store the values here before writing them to global_data_rc_channels
uint16_t old_values[NR_CHANNELS];
uint16_t cur_values[NR_CHANNELS];
uint8_t  function_map[NR_CHANNELS];
char names[12][9];



/****************************************************************************
 * Private Functions
 ****************************************************************************/
void press_enter_2(void)
{
	int c;
	printf("Press CTRL+ENTER to continue... \n");
	fflush(stdout);

	do c = getchar(); while ((c != '\n') && (c != EOF));
}

/**This gets all current values and writes them to min or max
 */
uint8_t get_val(uint16_t *buffer)
{
	if (0 == global_data_trylock(&global_data_rc_channels->access_conf)) {
		uint8_t i;

		for (i = 0; i < NR_CHANNELS; i++) {
			printf("Channel: %i\t RAW Value: %i: \n", i, global_data_rc_channels->chan[i].raw);
			buffer[i] = global_data_rc_channels->chan[i].raw;
		}

		global_data_unlock(&global_data_rc_channels->access_conf);
		return 0;

	} else
		return -1;
}

void set_channel(void)
{
	static uint8_t j = 0;
	uint8_t i;

	for (i = 0; i < NR_CHANNELS; i++) {
		if (ABS(old_values - cur_values) > 50) {
			function_map[j] = i;
			strcpy(names[i], global_data_rc_channels->function_names[j]);
			j++;
		}
	}
}

void write_dat(void)
{
	global_data_lock(&global_data_rc_channels->access_conf);
	uint8_t i;

	for (i = 0; i < NR_CHANNELS; i++) {
		global_data_rc_channels->function[i] = function_map[i];
		strcpy(global_data_rc_channels->chan[i].name, names[i]);

		printf("Channel %i\t Function %s\n", i,
		       global_data_rc_channels->chan[i].name);
	}

	global_data_unlock(&global_data_rc_channels->access_conf);
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/
int channels_cal(int argc, char *argv[])
{

	printf("This routine maps the input functions on the remote control\n");
	printf("to the corresponding function (Throttle, Roll,..)\n");
	printf("Always move the stick all the way\n");
	press_enter_2();

	printf("Pull the THROTTLE stick down\n");
	press_enter_2();

	while (get_val(old_values));

	printf("Move the THROTTLE stick up\n ");
	press_enter_2();

	while (get_val(cur_values));

	set_channel();

	printf("Pull the PITCH stick down\n");
	press_enter_2();

	while (get_val(old_values));

	printf("Move the PITCH stick up\n ");
	press_enter_2();

	while (get_val(cur_values));

	set_channel();

	printf("Put the ROLL stick to the left\n");
	press_enter_2();

	while (get_val(old_values));

	printf("Put the ROLL stick to the right\n ");
	press_enter_2();

	while (get_val(cur_values));

	set_channel();

	printf("Put the YAW stick to the left\n");
	press_enter_2();

	while (get_val(old_values));

	printf("Put the YAW stick to the right\n ");
	press_enter_2();

	while (get_val(cur_values));

	set_channel();

	uint8_t k;

	for (k = 5; k < NR_CHANNELS; k++) {
		function_map[k] = k;
		strcpy(names[k], global_data_rc_channels->function_names[k]);
	}

//write the values to global_data_rc_channels
	write_dat();

	return 0;

}

