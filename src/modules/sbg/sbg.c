/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file sbg.c
 * Application for use of SBG Systems IMU connected on an UART port of PX4.
 * Tested with IG-500N device
 * 
 * @author Emmanuel Roussel <rousselmanu@gmail.com>
 */

#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

#include <px4_config.h>
#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude.h>

#include "sbgcom/sbgCom.h"

#define UART_PORT "/dev/ttyS2"	/**< UART port of SBG device (default: TELEM2 port) */
#define UART_BAUD (230400)		/**< Baudrate */
//#define VERBOSE					/**< enable verbose console output (debug) */

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;			/**< daemon status flag */
static int daemon_task;						/**< Handle of daemon task / thread */
SbgOutput output;							/**< structure for IMU data, see "sbgcom/protocol/protocolOutput.h" */
struct vehicle_attitude_s att;				/**< attitude data structure */
orb_advert_t att_pub;						/**< attitude descriptor */
uint32_t sbgOutputMask;						/**< Output mask for received UART frame. See "sbgcom/protocol/protocolOutput.h". */

/**
 * daemon management function.
 */
__EXPORT int sbg_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int sbg_daemon_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/**
 * Callback function called when IMU frame is parsed and ready
 */
void userHandler(SbgProtocolHandle handle, SbgOutput *pOutput, void *pMainOutput_);

static void usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}
	
	warnx("usage: sbg {start|stop|status} [-output_mask]\n");
	warnx("try: sbg start -dpviqesa\n\n");
}

void userHandler(SbgProtocolHandle handle, SbgOutput *pOutput, void *pMainOutput_)
{
	SbgOutput *pMainOutput = (SbgOutput*)pMainOutput_;
	int i;
	/* The structure is simply copied into the ig500Thread function.
	We are in the same process as ig500Thread, so no memory will be corrupted */
	memcpy(pMainOutput,pOutput,sizeof(SbgOutput));
	
	if(sbgOutputMask&SBG_OUTPUT_EULER){	/* update Euler andles */
		att.roll = (float)(pOutput->stateEuler[0]);
		att.pitch = (float)(pOutput->stateEuler[1]);
		att.yaw = (float)(pOutput->stateEuler[2]);
	}
	if(sbgOutputMask&SBG_OUTPUT_QUATERNION){
		for(i=0; i<4; i++) att.q[i] = (float)(pOutput->stateQuat[i]);	/* update quaternion */
		att.q_valid = true;
	}
	if(sbgOutputMask&SBG_OUTPUT_MATRIX){
		for(i=0; i<9; i++) att.R[i] = (float)(pOutput->stateMatrix[i]);	/* update rotation matrix */
		att.R_valid = true;
	}
	orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
	
	#ifdef VERBOSE
	/* Print Euler angles from SBG IMU (roll, pitch, yaw), in deg. */
	printf("%3.2f\t%3.2f\t%3.2f\n",	(double)(pOutput->stateEuler[0])*57.2958, (double)(pOutput->stateEuler[1])*57.2958, (double)(pOutput->stateEuler[2])*57.2958);
	#endif
}

int sbg_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("sbg daemon already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("sbg_daemon",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_DEFAULT,
					 5000,
					 sbg_daemon_thread_main,
					 (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");
		} else {
			warnx("\tnot started\n");
		}
		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int sbg_daemon_thread_main(int argc, char *argv[])
{
	sbgOutputMask=SBG_OUTPUT_TIME_SINCE_RESET; 	/* Enable the time since the device is ON in milliseconds */

	warnx("Starting\n");

	int opt;
	while ((opt = getopt(argc, argv, "dpviqesan")) != -1) {
		switch (opt) {
			case 'd': sbgOutputMask |= SBG_OUTPUT_DEVICE_STATUS; break;	/* Enable the Device status output */

			case 'p': sbgOutputMask |= SBG_OUTPUT_GPS_POSITION; break;	/* Enable the raw GPS position data */
			case 'v': sbgOutputMask |= SBG_OUTPUT_GPS_NAVIGATION; break;	/* Enable the raw GPS navigation data such as velocity, heading */
			case 'i': sbgOutputMask |= SBG_OUTPUT_GPS_INFO; break;		/* Enable the raw GPS information data such as number of used sattelites */

			case 'q': sbgOutputMask |= SBG_OUTPUT_QUATERNION; break;	/* Enable quaternion attitude output */
			case 'e': sbgOutputMask |= SBG_OUTPUT_EULER; break;			/* Enable euler angles attitude output */

			case 's': sbgOutputMask |= SBG_OUTPUT_ACCELEROMETERS|SBG_OUTPUT_MAGNETOMETERS|SBG_OUTPUT_GYROSCOPES; break;	/* Enable calibrated sensors output */
			case 'a': sbgOutputMask |= SBG_OUTPUT_BARO_PRESSURE|SBG_OUTPUT_BARO_ALTITUDE; break;	/* Enable the raw pressure output in pascal, and the barometric altitude referenced on a user defined reference pressure */

			case 'n': sbgOutputMask |= SBG_OUTPUT_POSITION|SBG_OUTPUT_VELOCITY; break;		/* Enable the Kalman enhanced 3d position and velocity in North East Down frame */
			
			default:
				usage("Option warning");
				return 1;
		}
	}
//	verbose=!strcmp(argv[0], "-v");
	
	/* advertise attitude topic */
	memset(&att, 0, sizeof(att));
	att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

	/* Init communications with the SBG device */
	int try_cnt=0;
	SbgProtocolHandle protocolHandle;
	while ((try_cnt<5) && (sbgComInit(UART_PORT, UART_BAUD, &protocolHandle) != SBG_NO_ERROR))	{
		try_cnt++;
		usleep(30000);
		warnx("Trying again to init com...\n");
	}
	if(try_cnt>=5){
		err(1, "Unable to open IG-500 device\n");
		return 1;
	}
	warnx("IG-500 found\n");
	thread_running = true;
	usleep(100000);	/* Wait for the IMU to be ready */
	
	/* Define the default output mask for continuous mode (This setting is not saved in flash non volatile memory) */
	try_cnt=0;
	while ((try_cnt<5) && (sbgSetDefaultOutputMask(protocolHandle, sbgOutputMask) != SBG_NO_ERROR)){
		try_cnt++;
		usleep(30000);
		warnx("Trying again to set mask...\n");
	}
	if(try_cnt>=5){
		err(1, "Unable to set default output mask for IG-500 device\n");
		return 1;
	}
	warnx("Default output mask set\n");

	/* If 0 is passed as an argument, the current pressure is directly considered as the reference pressure. */
	sbgSetReferencePressure(protocolHandle, 0); 

	/* Enable continuous mode: SBG sends frames at 100 Hz. (Setting is not saved in flash memory) */
	memset(&output, 0, sizeof(output));		/* Initialize data */
	sbgSetContinuousModeCallback(protocolHandle, userHandler, &output); /* Initialize callback function */ 
	if (sbgSetContinuousMode(protocolHandle, SBG_CONTINUOUS_MODE_ENABLE, 1) != SBG_NO_ERROR){
		err(1, "Unable to enable Continuous mode\n");
		return 1;
	}
	warnx("Continuous mode ON\n");
	usleep(10000);

	sbgProtocolContinuousModeHandle(protocolHandle); /* First call, to print data */
	printf("Initial Euler angles:\n");
	printf("%3.2f\t%3.2f\t%3.2f\n",	(double)(output.stateEuler[0])*57.2958, (double)(output.stateEuler[1])*57.2958, (double)(output.stateEuler[2])*57.2958);

	/* ---------- MAIN LOOP ------------ */
	while(!thread_should_exit){
		sbgProtocolContinuousModeHandle(protocolHandle);
		usleep(1000);
	}
	/* --------------------------------- */

	sbgProtocolClose(protocolHandle);

	warnx("Exiting.\n");
	thread_running = false;
	return 0;
}
