/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <px4_platform_common/defines.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
// #include <lib/drivers/device/Device.hpp>
// #include "TFMINI.hpp"

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>

// #include "tests_main.h"

#include <math.h>
#include <float.h>
#include <fcntl.h>
#include <time.h>

// char attitude[11];
//  = "50000_50000";
float encoder_roll_feedback     = 0.0;
float encoder_pitch_feedback    = 0.0;

#include <px4_platform_common/log.h>

__EXPORT int cam_device_read_main(int argc, char *argv[]);

int cam_device_read_main(int argc, char *argv[])
{
// PX4_INFO("Hello Sky!");

// 	/* subscribe to vehicle_acceleration topic */
// 	int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_acceleration));
// 	/* limit the update rate to 5 Hz */
// 	orb_set_interval(sensor_sub_fd, 200);

// 	/* advertise attitude topic */
// 	struct vehicle_attitude_s att;
// 	memset(&att, 0, sizeof(att));
// 	orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

// 	/* one could wait for multiple topics with this technique, just using one here */
// 	px4_pollfd_struct_t fds[] = {
// 		{ .fd = sensor_sub_fd,   .events = POLLIN },
// 		/* there could be more file descriptors here, in the form like:
// 		 * { .fd = other_sub_fd,   .events = POLLIN },
// 		 */
// 	};

// 	int error_counter = 0;

// 	for (int i = 0; i < 5; i++) {
// 		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
// 		int poll_ret = px4_poll(fds, 1, 1000);

// 		/* handle the poll result */
// 		if (poll_ret == 0) {
// 			/* this means none of our providers is giving us data */
// 			PX4_ERR("Got no data within a second");

// 		} else if (poll_ret < 0) {
// 			/* this is seriously bad - should be an emergency */
// 			if (error_counter < 10 || error_counter % 50 == 0) {
// 				/* use a counter to prevent flooding (and slowing us down) */
// 				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
// 			}

// 			error_counter++;

// 		} else {

// 			if (fds[0].revents & POLLIN) {
// 				/* obtained data for the first file descriptor */
// 				struct vehicle_acceleration_s accel;
// 				/* copy sensors raw data into local buffer */
// 				orb_copy(ORB_ID(vehicle_acceleration), sensor_sub_fd, &accel);
// 				PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
// 					 (double)accel.xyz[0],
// 					 (double)accel.xyz[1],
// 					 (double)accel.xyz[2]);

// 				/* set att and publish this information for other apps
// 				 the following does not have any meaning, it's just an example
// 				*/
// 				att.q[0] = accel.xyz[0];
// 				att.q[1] = accel.xyz[1];
// 				att.q[2] = accel.xyz[2];

// 				orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
// 			}

// 			/* there could be more file descriptors here, in the form like:
// 			 * if (fds[1..n].revents & POLLIN) {}
// 			 */
// 		}
// 	}

	// For getting data from uart
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////

	/* input handling */
	char *uart_name = "/dev/ttyS5";

	// if (argc > 1) {
	// uart_name = argv[1];
	// }

	/* assuming NuttShell is on UART1 (/dev/ttyS0) */
	int test_uart = open(uart_name, O_RDWR | O_NOCTTY); //

	if (test_uart < 0) {
		PX4_INFO("ERROR opening UART %s, aborting..\n", uart_name);
	return test_uart;

	} else {
		PX4_INFO("Reading the UART port %s\n", uart_name);
	}

	// PX4_INFO("Step - x1");

	// sete the badrate to match in terminal
	struct termios uart1_config;
	int termios_state = 0;
	if ((termios_state = tcgetattr(test_uart, &uart1_config)) < 0) {
		PX4_INFO("ERROR getting termios config for UART1: %d\n", termios_state);
	}
	if (cfsetispeed(&uart1_config, B57600) < 0 ||
		cfsetospeed(&uart1_config, B57600) < 0) {
		PX4_INFO("ERROR setting termios config for UART1: %d\n", termios_state);
	}
	if ((termios_state = tcsetattr(test_uart, TCSANOW, &uart1_config)) < 0) {
		PX4_INFO("ERROR setting termios config for UART1\n");
	}
	// PX4_INFO("Step - x2");

	fcntl(test_uart, F_SETFL, 0);
	uart1_config.c_cflag = CRTSCTS | CS8 | CLOCAL | CREAD;
	uart1_config.c_iflag = IGNPAR | ICRNL;
	tcflush(test_uart, TCIFLUSH);
	tcsetattr(test_uart, TCSANOW, &uart1_config);

	// PX4_INFO("Step - x3");

  	// int readval = 0;
	char *readbuf = malloc(100);
	// char *readbuf[1];
	//  = malloc(1);
	// char attitude[11];
	// int flag_start = -1;
	// bool receiving_data = true;
    	// char startChar = ',';
    	// char endChar = '/';
    	// int index = 0;

	// while(1){

		// readval =
	// 	read(test_uart, readbuf, 1);
	// 	PX4_INFO("readbuf -> %c\n",*readbuf);
	// }
	int counter = 0;
	do{
		read(test_uart, readbuf, 1);

		// PX4_INFO("readbuf -> %c\n",*readbuf);
		// PX4_INFO("%s\n",*readbuf);
		printf("%c",*readbuf);
		// tcflush(test_uart, TCIFLUSH);
		counter++;
		// px4_usleep(10000);
	}while(counter < 100);

		// if (readval < 0) {
		// 	PX4_INFO("\n err : %d", errno);
		// 	perror("so this is the error");
		// }
		// if (receiving_data == true)
		// {
		// 	if (*readbuf != endChar)
		// 	{
		// 		attitude[index] = *readbuf;
		// 		index++;
		// 	}
		// 	else
		// 	{
		// 		attitude[index] = '\0';
		// 		receiving_data = false;
		// 		// new_data = false;
		// 		index = 0;
		// 		PX4_INFO("Attitude -> %s",attitude);
		// 	}
		// }
		// else if (*readbuf == startChar)
		// {
		// 	receiving_data = true;
		// 	index = 0;
		// }




	// bool receiving_data = false;
    	// int index = 0;
    	// char startChar = ',';
    	// char endChar = '_';
    	// bool new_data = false;
  	// // int readval_ = 0;
	// char *readbuf_ = malloc(14);
	// // readval = read(test_uart, readbuf, 13);

	// while (new_data == false)
	// {
	// 	// readval_ = read(test_uart, readbuf_, 13);
	// 	read(test_uart, readbuf_, 13);
        //     if (receiving_data == true)
        //     {
        //         if (*readbuf_ != endChar)
        //         {
        //             attitude[index] = *readbuf_;
        //             index++;
        //         }
        //         else
        //         {
        //             attitude[index] = '\0';
        //             receiving_data = false;
        //             new_data = false;
        //             index = 0;
        //         }
        //     }
        //     else if (*readbuf_ == startChar)
        //     {
        //         receiving_data = true;
        //         index = 0;
        //     }
	// }

	// char roll_char[]        = "11111";
        // char pitch_char[]       = "11111";

	// PX4_INFO("Roll -> %s",roll_char);

        // for (int i = 0; i < 11; i++)
        // {
        //     if (i < 5)
        //     {
        //         roll_char[i]                = attitude[i];
        //     } else if (i >= 6 && i < 11 )
        //     {
        //         pitch_char[i - 6]           = attitude[i];
        //     }
        // }

        // int encoder_roll_int      = atoi(roll_char);
        // int encoder_pitch_int     = atoi(pitch_char);

        // encoder_roll_feedback  =  (float)((encoder_roll_int  - 50000.0) / 100.0);
        // encoder_pitch_feedback =  (float)((encoder_pitch_int - 50000.0) / 100.0);

	// // PX4_INFO(encoder_roll_feedback);
	// // PX4_INFO("Roll -> %f",encoder_roll_feedback);
	// // PX4_INFO("Pitch -> %f\n",encoder_roll_feedback);

	// // printf("Open the %f\n",encoder_roll_feedback);

	// ScheduleOnInterval(10_ms); // 100 Hz
	// sleep(100);

	return 0;
}


				// char roll_char[]        = "11111";
				// char pitch_char[]       = "11111";

				// PX4_INFO("Roll -> %s",roll_char);

				// for (int i = 0; i < 11; i++)
				// {
				// if (i < 5)
				// {
				// 	roll_char[i]                = attitude[i];
				// } else if (i >= 6 && i < 11 )
				// {
				// 	pitch_char[i - 6]           = attitude[i];
				// }
				// }

				// int encoder_roll_int      = atoi(roll_char);
				// int encoder_pitch_int     = atoi(pitch_char);

				// encoder_roll_feedback  =  (float)((encoder_roll_int  - 50000.0) / 100.0);
				// encoder_pitch_feedback =  (float)((encoder_pitch_int - 50000.0) / 100.0);
