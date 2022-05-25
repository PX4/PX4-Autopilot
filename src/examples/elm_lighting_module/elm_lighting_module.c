/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file elm_lighting_module.c
 * Minimal application example for PX4 autopilot
 *
 * @author Landon Haugh <landon.haugh@nxp.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/lighting_states.h>

__EXPORT int elm_lighting_module_main(int argc, char *argv[]);

int elm_lighting_module_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

	/* advertise lighting_states topic */
	struct lighting_states_s lighting_states;
	memset(&lighting_states, 255, sizeof(lighting_states));
	orb_advert_t lighting_states_pub = orb_advertise(ORB_ID(lighting_states), &lighting_states);

/*
	for (int i = 0; i < 8; i++) {
		lighting_states.state[i] = 255;
	}
*/
    for(int i = 0; i < 200; i++) {
        px4_usleep(2000000);

        // Idle state
        printf("Updating state to IDLE\n");
        lighting_states.state[0] = 0;
        lighting_states.state[1] = 0;
        lighting_states.state[2] = 0;
        lighting_states.state[3] = 0;
        lighting_states.state[4] = 2;
        lighting_states.state[5] = 2;
        lighting_states.state[6] = 2;
        lighting_states.state[7] = 2;
        lighting_states.timestamp = hrt_absolute_time();
        orb_publish(ORB_ID(lighting_states), lighting_states_pub, &lighting_states);
        px4_usleep(2000000);
        

        // Braking state (NO OTHER STATES)
        printf("Updating state to BRAKE\n");
        lighting_states.state[0] = 1;
        lighting_states.state[1] = 1;
        lighting_states.state[2] = 1;
        lighting_states.state[3] = 1;
        lighting_states.state[4] = 2;
        lighting_states.state[5] = 2;
        lighting_states.state[6] = 2;
        lighting_states.state[7] = 2;
        lighting_states.timestamp = hrt_absolute_time();
        orb_publish(ORB_ID(lighting_states), lighting_states_pub, &lighting_states);
        px4_usleep(2000000); 
        

        // Idle state while reversing
        printf("Updating state to IDLE/REVERSE\n");
        lighting_states.state[0] = 0;
        lighting_states.state[1] = 2;
        lighting_states.state[2] = 0;
        lighting_states.state[3] = 2;
        lighting_states.state[4] = 2;
        lighting_states.state[5] = 2;
        lighting_states.state[6] = 2;
        lighting_states.state[7] = 2;
        lighting_states.timestamp = hrt_absolute_time();
        orb_publish(ORB_ID(lighting_states), lighting_states_pub, &lighting_states);
        px4_usleep(2000000);
        

        // Braking state while reversing
        printf("Updating state to BRAKE/REVERSE\n");
        lighting_states.state[0] = 1;
        lighting_states.state[1] = 2;
        lighting_states.state[2] = 1;
        lighting_states.state[3] = 2;
        lighting_states.state[4] = 2;
        lighting_states.state[5] = 2;
        lighting_states.state[6] = 2;
        lighting_states.state[7] = 2;
        lighting_states.timestamp = hrt_absolute_time();
        orb_publish(ORB_ID(lighting_states), lighting_states_pub, &lighting_states);
        px4_usleep(2000000);
        

        // Idle state while turning
        printf("Updating state to IDLE/TURN\n");
        lighting_states.state[0] = 0;
        lighting_states.state[1] = 3;
        lighting_states.state[2] = 0;
        lighting_states.state[3] = 0;
        lighting_states.state[4] = 2;
        lighting_states.state[5] = 3;
        lighting_states.state[6] = 2;
        lighting_states.state[7] = 2;
        lighting_states.timestamp = hrt_absolute_time();
        orb_publish(ORB_ID(lighting_states), lighting_states_pub, &lighting_states);
        px4_usleep(2000000);
        

        // Braking state while turning
        printf("Updating state to BRAKE/TURN\n");
        lighting_states.state[0] = 0;
        lighting_states.state[1] = 0;
        lighting_states.state[2] = 0;
        lighting_states.state[3] = 3;
        lighting_states.state[4] = 2;
        lighting_states.state[5] = 2;
        lighting_states.state[6] = 2;
        lighting_states.state[7] = 3;
        lighting_states.timestamp = hrt_absolute_time();
        orb_publish(ORB_ID(lighting_states), lighting_states_pub, &lighting_states);
        px4_usleep(2000000);
        

        // Return to Idle
        printf("Updating state to IDLE\n");
        lighting_states.state[0] = 0;
        lighting_states.state[1] = 0;
        lighting_states.state[2] = 0;
        lighting_states.state[3] = 0;
        lighting_states.state[4] = 2;
        lighting_states.state[5] = 2;
        lighting_states.state[6] = 2;
        lighting_states.state[7] = 2;
        lighting_states.timestamp = hrt_absolute_time();
        orb_publish(ORB_ID(lighting_states), lighting_states_pub, &lighting_states);
        px4_usleep(2000000);
    }

	PX4_INFO("exiting");

	return 0;
}
