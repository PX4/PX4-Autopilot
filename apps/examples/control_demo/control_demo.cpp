/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Example User <mail@example.com>
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
 * @file control_demo.cpp
 * Demonstration of control library
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <systemlib/systemlib.h>
#include <systemlib/control/fixedwing.h>
#include <systemlib/param/param.h>
#include <drivers/drv_hrt.h>
#include <math.h>

static bool thread_should_exit = false;     /**< Deamon exit flag */
static bool thread_running = false;     /**< Deamon status flag */
static int deamon_task;             /**< Handle of deamon task / thread */

/**
 * Deamon management function.
 */
extern "C" __EXPORT int control_demo_main(int argc, char *argv[]);

/**
 * Mainloop of deamon.
 */
int control_demo_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
    if (reason)
        fprintf(stderr, "%s\n", reason);
    fprintf(stderr, "usage: control_demo {start|stop|status} [-p <additional params>]\n\n");
    exit(1);
}

/**
 * The deamon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 * 
 * The actual stack size should be set in the call
 * to task_create().
 */
int control_demo_main(int argc, char *argv[])
{

    if (argc < 1)
        usage("missing command");

    if (!strcmp(argv[1], "start")) {

        if (thread_running) {
            printf("control_demo already running\n");
            /* this is not an error */
            exit(0);
        }

        thread_should_exit = false;
        deamon_task = task_spawn("control_demo",
                     SCHED_DEFAULT,
                     SCHED_PRIORITY_DEFAULT,
                     8192,
                     control_demo_thread_main,
                     (argv) ? (const char **)&argv[2] : (const char **)NULL);
        exit(0);
    }

    if (!strcmp(argv[1], "stop")) {
        thread_should_exit = true;
        exit(0);
    }

    if (!strcmp(argv[1], "status")) {
        if (thread_running) {
            printf("\tcontrol_demo app is running\n");
        } else {
            printf("\tcontrol_demo app not started\n");
        }
        exit(0);
    }

    usage("unrecognized command");
    exit(1);
}

int control_demo_thread_main(int argc, char *argv[]) {

    printf("[control_Demo] starting\n");

    using namespace control;
    Block fw(NULL,"FW");
    fixedwing::BlockStabilization blockStabilization(&fw,"STAB");
    fixedwing::BlockBacksideAutopilot blockBacksideAutopilot(&fw,"BS");
    fixedwing::BlockOutputs blockOutputs(&fw,"OUT");

    thread_running = true;
    uint32_t loopCount = 0;
    uint32_t controlMode = 0;

    fw.setDt(1.0f /50.0f);

    while (!thread_should_exit) {

        struct timespec ts;
        abstime_to_ts(&ts,hrt_absolute_time());
        float t = ts.tv_sec + ts.tv_nsec/1.0e9;
        float u = sin(t);

        float p = u;
        float q = u;
        float r = u;

        float h = u;
        float v = u;
        float phi = u;
        float theta = u;
        float psi = u;

        float pCmd = 0;
        float qCmd = 0;
        float rCmd = 0;

        float vCmd = 0;
        float hCmd = 0;
        float psiCmd = 0;

        float manualAileron = 0.1;
        float manualElevator = 0.2;
        float manualRudder = 0.3;
        float manualThrottle = 0.4;


        if (controlMode == 0)
        {
            // stabilize
            blockStabilization.update(pCmd, qCmd, rCmd, p, q, r);
            blockOutputs.update(blockStabilization.getAileron(),
                    blockStabilization.getElevator(),
                    blockStabilization.getRudder(),
                    manualThrottle);
        }
        else if (controlMode == 1)
        {
            // auto
            blockBacksideAutopilot.update(hCmd, vCmd, rCmd, psiCmd,
                h, v,
                phi, theta, psi,
                p, q, r);
            blockOutputs.update(blockBacksideAutopilot.getAileron(),
                    blockBacksideAutopilot.getElevator(),
                    blockBacksideAutopilot.getRudder(),
                    blockBacksideAutopilot.getThrottle());
        }
        else if (controlMode == 2)
        {
            // manual
            blockOutputs.update(manualAileron,
                    manualElevator,
                    manualRudder,
                    manualThrottle);
        }

        if (loopCount-- <= 0)
        {
            loopCount = 100;
            blockStabilization.updateParams();
            blockBacksideAutopilot.updateParams();
            blockOutputs.updateParams();
            printf("t: %8.4f, u: %8.4f\n", (double)t, (double)u);
            printf("control mode: %d\n", controlMode);
            printf("aileron: %8.4f, elevator: %8.4f, "
                    "rudder: %8.4f, throttle: %8.4f\n",
                    (double)blockOutputs.getAileron(),
                    (double)blockOutputs.getElevator(),
                    (double)blockOutputs.getRudder(),
                    (double)blockOutputs.getThrottle());
            controlMode++;
            fflush(stdout);
        }

        if (controlMode > 2) controlMode = 0;

        usleep(20000);
    }

    printf("[control_demo] exiting.\n");

    thread_running = false;

    return 0;
}
