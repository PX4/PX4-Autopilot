#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <poll.h>
#include <float.h>
#include <stdint.h>
#include <string.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/module.h>
#include <drivers/drv_hrt.h>
#include <lib/ecl/geo/geo.h>
#include <matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <uORB/uORB.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/experiment.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/rc_parameter_map.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/debug_key_value.h>
#include <string.h>
#include <board_config.h>
#include <px4_platform_common/board_common.h>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/module.h>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/module_params.h>
#include <parameters/param.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
//#include <drivers/uavcan/libuavcan/libuavcan/include/uavcan/driver/system_clock.hpp>
#include "experiment.h"
#include "params.h"

/* Prototypes */

/** Initialize all parameter handles and values**/
extern "C" int exp_parameters_init(struct param_handles *h);

/**Update all parameters**/
extern "C" int exp_parameters_update(const struct param_handles *h, struct params *p);

/**Daemon management function.
 * This function allows to start / stop the background task (daemon).
 * The purpose of it is to be able to start on the command line, query its status and stop it, without giving up
 * the command line to one particular process or the need for bg/fg^Z support by the shell.**/
extern "C" __EXPORT int experiment_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/**
 * Mainloop of daemon.
 */
int fixedwing_control_thread_main(int argc, char *argv[]);

/* Deamon variable*/
static bool thread_should_exit = false;		/**< Daemon exit flag */
static bool thread_running = false;		/**< Daemon status flag */
static int deamon_task;				/**< Handle of deamon task / thread */

//TO WORK WITH PARAMETER
static struct params pp;
static struct param_handles ph;

int exp_parameters_init(struct param_handles *handles)
{
	handles->input_type 	=	param_find("EXP_INPUT");
	handles->output_type 	=	param_find("EXP_OUTPUT");
	handles->exp_flag 	=	param_find("EXP_FLAG");

	return 0;
}

int exp_parameters_update(const struct param_handles *handles, struct params *parameters)
{
	param_get(handles->input_type, &(parameters->input_type));
	param_get(handles->output_type, &(parameters->output_type));
	param_get(handles->exp_flag, &(parameters->exp_flag));

	return 0;
}


/* Main Thread */
int experiment_thread_main(int argc, char *argv[])
{
    /* welcome user (warnx prints a line, including an appended\n, with variable arguments */
	warnx("experiment application started");

    //Input charactertictics
    const float magnitude = 1; 
    const float duration = 20;
    float freq_start = 0.1;
    float freq_end = 1.0;

    /* initialize parameters, first the handles, then the values */
	exp_parameters_init(&ph);
	exp_parameters_update(&ph, &pp);

    // Subscribe to input_rc topic
    int rc_sub = orb_subscribe(ORB_ID(input_rc));
    struct input_rc_s rc_data;

    // Advertise to experiment topic
    struct experiment_s experiment_data;
    memset(&experiment_data, 0, sizeof(experiment_data));
    orb_advert_t experiment_pub = orb_advertise(ORB_ID(experiment), &experiment_data);

    /* advertise debug value */
    struct debug_key_value_s dbg;
    strncpy(dbg.key, "velx", sizeof(dbg.key));
    dbg.value = 0.0f;
    orb_advert_t pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);
    dbg.value = experiment_data.experiment_running;
    orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);

    //Initalize loop variables
    int error_counter = 0;
    double step, doublet, sine_sweep;
    bool experiment = false;
    uint64_t start_time = 0;
    float input = 0.0f;
    bool last_state = false;
    float delta_t = 0;

    // Wait for up to 20ms for new data
        struct pollfd fds[1];
        fds[0].fd = rc_sub;
        fds[0].events = POLLIN;

    // To activate experiment via RC and publish that experiment is activated
    while (!thread_should_exit) {
        
        int poll_ret = poll(fds, 1, 20);

        // If new data is available, read the input_rc topic
	    if (poll_ret == 0) {
		    /* this means none of our providers is giving us data */
		    //PX4_ERR("Got no data within a second");

	    } else if (poll_ret < 0) {
		    /* this is seriously bad - should be an emergency */
	    	if (error_counter < 10 || error_counter % 50 == 0) {
	    		/* use a counter to prevent flooding (and slowing us down) */
	    		//PX4_ERR("ERROR return value from poll(): %d", poll_ret);
	    	}
	    	error_counter++;

	    } else {
            //When the rc input is avaiable copy to rc_data
            orb_copy(ORB_ID(input_rc), rc_sub, &rc_data);

            // Check if RC channel 7 is high and set experiment flag
            if (rc_data.values[6] > 1500) {
            //if(true){
                experiment = true;
            } else {
                experiment = false;
            }
        }
    
        // Publish to experiment topic
        if (experiment != last_state) {
            if(experiment){
                    pp.exp_flag = 1;
                    PX4_INFO("Experiment activated.\n");
                    start_time = hrt_absolute_time()/ 1000000.0f;
                    //PX4_INFO_RAW("start time reset at %.4f secs \n",(double)start_time);
                } else {
                    pp.exp_flag = 0;
                    PX4_INFO("Experiment deactivated.\n");;
                }
            experiment_data.experiment_running = experiment;
            experiment_data.timestamp = hrt_absolute_time();
            orb_publish(ORB_ID(experiment), experiment_pub, &experiment_data);
            dbg.value = experiment_data.experiment_running;
            orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);
            exp_parameters_update(&ph, &pp);
            last_state = true;
        } 
            
        //To generate the input based on choice
        if (experiment) {
            // Calculate time since last state change
            //if (delta_t<= duration){
            delta_t = (hrt_absolute_time()) / 1000000.0f - start_time; 
            //PX4_INFO_RAW("delta time at %.4f secs \n",(double)delta_t);
            //} else {
               // delta_t = duration+1;
            //}

            switch (pp.input_type) {
            case 100:
                // Generate step input
                if (delta_t <= 2) {
                    step = 0;
                } else if (2 < delta_t && delta_t <= duration) {
                    step = magnitude;
                } else {
                    step = 0;
                }
                input = step;
                experiment_data.injection_input[experiment_s::INDEX_STEP_INPUT] = step;
                PX4_INFO_RAW("Generating step with magnitude %.4f and at %.4f secs \n",(double)step, (double)delta_t);
            break;
            case 110:
                // Generate doublet input
                if (delta_t <= 2) {
                    doublet = 0;
                } else if (2 <= delta_t && delta_t <= 2.5f) {
                    doublet = magnitude;
                } else if (2.5f <= delta_t && delta_t <= 3) {
                    doublet = -magnitude;
                } else {
                    doublet = 0;
                }
                input = doublet;
                experiment_data.injection_input[experiment_s::INDEX_DOUBLET_INPUT] = doublet;
                //PX4_INFO_RAW("Generating doublet with magnitude %.4f and at %.4f secs \n",(double)doublet, (double)delta_t);
            break;
            case 111:
                // Generate sine sweep input
                float k = (freq_end - freq_start) / (duration-2.0f);
                if (delta_t <= 2) {
                    sine_sweep = 0;
                } else if (delta_t >= 2 && delta_t <= duration) {
                    float f = freq_start + k*(delta_t-2);
                    sine_sweep = magnitude*(float)sin(2.0f*M_PI_F*f*delta_t);
                } else {
                    sine_sweep = 0;
                }
                input = sine_sweep;
                experiment_data.injection_input[experiment_s::INDEX_SINE_SWEEP_INPUT] = sine_sweep;
                PX4_INFO_RAW("Generating sine sweep with magnitude %.4f and at %.4f secs \n",(double)sine_sweep, (double)delta_t);
            break;
            }

            //To send the generated input to the actuator of choice
            switch(pp.output_type){
            case 10:
                PX4_INFO("Generating the input in throttle");
                experiment_data.injection_output[experiment_s::INDEX_THROTTLE_OUTPUT] = input;
                experiment_data.timestamp = hrt_absolute_time();
                experiment_data.control[experiment_s::INDEX_ROLL] = 0.0f;
                experiment_data.control[experiment_s::INDEX_PITCH] = 0.0f;
                experiment_data.control[experiment_s::INDEX_YAW] = 0.0f;
                experiment_data.control[experiment_s::INDEX_THROTTLE] = input;
                orb_publish(ORB_ID(experiment), experiment_pub, &experiment_data);
            break;
            case 11:
                //PX4_INFO("Generating the input in elevator");
                experiment_data.injection_output[experiment_s::INDEX_ELEVATOR_OUTPUT] = input;
                experiment_data.timestamp = hrt_absolute_time();
                experiment_data.control[experiment_s::INDEX_ROLL] = 0.0f;
                experiment_data.control[experiment_s::INDEX_PITCH] = input;
                experiment_data.control[experiment_s::INDEX_YAW] = 0.0f;
                experiment_data.control[experiment_s::INDEX_THROTTLE] = 0.0f;
                orb_publish(ORB_ID(experiment), experiment_pub, &experiment_data);
            break;
            }
            last_state = true;
        //When experiment is not active, no input sent
        } else {
            //PX4_INFO("No experimental input generated");
            experiment_data.timestamp = hrt_absolute_time();
            experiment_data.control[experiment_s::INDEX_ROLL] = 0.0f;
            experiment_data.control[experiment_s::INDEX_PITCH] = 0.0f;
            experiment_data.control[experiment_s::INDEX_YAW] = 0.0f;
            experiment_data.control[experiment_s::INDEX_THROTTLE] = 0.0f;
            orb_publish(ORB_ID(experiment), experiment_pub, &experiment_data);
            last_state = false;
        }
            
        //To update parameters
        exp_parameters_update(&ph, &pp);
        // Sleep for 20ms
        usleep(20000);
        //}
    }
    warnx("experiment stopped.\n");
	thread_running = false;
    return 0;
}

/* Startup Functions */

static void
usage(const char *reason)
{
	if (reason) {
		fprintf(stderr, "%s\n", reason);
	}

	fprintf(stderr, "usage: experiment {start|stop|status}\n\n");
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to px4_task_spawn_cmd().
 */
int experiment_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("experiment already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		deamon_task = px4_task_spawn_cmd("experiment",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 20,
						 2048,
						 experiment_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)nullptr);
		thread_running = true;
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\texperiment is running\n");

		} else {
			printf("\texperiment not started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 0;
}

//clock_gettime
