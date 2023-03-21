#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <poll.h>
#include <float.h>
#include <stdint.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/module.h>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <uORB/uORB.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/experiment.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/rc_parameter_map.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/mavlink_log.h>

#define PI 3.14159265358979323846

__EXPORT int experiment_main(int argc, char *argv[]);

int experiment_main(int argc, char *argv[])
{

    //Input charactertictics
    const float magnitude = 0.1; 
    const float duration = 20;

    // Subscribe to input_rc topic
    int rc_sub = orb_subscribe(ORB_ID(input_rc));
    struct input_rc_s rc_data;

    // Advertise to experiment topic
    struct experiment_s experiment_data;
    memset(&experiment_data, 0, sizeof(experiment_data));
    orb_advert_t experiment_pub = orb_advertise(ORB_ID(experiment), &experiment_data);

    //Initalize loop variables
    int error_counter = 0;
    double step, doublet, sine_sweep;
    bool experiment = false;
    bool last_state = false;
    uint64_t start_time = 0;
    float input = 0.0f;

    // To activate experiment via RC and publish that experiment is activated
    while (true) {
        // Wait for up to 20ms for new data
        struct pollfd fds[1];
        fds[0].fd = rc_sub;
        fds[0].events = POLLIN;
        int poll_ret = poll(fds, 1, 20);

        // If new data is available, read the input_rc topic
	    if (poll_ret == 0) {
		    /* this means none of our providers is giving us data */
		    PX4_ERR("Got no data within a second");

	    } else if (poll_ret < 0) {
		    /* this is seriously bad - should be an emergency */
	    	if (error_counter < 10 || error_counter % 50 == 0) {
	    		/* use a counter to prevent flooding (and slowing us down) */
	    		PX4_ERR("ERROR return value from poll(): %d", poll_ret);
	    	}
	    	error_counter++;

	    } else {
            orb_copy(ORB_ID(input_rc), rc_sub, &rc_data);

            // Check if RC channel 7 is high and set experiment flag
            if (rc_data.values[6] > 1500) {
                experiment = true;
                PX4_INFO("Experiment activated ");
                start_time = hrt_absolute_time()/ 1000000.0f;
            } else {
                experiment = false;
            }
        }

        // Publish to experiment topic
        if (experiment != last_state) {
            experiment_data.experiment_running = experiment;
            experiment_data.timestamp = hrt_absolute_time();
            orb_publish(ORB_ID(experiment), experiment_pub, &experiment_data);
        } 

        if (experiment_data.experiment_running) {
            PX4_INFO("Experiment started.\n");

            // Calculate time since last state change
            float delta_t = (hrt_absolute_time() - start_time) / 1000000.0f;

            switch (experiment_data.injection_input) {
                case EXPERIMENT_INDEX_STEP_INPUT:
                    PX4_INFO("Generating step with magnitude 0.1 and duration 20sec");
                    // Generate step input
                    if (delta_t <= 2) {
                        step = 0;
                    } else if (2 < delta_t && delta_t <= duration) {
                        step = magnitude;
                    } else {
                        step = 0;
                    }
                    input = step;
                    experiment_data.injection_input[EXPERIMENT_INDEX_STEP_INPUT] = step;
                break;
                case EXPERIMENT_INDEX_DOUBLET_INPUT:
                    PX4_INFO("Generating doublet with magnitude 0.1 and duration 20sec");
                    // Generate doublet input
                    if (delta_t <= 2) {
                        doublet = 0;
                    } else if (2 <= delta_t && delta_t <= 2.5) {
                        doublet = magnitude;
                    } else if (2.5 <= delta_t && delta_t <= 3) {
                        doublet = -magnitude;
                    } else {
                        doublet = 0;
                    }
                    input = doublet;
                    experiment_data.injection_input[EXPERIMENT_INDEX_DOUBLET_INPUT] = doublet;
                break;
                case EXPERIMENT_INDEX_SINE_SWEEP:
                    PX4_INFO("Generating sine sweep with magnitude 0.1 and duration 20sec");
                    // Generate sine sweep input
                    double freq_start = 0.1;
                    double freq_end = 7.0;
                    double k = (freq_end - freq_start) / (duration-2);
                    if (delta_t <= 2) {
                        sine_sweep = 0;
                    } else if (delta_t >= 2 && delta_t <= duration) {
                        double f = freq_start + k*(del-2);
                        sine_sweep = magnitude*sin(2*PI*f*delta_t);
                    } else {
                        sine_sweep = 0;
                    }
                    input = sine_sweep;
                    experiment_data.injection_input[EXPERIMENT_INDEX_SINE_SWEEP] = sine_sweep;
                break;
            }
            switch(experiment_data.injection_output){
                case EXPERIMENT_INDEX_THROTTLE_OUTPUT:
                    PX4_INFO("Generating the input in throttle");
                    experiment_data.injection_output[EXPERIMENT_INDEX_THROTTLE_OUTPUT] = input;
                    experiment_data.timestamp = hrt_absolute_time();
                    experiment_data.control[EXPERIMENT_INDEX_ROLL] = 0.0f;
                    experiment_data.control[EXPERIMENT_INDEX_PITCH] = 0.0f;
                    experiment_data.control[EXPERIMENT_INDEX_YAW] = 0.0f;
                    experiment_data.control[EXPERIMENT_INDEX_THROTTLE] = input;
                    orb_publish(ORB_ID(experiment), experiment_pub, &experiment_data);
                break;
                case EXPERIMENT_INDEX_ELEVATOR_OUTPUT:
                    PX4_INFO("Generating the input in elevator");
                    experiment_data.injection_output[EXPERIMENT_INDEX_ELEVATOR_OUTPUT] = input;
                    experiment_data.timestamp = hrt_absolute_time();
                    experiment_data.control[EXPERIMENT_INDEX_ROLL] = 0.0f;
                    experiment_data.control[EXPERIMENT_INDEX_PITCH] = input;
                    experiment_data.control[EXPERIMENT_INDEX_YAW] = 0.0f;
                    experiment_data.control[EXPERIMENT_INDEX_THROTTLE] = 0.0f;
                    orb_publish(ORB_ID(experiment), experiment_pub, &experiment_data);
                break;
            }
        } else {
            // Set actuator_controls output
            PX4_INFO("No experimental input generated");
            experiment_data.timestamp = hrt_absolute_time();
            experiment_data.control[EXPERIMENT_INDEX_ROLL] = 0.0f;
            experiment_data.control[EXPERIMENT_INDEX_PITCH] = 0.0f;
            experiment_data.control[EXPERIMENT_INDEX_YAW] = 0.0f;
            experiment_data.control[EXPERIMENT_INDEX_THROTTLE] = 0.0f;
        }
        PX4_INFO("Experiment completed.\n");
        last_state = experiment;
        // Sleep for 20ms
        usleep(20000);
    }
    return 0;
}

