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
    // Subscribe to input_rc topic
    int rc_sub = orb_subscribe(ORB_ID(input_rc));
    struct input_rc_s rc_data;

    // Advertise to experiment topic
    struct experiment_s experiment_data;
    memset(&experiment_data, 0, sizeof(experiment_data));
    orb_advert_t experiment_pub = orb_advertise(ORB_ID(experiment), &experiment_data);


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
                experiment_data.experiment_running = true;
            } else {
                experiment_data.experiment_running = false;
            }
        }

        // Publish to experiment topic
        if (experiment_running) {
            experiment_data.experiment_running = true;
            experiment_data.timestamp = hrt_absolute_time();
            orb_publish(ORB_ID(experiment), experiment_pub, &experiment_data);
            PX4_INFO("Experiment activated ");
        } else {
            experiment_data.experiment_running = false;
            experiment_data.timestamp = hrt_absolute_time();
            orb_publish(ORB_ID(experiment), experiment_pub, &experiment_data);
        }
    }

    // Input charactertictics
    double magnitude = 0.1; 
    double duration = 20;
    double Ts = 0.02;

    //Initialization of local variables for input generation
    double step, doublet, sine_sweep;
    int num_samples = duration/Ts;
    double input[num_samples];

    if (experiment_data.experiment_running) {
        int n = 0;
        while (true) {
            PX4_INFO("Experiment started.\n");
            switch (experiment_data.injection_input) {
                case EXPERIMENT_INDEX_STEP_INPUT:
                    PX4_INFO("Generating step with magnitude 0.1 and duration 20sec");
                    // Generate step input
                    for (double t = 0; t <= duration; t += Ts) {
                        if (t <= 2) {
                            step = 0;
                        } else if (2 <= t <= duration) {
                            step = magnitude;
                        } else {
                            step = 0;
                        }
                        input[n] = step;
                        n++;
                    }
                    experiment_data.injection_input[EXPERIMENT_INDEX_STEP_INPUT] = input;
                    break;
                case EXPERIMENT_INDEX_DOUBLET_INPUT:
                    PX4_INFO("Generating doublet with magnitude 0.1 and duration 20sec");
                    // Generate doublet input
                    for (double t = 0; t <= duration; t += Ts) {

                        if (t <= 2) {
                            doublet = 0;
                        } else if (2 < t && t <= 2.5) {
                            doublet = magnitude;
                        } else if (2.5 < t && t <= 3) {
                            doublet = -magnitude;
                        } else {
                            doublet = 0;
                        }
                        input[n] = step;
                        n++;
                    }
                    experiment_data.injection_input[EXPERIMENT_INDEX_DOUBLET_INPUT] = input;
                    break;
                case EXPERIMENT_INDEX_SINE_SWEEP:
                    
                    // Generate sine sweep input
                    double freq_start = 0.1;
                    double freq_end = 7.0;
                    double k = (freq_end - freq_start) / (duration-2);
                    double f;
                    for (double t = 0; t <= duration; t += Ts) {
                        time[n] = t;
                        if (t <= 2) {
                            sine_sweep = 0;
                        } else if (t >= 2 && t <= duration) {
                            f = freq_start + k*(t-2);
                            sine_sweep = magnitude*sin(2*PI*f*t);
                        } else {
                            sine_sweep = 0;
                        }
                        input[n] = sine_sweep;
                        n++;
                    }
                    experiment_data.injection_input[EXPERIMENT_INDEX_SINE_SWEEP] = input;
                    break;
            }
            switch(experiment_data.injection_output){
                case EXPERIMENT_INDEX_THROTTLE_OUTPUT:
                PX4_INFO("Generating the input in throttle");
                for (int i = 0; i < num_samples; i++) {
                throttle[i] = input[i];
                }
                experiment_data.injection_output[EXPERIMENT_INDEX_THROTTLE_OUTPUT] = throttle;
                break;
                case EXPERIMENT_INDEX_ELEVATOR_OUTPUT:
                PX4_INFO("Generating the input in elevator");
                for (int i = 0; i < num_samples; i++) {
                elevator[i] = input[i];
                }
                experiment_data.injection_output[EXPERIMENT_INDEX_ELEVATOR_OUTPUT] = input;
                break;
            }
            PX4_INFO("Experiment completed.\n");
            // Sleep for 20ms
            usleep(20000);
        }
    }
    return 0;
}

