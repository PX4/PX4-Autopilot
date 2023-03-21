#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>
#include <px4_platform_common/module.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <math.h>
#include <stdbool.h>
#include <float.h>
#include <stdint.h>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <uORB/uORB.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/experiment.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/rc_parameter_map.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/mavlink_log.h>


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

    //const param_t param_rc_map_exp_switch = param_find("RC_MAP_EXP_SW");
    //int32_t rc_map_exp_switch = 0;
    //param_get(param_find("RC_MAP_EXP_SW"), &rc_map_exp_switch);

    // Initialize loop variables
    bool experiment = false;
    bool last_state = false;
    uint64_t start_time = 0;
    const float doublet_amplitude = 0.1f;
    const float doublet_duration = 5.0f;
    float half_duration = doublet_duration / 2.0f;
    int error_counter = 0;

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

            // Check if RC channel 5 is high and set experiment flag
            if (rc_data.values[12] > 1000) {
                experiment = true;
                //rc_map_exp_sw = 1;
            } else {
                experiment = false;
            }
        }

        // If experiment flag has been set, publish to experiment topic
        if (experiment != last_state) {
            // Set experiment flag in experiment topic
            experiment_data.experiment_running = experiment;
            experiment_data.timestamp = hrt_absolute_time();
            start_time = hrt_absolute_time()/ 1000000.0f;
            orb_publish(ORB_ID(experiment), experiment_pub, &experiment_data);
        }

        // If experiment flag is true, set experiment_controls output to a doublet signal
        if (experiment) {
            // Calculate time since last state change
            float delta_t = (hrt_absolute_time() - start_time) / 1000000.0f;

            // Calculate pitch doublet signal
            float pitch = 0.0f;
            if (delta_t >= doublet_duration) {
                // Experiment is over
                experiment = false;
                // Reset pitch to zero
                pitch = 0.0f;
            } else {
                // Calculate pitch doublet signal
                if (delta_t < half_duration) {
                    pitch = doublet_amplitude;
                } else {
                    pitch = -doublet_amplitude;
                }
            }
            // Set actuator_controls output
            experiment_data.timestamp = hrt_absolute_time();
            experiment_data.control[EXPERIMENT_INDEX_ROLL] = 0.0f;
            experiment_data.control[EXPERIMENT_INDEX_PITCH] = pitch;
            experiment_data.control[EXPERIMENT_INDEX_YAW] = 0.0f;
            experiment_data.control[EXPERIMENT_INDEX_THROTTLE] = 0.0f;
            orb_publish(ORB_ID(experiment), experiment_pub, &experiment_data);
        } else {
            // Set actuator_controls output
            experiment_data.timestamp = hrt_absolute_time();
            experiment_data.control[EXPERIMENT_INDEX_ROLL] = 0.0f;
            experiment_data.control[EXPERIMENT_INDEX_PITCH] = 0.0f;
            experiment_data.control[EXPERIMENT_INDEX_YAW] = 0.0f;
            experiment_data.control[EXPERIMENT_INDEX_THROTTLE] = 0.0f;
        }
        last_state = experiment;
        // Sleep for 20ms
        usleep(20000);
    }
    return 0;
}

int parameters_init(struct param_handles *handles)
{
	/* PID parameters */
	handles->hdng_p = param_find("EXFW_HDNG_P");
	handles->roll_p = param_find("EXFW_ROLL_P");
	handles->pitch_p = param_find("EXFW_PITCH_P");

	return 0;
}

int parameters_update(const struct param_handles *handles, struct params *parameters)
{
	param_get(handles->hdng_p, &(parameters->hdng_p));
	param_get(handles->roll_p, &(parameters->roll_p));
	param_get(handles->pitch_p, &(parameters->pitch_p));

	return 0;
}
