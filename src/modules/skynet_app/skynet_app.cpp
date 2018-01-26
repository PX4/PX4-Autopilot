//Here is a file that I made.



#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>


/********************************************************************
 * Includes that are not part of original "px4_example_app" are here*/

#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_command.h>


static volatile bool thread_running = false;	








extern "C" __EXPORT int skynet_app_main(int argc, char *argv[]);

static void usage(const char * reason);

int skynet_app_main(int argc, char *argv[])
{
    if(argc < 2) {
        usage("missing command");
            return 1;
    }

    if(!strcmp(argv[1], "start")) {
        if(thread_running) {
            warnx("skynet_app already running");
            return 0;
        }
        thread_running = true;

        PX4_INFO("skynet_app start.");
        return 0;
    }
    
    if(!strcmp(argv[1], "stop")) {
        if(!thread_running) {
            warnx("skynet_app not running; no shutdown required.");
            return 0;
        }
        thread_running = false;


        PX4_INFO("skynet_app shutting down.");
        return 0;
    }

    if(!thread_running) {
        warnx("skynet_app not yet running");
        return 1;
    }

    
    if(!strcmp(argv[1], "status")) {
        PX4_INFO("skynet_app status.");
        return 0;
    }
    
       
    if(!strcmp(argv[1], "test")) {
        
        PX4_INFO("skynet_app simple test.");
        
        position_setpoint_s z;
        z.x = 0.0f;
        if(z.x < 1.0f)
            PX4_INFO("x coordinate set, test win. yay.");

        return 0;
    }
    
    usage("command given but not recognized.");
    return 1;



   

















#ifdef SKYNET
    /* subscribe to sensor_combined topic */
    int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
    /* limit the update rate to 5 Hz */
    orb_set_interval(sensor_sub_fd, 200);

    /* advertise attitude topic */
    struct vehicle_attitude_s att;
    memset(&att, 0, sizeof(att));
    orb_advert_t att_pub = orb_advertise(ORB_ID(vehicle_attitude), &att);

    /* one could wait for multiple topics with this technique, just using one here */
    px4_pollfd_struct_t fds[] = {
        { .fd = sensor_sub_fd,   .events = POLLIN },
        /* there could be more file descriptors here, in the form like:
         * { .fd = other_sub_fd,   .events = POLLIN },
         */
    };

    int error_counter = 0;

    for (int i = 0; i < 5; i++) {
        /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
        int poll_ret = px4_poll(fds, 1, 1000);

        /* handle the poll result */
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

            if (fds[0].revents & POLLIN) {
                /* obtained data for the first file descriptor */
                struct sensor_combined_s raw;
                /* copy sensors raw data into local buffer */
                orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
                PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
                        (double)raw.accelerometer_m_s2[0],
                        (double)raw.accelerometer_m_s2[1],
                        (double)raw.accelerometer_m_s2[2]);

                /* set att and publish this information for other apps
                   the following does not have any meaning, it's just an example
                   */
                att.q[0] = raw.accelerometer_m_s2[0];
                att.q[1] = raw.accelerometer_m_s2[1];
                att.q[2] = raw.accelerometer_m_s2[2];

                orb_publish(ORB_ID(vehicle_attitude), att_pub, &att);
            }

            /* there could be more file descriptors here, in the form like:
             * if (fds[1..n].revents & POLLIN) {}
             */
        }
    }

#endif
    PX4_INFO("exiting");

    return 0;
}




static void usage(const char *reason)
{
	if (reason && *reason > 0) {
		PX4_INFO("%s", reason);
	}

	PX4_INFO("usage: skynet_app {start|stop|status|test}\n");
        return;
}


















