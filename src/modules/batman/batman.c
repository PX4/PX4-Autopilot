//!
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/alphaomega.h>

__EXPORT int batman_main(int argc, char *argv[]);

int batman_main(int argc, char *argv[])
{
	PX4_INFO("Hello Pixhawk Code!");
    int alphaomega_sub_fd = orb_subscribe(ORB_ID(alphaomega));
    orb_set_interval(alphaomega_sub_fd, 200);

	px4_pollfd_struct_t fds[] = {
        { .fd = alphaomega_sub_fd,   .events = POLLIN },
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

					struct alphaomega_s stuff;
					orb_copy(ORB_ID(alphaomega), alphaomega_sub_fd, &stuff);
					PX4_INFO("Success!\t%8.4f\t%8.4f\t%8.4f",(double)stuff.alpha[0],(double)stuff.alpha[1],
				          (double)stuff.omega[3]);

				}

		}
	}

	PX4_INFO("exiting");

	return 0;
}
