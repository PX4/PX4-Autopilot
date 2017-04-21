#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>

__EXPORT int px4_hello_sky_main(int argc, char *argv[]);

int px4_hello_sky_main(int argc, char *argv[])
{
	/*TO USE THE NUTTX CONSOLE, REMOVE THE SD CARD!*/
    PX4_INFO("Hello Sky!");
    return OK;
}
