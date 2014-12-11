
#include <nuttx/config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <time.h>

#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>
#include <systemlib/err.h>

#include <nuttx/spi.h>

#include "tests.h"

__EXPORT int yoman_main(int argc, char *argv[]);

int yoman_main(int argc, char *argv[])
{

	int fd, result;
	

	fd = open(PWM_OUTPUT_DEVICE_PATH, O_RDWR);

	//wait for 5 sec and make channal0 to 1600(high)
	usleep(5000000);
	printf("Advancing channel 0 to 1600\n");
	result = ioctl(fd, PWM_SERVO_SET(0), 1600);
	printf("%d \n",result );
	printf("Advancing channel 1 to 1800\n");
	result = ioctl(fd, PWM_SERVO_SET(1), 1800);
	printf("%d \n",result );
	

	//wait for 6sec and make channel0 to 1100(low)
	usleep(6000000);
	printf("Advancing channel 0 to 1100\n");
	result = ioctl(fd, PWM_SERVO_SET(0), 1100);
	printf("%d \n",result );	

	//wait for 6sec and make channel0 to 1350(low)
	usleep(6000000);
	printf("Advancing channel 0 to 1350\n");
	result = ioctl(fd, PWM_SERVO_SET(0), 1350);
	printf("%d \n",result );	

	////wait for 6sec and make channel0 to 1350(low)
	usleep(6000000);
	result = ioctl(fd, PWM_SERVO_SET(0), 1100);
	printf("%d \n",result );

	for (int i = 0; i <20; ++i)
	{
		/* code */
		result = ioctl(fd, PWM_SERVO_SET(0), 1100+10*i);
		printf("%d \n",result );
		usleep(100000);

	}


	return 0;
}
