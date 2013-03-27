/*
 * sounds.c
 *
 *  Created on: Feb 26, 2013
 *      Author: samuezih
 */

#include <sys/types.h>
#include <fcntl.h>
#include <drivers/drv_tone_alarm.h>


static int buzzer;

int sounds_init(void)
{
	buzzer = open("/dev/tone_alarm", O_WRONLY);

	if (buzzer < 0) {
		warnx("Buzzer: open fail\n");
		return ERROR;
	}

	return 0;
}

void sounds_deinit(void)
{
	close(buzzer);
}

void tune_tetris(void)
{
	ioctl(buzzer, TONE_SET_ALARM, 6);
}

void tune_sonar(void)
{
	ioctl(buzzer, TONE_SET_ALARM, 7);
}
