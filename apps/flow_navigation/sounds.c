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

void tune_sonar(void)
{
	ioctl(buzzer, TONE_SET_ALARM, 4);
}

void tune_ready(void)
{
	ioctl(buzzer, TONE_SET_ALARM, 3);
}

void tune_mission_started(void)
{
	ioctl(buzzer, TONE_SET_ALARM, 3);
}

void tune_mission_aborded(void)
{
	ioctl(buzzer, TONE_SET_ALARM, 5);
}

void tune_mission_accomplished(void)
{
	ioctl(buzzer, TONE_SET_ALARM, 8);
}
