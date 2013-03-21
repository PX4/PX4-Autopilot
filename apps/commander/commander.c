/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Petri Tanskanen <petri.tanskanen@inf.ethz.ch>
 *           Lorenz Meier <lm@inf.ethz.ch>
 *           Thomas Gubler <thomasgubler@student.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file commander.c
 * Main system state machine implementation.
 *
 * @author Petri Tanskanen <petri.tanskanen@inf.ethz.ch>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 *
 */

#include "commander.h"

#include <nuttx/config.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <sys/prctl.h>
#include <string.h>
#include <drivers/drv_led.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>
#include "state_machine_helper.h"
#include "systemlib/systemlib.h"
#include <math.h>
#include <poll.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/offboard_control_setpoint.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/differential_pressure.h>
#include <mavlink/mavlink_log.h>

#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

/* XXX MOVE CALIBRATION TO SENSORS APP THREAD */
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>

#include "calibration_routines.h"


PARAM_DEFINE_INT32(SYS_FAILSAVE_LL, 0);	/**< Go into low-level failsafe after 0 ms */
//PARAM_DEFINE_INT32(SYS_FAILSAVE_HL, 0);	/**< Go into high-level failsafe after 0 ms */
PARAM_DEFINE_FLOAT(TRIM_ROLL, 0.0f);
PARAM_DEFINE_FLOAT(TRIM_PITCH, 0.0f);
PARAM_DEFINE_FLOAT(TRIM_YAW, 0.0f);

#include <systemlib/cpuload.h>
extern struct system_load_s system_load;

/* Decouple update interval and hysteris counters, all depends on intervals */
#define COMMANDER_MONITORING_INTERVAL 50000
#define COMMANDER_MONITORING_LOOPSPERMSEC (1/(COMMANDER_MONITORING_INTERVAL/1000.0f))
#define LOW_VOLTAGE_BATTERY_COUNTER_LIMIT (LOW_VOLTAGE_BATTERY_HYSTERESIS_TIME_MS*COMMANDER_MONITORING_LOOPSPERMSEC)
#define CRITICAL_VOLTAGE_BATTERY_COUNTER_LIMIT (CRITICAL_VOLTAGE_BATTERY_HYSTERESIS_TIME_MS*COMMANDER_MONITORING_LOOPSPERMSEC)

#define STICK_ON_OFF_LIMIT 0.75f
#define STICK_THRUST_RANGE 1.0f
#define STICK_ON_OFF_HYSTERESIS_TIME_MS 1000
#define STICK_ON_OFF_COUNTER_LIMIT (STICK_ON_OFF_HYSTERESIS_TIME_MS*COMMANDER_MONITORING_LOOPSPERMSEC)

#define GPS_FIX_TYPE_2D 2
#define GPS_FIX_TYPE_3D 3
#define GPS_QUALITY_GOOD_HYSTERIS_TIME_MS 5000
#define GPS_QUALITY_GOOD_COUNTER_LIMIT (GPS_QUALITY_GOOD_HYSTERIS_TIME_MS*COMMANDER_MONITORING_LOOPSPERMSEC)

/* File descriptors */
static int leds;
static int buzzer;
static int mavlink_fd;
static bool commander_initialized = false;
static struct vehicle_status_s current_status; /**< Main state machine */
static orb_advert_t stat_pub;

// static uint16_t nofix_counter = 0;
// static uint16_t gotfix_counter = 0;

static unsigned int failsafe_lowlevel_timeout_ms;

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

/* pthread loops */
static void *orb_receive_loop(void *arg);

__EXPORT int commander_main(int argc, char *argv[]);

/**
 * Mainloop of commander.
 */
int commander_thread_main(int argc, char *argv[]);

static int buzzer_init(void);
static void buzzer_deinit(void);
static int led_init(void);
static void led_deinit(void);
static int led_toggle(int led);
static int led_on(int led);
static int led_off(int led);
static void do_gyro_calibration(int status_pub, struct vehicle_status_s *status);
static void do_mag_calibration(int status_pub, struct vehicle_status_s *status);
static void do_rc_calibration(int status_pub, struct vehicle_status_s *status);
static void do_accel_calibration(int status_pub, struct vehicle_status_s *status);
static void handle_command(int status_pub, struct vehicle_status_s *current_status, struct vehicle_command_s *cmd);

int trigger_audio_alarm(uint8_t old_mode, uint8_t old_state, uint8_t new_mode, uint8_t new_state);



/**
 * Print the correct usage.
 */
static void usage(const char *reason);

/**
 * Sort calibration values.
 *
 * Sorts the calibration values with bubble sort.
 *
 * @param a 	The array to sort
 * @param n 	The number of entries in the array
 */
// static void cal_bsort(float a[], int n);

static int buzzer_init()
{
	buzzer = open("/dev/tone_alarm", O_WRONLY);

	if (buzzer < 0) {
		warnx("Buzzer: open fail\n");
		return ERROR;
	}

	return 0;
}

static void buzzer_deinit()
{
	close(buzzer);
}


static int led_init()
{
	leds = open(LED_DEVICE_PATH, 0);

	if (leds < 0) {
		warnx("LED: open fail\n");
		return ERROR;
	}

	if (ioctl(leds, LED_ON, LED_BLUE) || ioctl(leds, LED_ON, LED_AMBER)) {
		warnx("LED: ioctl fail\n");
		return ERROR;
	}

	return 0;
}

static void led_deinit()
{
	close(leds);
}

static int led_toggle(int led)
{
	static int last_blue = LED_ON;
	static int last_amber = LED_ON;

	if (led == LED_BLUE) last_blue = (last_blue == LED_ON) ? LED_OFF : LED_ON;

	if (led == LED_AMBER) last_amber = (last_amber == LED_ON) ? LED_OFF : LED_ON;

	return ioctl(leds, ((led == LED_BLUE) ? last_blue : last_amber), led);
}

static int led_on(int led)
{
	return ioctl(leds, LED_ON, led);
}

static int led_off(int led)
{
	return ioctl(leds, LED_OFF, led);
}

enum AUDIO_PATTERN {
	AUDIO_PATTERN_ERROR = 2,
	AUDIO_PATTERN_NOTIFY_POSITIVE = 3,
	AUDIO_PATTERN_NOTIFY_NEUTRAL = 4,
	AUDIO_PATTERN_NOTIFY_NEGATIVE = 5,
	AUDIO_PATTERN_NOTIFY_CHARGE = 6
};

int trigger_audio_alarm(uint8_t old_mode, uint8_t old_state, uint8_t new_mode, uint8_t new_state)
{

	/* Trigger alarm if going into any error state */
	if (((new_state == SYSTEM_STATE_GROUND_ERROR) && (old_state != SYSTEM_STATE_GROUND_ERROR)) ||
	    ((new_state == SYSTEM_STATE_MISSION_ABORT) && (old_state != SYSTEM_STATE_MISSION_ABORT))) {
		ioctl(buzzer, TONE_SET_ALARM, 0);
		ioctl(buzzer, TONE_SET_ALARM, AUDIO_PATTERN_ERROR);
	}

	/* Trigger neutral on arming / disarming */
	if (((new_state == SYSTEM_STATE_GROUND_READY) && (old_state != SYSTEM_STATE_GROUND_READY))) {
		ioctl(buzzer, TONE_SET_ALARM, 0);
		ioctl(buzzer, TONE_SET_ALARM, AUDIO_PATTERN_NOTIFY_NEUTRAL);
	}

	/* Trigger Tetris on being bored */

	return 0;
}

void tune_confirm(void)
{
	ioctl(buzzer, TONE_SET_ALARM, 3);
}

void tune_error(void)
{
	ioctl(buzzer, TONE_SET_ALARM, 4);
}

void do_rc_calibration(int status_pub, struct vehicle_status_s *status)
{
	if (current_status.offboard_control_signal_lost) {
		mavlink_log_critical(mavlink_fd, "TRIM CAL: ABORT. No RC signal.");
		return;
	}

	int sub_man = orb_subscribe(ORB_ID(manual_control_setpoint));
	struct manual_control_setpoint_s sp;
	orb_copy(ORB_ID(manual_control_setpoint), sub_man, &sp);

	/* set parameters */

	float p = sp.roll;
	param_set(param_find("TRIM_ROLL"), &p);
	p = sp.pitch;
	param_set(param_find("TRIM_PITCH"), &p);
	p = sp.yaw;
	param_set(param_find("TRIM_YAW"), &p);

	/* store to permanent storage */
	/* auto-save to EEPROM */
	int save_ret = param_save_default();

	if (save_ret != 0) {
		mavlink_log_critical(mavlink_fd, "TRIM CAL: WARN: auto-save of params failed");
	}

	mavlink_log_info(mavlink_fd, "trim calibration done");
}

void do_mag_calibration(int status_pub, struct vehicle_status_s *status)
{

	/* set to mag calibration mode */
	status->flag_preflight_mag_calibration = true;
	state_machine_publish(status_pub, status, mavlink_fd);

	int sub_mag = orb_subscribe(ORB_ID(sensor_mag));
	struct mag_report mag;

	/* 45 seconds */
	uint64_t calibration_interval = 45 * 1000 * 1000;

	/* maximum 2000 values */
	const unsigned int calibration_maxcount = 500;
	unsigned int calibration_counter = 0;

	/* limit update rate to get equally spaced measurements over time (in ms) */
	orb_set_interval(sub_mag, (calibration_interval / 1000) / calibration_maxcount);

	// XXX old cal
	//  * FLT_MIN is not the most negative float number,
	//  * but the smallest number by magnitude float can
	//  * represent. Use -FLT_MAX to initialize the most
	//  * negative number

	// float mag_max[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
	// float mag_min[3] = {FLT_MAX, FLT_MAX, FLT_MAX};

	int fd = open(MAG_DEVICE_PATH, O_RDONLY);

	/* erase old calibration */
	struct mag_scale mscale_null = {
		0.0f,
		1.0f,
		0.0f,
		1.0f,
		0.0f,
		1.0f,
	};

	if (OK != ioctl(fd, MAGIOCSSCALE, (long unsigned int)&mscale_null)) {
		warn("WARNING: failed to set scale / offsets for mag");
		mavlink_log_info(mavlink_fd, "failed to set scale / offsets for mag");
	}

	/* calibrate range */
	if (OK != ioctl(fd, MAGIOCCALIBRATE, fd)) {
		warnx("failed to calibrate scale");
	}

	close(fd);

	/* calibrate offsets */

	// uint64_t calibration_start = hrt_absolute_time();

	uint64_t axis_deadline = hrt_absolute_time();
	uint64_t calibration_deadline = hrt_absolute_time() + calibration_interval;

	const char axislabels[3] = { 'X', 'Y', 'Z'};
	int axis_index = -1;

	float *x = (float *)malloc(sizeof(float) * calibration_maxcount);
	float *y = (float *)malloc(sizeof(float) * calibration_maxcount);
	float *z = (float *)malloc(sizeof(float) * calibration_maxcount);

	if (x == NULL || y == NULL || z == NULL) {
		warnx("mag cal failed: out of memory");
		mavlink_log_info(mavlink_fd, "mag cal failed: out of memory");
		warnx("x:%p y:%p z:%p\n", x, y, z);
		return;
	}

	tune_confirm();
	sleep(2);
	tune_confirm();

	while (hrt_absolute_time() < calibration_deadline &&
	       calibration_counter < calibration_maxcount) {

		/* wait blocking for new data */
		struct pollfd fds[1] = { { .fd = sub_mag, .events = POLLIN } };

		/* user guidance */
		if (hrt_absolute_time() >= axis_deadline &&
		    axis_index < 3) {

			axis_index++;

			char buf[50];
			sprintf(buf, "Please rotate around %c", axislabels[axis_index]);
			mavlink_log_info(mavlink_fd, buf);
			tune_confirm();

			axis_deadline += calibration_interval / 3;
		}

		if (!(axis_index < 3)) {
			break;
		}

		// int axis_left = (int64_t)axis_deadline - (int64_t)hrt_absolute_time();

		// if ((axis_left / 1000) == 0 && axis_left > 0) {
		// 	char buf[50];
		// 	sprintf(buf, "[cmd] %d seconds left for axis %c", axis_left, axislabels[axis_index]);
		// 	mavlink_log_info(mavlink_fd, buf);
		// }

		int poll_ret = poll(fds, 1, 1000);

		if (poll_ret) {
			orb_copy(ORB_ID(sensor_mag), sub_mag, &mag);

			x[calibration_counter] = mag.x;
			y[calibration_counter] = mag.y;
			z[calibration_counter] = mag.z;

			/* get min/max values */

			// if (mag.x < mag_min[0]) {
			// 	mag_min[0] = mag.x;
			// }
			// else if (mag.x > mag_max[0]) {
			// 	mag_max[0] = mag.x;
			// }

			// if (raw.magnetometer_ga[1] < mag_min[1]) {
			// 	mag_min[1] = raw.magnetometer_ga[1];
			// }
			// else if (raw.magnetometer_ga[1] > mag_max[1]) {
			// 	mag_max[1] = raw.magnetometer_ga[1];
			// }

			// if (raw.magnetometer_ga[2] < mag_min[2]) {
			// 	mag_min[2] = raw.magnetometer_ga[2];
			// }
			// else if (raw.magnetometer_ga[2] > mag_max[2]) {
			// 	mag_max[2] = raw.magnetometer_ga[2];
			// }

			calibration_counter++;

		} else if (poll_ret == 0) {
			/* any poll failure for 1s is a reason to abort */
			mavlink_log_info(mavlink_fd, "mag cal canceled (timed out)");
			break;
		}
	}

	float sphere_x;
	float sphere_y;
	float sphere_z;
	float sphere_radius;

	sphere_fit_least_squares(x, y, z, calibration_counter, 100, 0.0f, &sphere_x, &sphere_y, &sphere_z, &sphere_radius);

	free(x);
	free(y);
	free(z);

	if (isfinite(sphere_x) && isfinite(sphere_y) && isfinite(sphere_z)) {

		fd = open(MAG_DEVICE_PATH, 0);

		struct mag_scale mscale;

		if (OK != ioctl(fd, MAGIOCGSCALE, (long unsigned int)&mscale))
			warn("WARNING: failed to get scale / offsets for mag");

		mscale.x_offset = sphere_x;
		mscale.y_offset = sphere_y;
		mscale.z_offset = sphere_z;

		if (OK != ioctl(fd, MAGIOCSSCALE, (long unsigned int)&mscale))
			warn("WARNING: failed to set scale / offsets for mag");

		close(fd);

		/* announce and set new offset */

		if (param_set(param_find("SENS_MAG_XOFF"), &(mscale.x_offset))) {
			warnx("Setting X mag offset failed!\n");
		}

		if (param_set(param_find("SENS_MAG_YOFF"), &(mscale.y_offset))) {
			warnx("Setting Y mag offset failed!\n");
		}

		if (param_set(param_find("SENS_MAG_ZOFF"), &(mscale.z_offset))) {
			warnx("Setting Z mag offset failed!\n");
		}

		if (param_set(param_find("SENS_MAG_XSCALE"), &(mscale.x_scale))) {
			warnx("Setting X mag scale failed!\n");
		}

		if (param_set(param_find("SENS_MAG_YSCALE"), &(mscale.y_scale))) {
			warnx("Setting Y mag scale failed!\n");
		}

		if (param_set(param_find("SENS_MAG_ZSCALE"), &(mscale.z_scale))) {
			warnx("Setting Z mag scale failed!\n");
		}

		/* auto-save to EEPROM */
		int save_ret = param_save_default();

		if (save_ret != 0) {
			warn("WARNING: auto-save of params to storage failed");
			mavlink_log_info(mavlink_fd, "FAILED storing calibration");
		}

		warnx("\tscale: %.6f %.6f %.6f\n         \toffset: %.6f %.6f %.6f\nradius: %.6f GA\n",
		       (double)mscale.x_scale, (double)mscale.y_scale, (double)mscale.z_scale,
		       (double)mscale.x_offset, (double)mscale.y_offset, (double)mscale.z_offset, (double)sphere_radius);

		char buf[52];
		sprintf(buf, "mag off: x:%.2f y:%.2f z:%.2f Ga", (double)mscale.x_offset,
			(double)mscale.y_offset, (double)mscale.z_offset);
		mavlink_log_info(mavlink_fd, buf);

		sprintf(buf, "mag scale: x:%.2f y:%.2f z:%.2f", (double)mscale.x_scale,
			(double)mscale.y_scale, (double)mscale.z_scale);
		mavlink_log_info(mavlink_fd, buf);

		mavlink_log_info(mavlink_fd, "mag calibration done");

		tune_confirm();
		sleep(2);
		tune_confirm();
		sleep(2);
		/* third beep by cal end routine */

	} else {
		mavlink_log_info(mavlink_fd, "mag calibration FAILED (NaN in sphere fit)");
	}

	/* disable calibration mode */
	status->flag_preflight_mag_calibration = false;
	state_machine_publish(status_pub, status, mavlink_fd);

	close(sub_mag);
}

void do_gyro_calibration(int status_pub, struct vehicle_status_s *status)
{
	/* set to gyro calibration mode */
	status->flag_preflight_gyro_calibration = true;
	state_machine_publish(status_pub, status, mavlink_fd);

	const int calibration_count = 5000;

	int sub_sensor_combined = orb_subscribe(ORB_ID(sensor_combined));
	struct sensor_combined_s raw;

	int calibration_counter = 0;
	float gyro_offset[3] = {0.0f, 0.0f, 0.0f};

	/* set offsets to zero */
	int fd = open(GYRO_DEVICE_PATH, 0);
	struct gyro_scale gscale_null = {
		0.0f,
		1.0f,
		0.0f,
		1.0f,
		0.0f,
		1.0f,
	};

	if (OK != ioctl(fd, GYROIOCSSCALE, (long unsigned int)&gscale_null))
		warn("WARNING: failed to set scale / offsets for gyro");

	close(fd);

	while (calibration_counter < calibration_count) {

		/* wait blocking for new data */
		struct pollfd fds[1] = { { .fd = sub_sensor_combined, .events = POLLIN } };

		int poll_ret = poll(fds, 1, 1000);

		if (poll_ret) {
			orb_copy(ORB_ID(sensor_combined), sub_sensor_combined, &raw);
			gyro_offset[0] += raw.gyro_rad_s[0];
			gyro_offset[1] += raw.gyro_rad_s[1];
			gyro_offset[2] += raw.gyro_rad_s[2];
			calibration_counter++;

		} else if (poll_ret == 0) {
			/* any poll failure for 1s is a reason to abort */
			mavlink_log_info(mavlink_fd, "gyro calibration aborted, retry");
			return;
		}
	}

	gyro_offset[0] = gyro_offset[0] / calibration_count;
	gyro_offset[1] = gyro_offset[1] / calibration_count;
	gyro_offset[2] = gyro_offset[2] / calibration_count;

	/* exit gyro calibration mode */
	status->flag_preflight_gyro_calibration = false;
	state_machine_publish(status_pub, status, mavlink_fd);

	if (isfinite(gyro_offset[0]) && isfinite(gyro_offset[1]) && isfinite(gyro_offset[2])) {

		if (param_set(param_find("SENS_GYRO_XOFF"), &(gyro_offset[0]))
			|| param_set(param_find("SENS_GYRO_YOFF"), &(gyro_offset[1]))
			|| param_set(param_find("SENS_GYRO_ZOFF"), &(gyro_offset[2]))) {
			mavlink_log_critical(mavlink_fd, "Setting gyro offsets failed!");
		}

		/* set offsets to actual value */
		fd = open(GYRO_DEVICE_PATH, 0);
		struct gyro_scale gscale = {
			gyro_offset[0],
			1.0f,
			gyro_offset[1],
			1.0f,
			gyro_offset[2],
			1.0f,
		};

		if (OK != ioctl(fd, GYROIOCSSCALE, (long unsigned int)&gscale))
			warn("WARNING: failed to set scale / offsets for gyro");

		close(fd);

		/* auto-save to EEPROM */
		int save_ret = param_save_default();

		if (save_ret != 0) {
			warn("WARNING: auto-save of params to storage failed");
		}

		// char buf[50];
		// sprintf(buf, "cal: x:%8.4f y:%8.4f z:%8.4f", (double)gyro_offset[0], (double)gyro_offset[1], (double)gyro_offset[2]);
		// mavlink_log_info(mavlink_fd, buf);
		mavlink_log_info(mavlink_fd, "gyro calibration done");

		tune_confirm();
		sleep(2);
		tune_confirm();
		sleep(2);
		/* third beep by cal end routine */

	} else {
		mavlink_log_info(mavlink_fd, "gyro calibration FAILED (NaN)");
	}

	close(sub_sensor_combined);
}

void do_accel_calibration(int status_pub, struct vehicle_status_s *status)
{
	/* announce change */

	mavlink_log_info(mavlink_fd, "keep it level and still");
	/* set to accel calibration mode */
	status->flag_preflight_accel_calibration = true;
	state_machine_publish(status_pub, status, mavlink_fd);

	const int calibration_count = 2500;

	int sub_sensor_combined = orb_subscribe(ORB_ID(sensor_combined));
	struct sensor_combined_s raw;

	int calibration_counter = 0;
	float accel_offset[3] = {0.0f, 0.0f, 0.0f};

	int fd = open(ACCEL_DEVICE_PATH, 0);
	struct accel_scale ascale_null = {
		0.0f,
		1.0f,
		0.0f,
		1.0f,
		0.0f,
		1.0f,
	};

	if (OK != ioctl(fd, ACCELIOCSSCALE, (long unsigned int)&ascale_null))
		warn("WARNING: failed to set scale / offsets for accel");

	close(fd);

	while (calibration_counter < calibration_count) {

		/* wait blocking for new data */
		struct pollfd fds[1] = { { .fd = sub_sensor_combined, .events = POLLIN } };

		int poll_ret = poll(fds, 1, 1000);

		if (poll_ret) {
			orb_copy(ORB_ID(sensor_combined), sub_sensor_combined, &raw);
			accel_offset[0] += raw.accelerometer_m_s2[0];
			accel_offset[1] += raw.accelerometer_m_s2[1];
			accel_offset[2] += raw.accelerometer_m_s2[2];
			calibration_counter++;

		} else if (poll_ret == 0) {
			/* any poll failure for 1s is a reason to abort */
			mavlink_log_info(mavlink_fd, "acceleration calibration aborted");
			return;
		}
	}

	accel_offset[0] = accel_offset[0] / calibration_count;
	accel_offset[1] = accel_offset[1] / calibration_count;
	accel_offset[2] = accel_offset[2] / calibration_count;

	if (isfinite(accel_offset[0]) && isfinite(accel_offset[1]) && isfinite(accel_offset[2])) {

		/* add the removed length from x / y to z, since we induce a scaling issue else */
		float total_len = sqrtf(accel_offset[0] * accel_offset[0] + accel_offset[1] * accel_offset[1] + accel_offset[2] * accel_offset[2]);

		/* if length is correct, zero results here */
		accel_offset[2] = accel_offset[2] + total_len;

		float scale = 9.80665f / total_len;

		if (param_set(param_find("SENS_ACC_XOFF"), &(accel_offset[0]))
			|| param_set(param_find("SENS_ACC_YOFF"), &(accel_offset[1]))
			|| param_set(param_find("SENS_ACC_ZOFF"), &(accel_offset[2]))
			|| param_set(param_find("SENS_ACC_XSCALE"), &(scale))
			|| param_set(param_find("SENS_ACC_YSCALE"), &(scale))
			|| param_set(param_find("SENS_ACC_ZSCALE"), &(scale))) {
			mavlink_log_critical(mavlink_fd, "Setting offs or scale failed!");
		}

		fd = open(ACCEL_DEVICE_PATH, 0);
		struct accel_scale ascale = {
			accel_offset[0],
			scale,
			accel_offset[1],
			scale,
			accel_offset[2],
			scale,
		};

		if (OK != ioctl(fd, ACCELIOCSSCALE, (long unsigned int)&ascale))
			warn("WARNING: failed to set scale / offsets for accel");

		close(fd);

		/* auto-save to EEPROM */
		int save_ret = param_save_default();

		if (save_ret != 0) {
			warn("WARNING: auto-save of params to storage failed");
		}

		//char buf[50];
		//sprintf(buf, "[cmd] accel cal: x:%8.4f y:%8.4f z:%8.4f\n", (double)accel_offset[0], (double)accel_offset[1], (double)accel_offset[2]);
		//mavlink_log_info(mavlink_fd, buf);
		mavlink_log_info(mavlink_fd, "accel calibration done");

		tune_confirm();
		sleep(2);
		tune_confirm();
		sleep(2);
		/* third beep by cal end routine */

	} else {
		mavlink_log_info(mavlink_fd, "accel calibration FAILED (NaN)");
	}

	/* exit accel calibration mode */
	status->flag_preflight_accel_calibration = false;
	state_machine_publish(status_pub, status, mavlink_fd);

	close(sub_sensor_combined);
}

void do_airspeed_calibration(int status_pub, struct vehicle_status_s *status)
{
	/* announce change */

	mavlink_log_info(mavlink_fd, "keep it still");
	/* set to accel calibration mode */
	status->flag_preflight_airspeed_calibration = true;
	state_machine_publish(status_pub, status, mavlink_fd);

	const int calibration_count = 2500;

	int sub_differential_pressure = orb_subscribe(ORB_ID(differential_pressure));
	struct differential_pressure_s differential_pressure;

	int calibration_counter = 0;
	float airspeed_offset = 0.0f;

	while (calibration_counter < calibration_count) {

		/* wait blocking for new data */
		struct pollfd fds[1] = { { .fd = sub_differential_pressure, .events = POLLIN } };

		int poll_ret = poll(fds, 1, 1000);

		if (poll_ret) {
			orb_copy(ORB_ID(differential_pressure), sub_differential_pressure, &differential_pressure);
			airspeed_offset += differential_pressure.voltage;
			calibration_counter++;

		} else if (poll_ret == 0) {
			/* any poll failure for 1s is a reason to abort */
			mavlink_log_info(mavlink_fd, "airspeed calibration aborted");
			return;
		}
	}

	airspeed_offset = airspeed_offset / calibration_count;

	if (isfinite(airspeed_offset)) {

		if (param_set(param_find("SENS_VAIR_OFF"), &(airspeed_offset))) {
			mavlink_log_critical(mavlink_fd, "Setting offs failed!");
		}

		/* auto-save to EEPROM */
		int save_ret = param_save_default();

		if (save_ret != 0) {
			warn("WARNING: auto-save of params to storage failed");
		}

		//char buf[50];
		//sprintf(buf, "[cmd] accel cal: x:%8.4f y:%8.4f z:%8.4f\n", (double)accel_offset[0], (double)accel_offset[1], (double)accel_offset[2]);
		//mavlink_log_info(mavlink_fd, buf);
		mavlink_log_info(mavlink_fd, "airspeed calibration done");

		tune_confirm();
		sleep(2);
		tune_confirm();
		sleep(2);
		/* third beep by cal end routine */

	} else {
		mavlink_log_info(mavlink_fd, "airspeed calibration FAILED (NaN)");
	}

	/* exit airspeed calibration mode */
	status->flag_preflight_airspeed_calibration = false;
	state_machine_publish(status_pub, status, mavlink_fd);

	close(sub_differential_pressure);
}



void handle_command(int status_pub, struct vehicle_status_s *current_vehicle_status, struct vehicle_command_s *cmd)
{
	/* result of the command */
	uint8_t result = VEHICLE_CMD_RESULT_UNSUPPORTED;

	/* announce command handling */
	tune_confirm();


	/* supported command handling start */

	/* request to set different system mode */
	switch (cmd->command) {
	case VEHICLE_CMD_DO_SET_MODE: {
			if (OK == update_state_machine_mode_request(status_pub, current_vehicle_status, mavlink_fd, (uint8_t)cmd->param1)) {
				result = VEHICLE_CMD_RESULT_ACCEPTED;

			} else {
				result = VEHICLE_CMD_RESULT_DENIED;
			}
		}
		break;

	case VEHICLE_CMD_COMPONENT_ARM_DISARM: {
			/* request to arm */
			if ((int)cmd->param1 == 1) {
				if (OK == update_state_machine_mode_request(status_pub, current_vehicle_status, mavlink_fd, VEHICLE_MODE_FLAG_SAFETY_ARMED)) {
					result = VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					result = VEHICLE_CMD_RESULT_DENIED;
				}

				/* request to disarm */

			} else if ((int)cmd->param1 == 0) {
				if (OK == update_state_machine_mode_request(status_pub, current_vehicle_status, mavlink_fd, VEHICLE_MODE_FLAG_SAFETY_ARMED)) {
					result = VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					result = VEHICLE_CMD_RESULT_DENIED;
				}
			}
		}
		break;

		/* request for an autopilot reboot */
	case VEHICLE_CMD_PREFLIGHT_REBOOT_SHUTDOWN: {
			if ((int)cmd->param1 == 1) {
				if (OK == do_state_update(status_pub, current_vehicle_status, mavlink_fd, SYSTEM_STATE_REBOOT)) {
					/* SPECIAL CASE: SYSTEM WILL NEVER RETURN HERE */
					result = VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					/* system may return here */
					result = VEHICLE_CMD_RESULT_DENIED;
				}
			}
		}
		break;

//		/* request to land */
//		case VEHICLE_CMD_NAV_LAND:
//		 {
//				//TODO: add check if landing possible
//				//TODO: add landing maneuver
//
//				if (0 == update_state_machine_custom_mode_request(status_pub, current_vehicle_status, SYSTEM_STATE_ARMED)) {
//					result = VEHICLE_CMD_RESULT_ACCEPTED;
//		}		}
//		break;
//
//		/* request to takeoff */
//		case VEHICLE_CMD_NAV_TAKEOFF:
//		{
//			//TODO: add check if takeoff possible
//			//TODO: add takeoff maneuver
//
//			if (0 == update_state_machine_custom_mode_request(status_pub, current_vehicle_status, SYSTEM_STATE_AUTO)) {
//				result = VEHICLE_CMD_RESULT_ACCEPTED;
//			}
//		}
//		break;
//
		/* preflight calibration */
	case VEHICLE_CMD_PREFLIGHT_CALIBRATION: {
			bool handled = false;

			/* gyro calibration */
			if ((int)(cmd->param1) == 1) {
				/* transition to calibration state */
				do_state_update(status_pub, &current_status, mavlink_fd, SYSTEM_STATE_PREFLIGHT);

				if (current_status.state_machine == SYSTEM_STATE_PREFLIGHT) {
					mavlink_log_info(mavlink_fd, "starting gyro cal");
					tune_confirm();
					do_gyro_calibration(status_pub, &current_status);
					mavlink_log_info(mavlink_fd, "finished gyro cal");
					tune_confirm();
					do_state_update(status_pub, &current_status, mavlink_fd, SYSTEM_STATE_STANDBY);
					result = VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					mavlink_log_critical(mavlink_fd, "REJECTING gyro cal");
					result = VEHICLE_CMD_RESULT_DENIED;
				}

				handled = true;
			}

			/* magnetometer calibration */
			if ((int)(cmd->param2) == 1) {
				/* transition to calibration state */
				do_state_update(status_pub, &current_status, mavlink_fd, SYSTEM_STATE_PREFLIGHT);

				if (current_status.state_machine == SYSTEM_STATE_PREFLIGHT) {
					mavlink_log_info(mavlink_fd, "starting mag cal");
					tune_confirm();
					do_mag_calibration(status_pub, &current_status);
					mavlink_log_info(mavlink_fd, "finished mag cal");
					tune_confirm();
					do_state_update(status_pub, &current_status, mavlink_fd, SYSTEM_STATE_STANDBY);
					result = VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					mavlink_log_critical(mavlink_fd, "REJECTING mag cal");
					result = VEHICLE_CMD_RESULT_DENIED;
				}

				handled = true;
			}

			/* zero-altitude pressure calibration */
			if ((int)(cmd->param3) == 1) {
				/* transition to calibration state */
				do_state_update(status_pub, &current_status, mavlink_fd, SYSTEM_STATE_PREFLIGHT);

				if (current_status.state_machine == SYSTEM_STATE_PREFLIGHT) {
					mavlink_log_info(mavlink_fd, "zero altitude cal. not implemented");
					tune_confirm();

				} else {
					mavlink_log_critical(mavlink_fd, "REJECTING altitude calibration");
					result = VEHICLE_CMD_RESULT_DENIED;
				}

				handled = true;
			}

			/* trim calibration */
			if ((int)(cmd->param4) == 1) {
				/* transition to calibration state */
				do_state_update(status_pub, &current_status, mavlink_fd, SYSTEM_STATE_PREFLIGHT);

				if (current_status.state_machine == SYSTEM_STATE_PREFLIGHT) {
					mavlink_log_info(mavlink_fd, "starting trim cal");
					tune_confirm();
					do_rc_calibration(status_pub, &current_status);
					mavlink_log_info(mavlink_fd, "finished trim cal");
					tune_confirm();
					do_state_update(status_pub, &current_status, mavlink_fd, SYSTEM_STATE_STANDBY);
					result = VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					mavlink_log_critical(mavlink_fd, "REJECTING trim cal");
					result = VEHICLE_CMD_RESULT_DENIED;
				}

				handled = true;
			}

			/* accel calibration */
			if ((int)(cmd->param5) == 1) {
				/* transition to calibration state */
				do_state_update(status_pub, &current_status, mavlink_fd, SYSTEM_STATE_PREFLIGHT);

				if (current_status.state_machine == SYSTEM_STATE_PREFLIGHT) {
					mavlink_log_info(mavlink_fd, "CMD starting accel cal");
					tune_confirm();
					do_accel_calibration(status_pub, &current_status);
					tune_confirm();
					mavlink_log_info(mavlink_fd, "CMD finished accel cal");
					do_state_update(status_pub, &current_status, mavlink_fd, SYSTEM_STATE_STANDBY);
					result = VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					mavlink_log_critical(mavlink_fd, "REJECTING accel cal");
					result = VEHICLE_CMD_RESULT_DENIED;
				}

				handled = true;
			}

			/* airspeed calibration */
			if ((int)(cmd->param6) == 1) { //xxx: this is not defined by the mavlink protocol
				/* transition to calibration state */
				do_state_update(status_pub, &current_status, mavlink_fd, SYSTEM_STATE_PREFLIGHT);

				if (current_status.state_machine == SYSTEM_STATE_PREFLIGHT) {
					mavlink_log_info(mavlink_fd, "CMD starting airspeed cal");
					tune_confirm();
					do_airspeed_calibration(status_pub, &current_status);
					tune_confirm();
					mavlink_log_info(mavlink_fd, "CMD finished airspeed cal");
					do_state_update(status_pub, &current_status, mavlink_fd, SYSTEM_STATE_STANDBY);
					result = VEHICLE_CMD_RESULT_ACCEPTED;

				} else {
					mavlink_log_critical(mavlink_fd, "REJECTING airspeed cal");
					result = VEHICLE_CMD_RESULT_DENIED;
				}

				handled = true;
			}

			/* none found */
			if (!handled) {
				//warnx("refusing unsupported calibration request\n");
				mavlink_log_critical(mavlink_fd, "CMD refusing unsup. calib. request");
				result = VEHICLE_CMD_RESULT_UNSUPPORTED;
			}
		}
		break;

	case VEHICLE_CMD_PREFLIGHT_STORAGE: {
			if (current_status.flag_system_armed &&
			    ((current_status.system_type == VEHICLE_TYPE_QUADROTOR) ||
			     (current_status.system_type == VEHICLE_TYPE_HEXAROTOR) ||
			     (current_status.system_type == VEHICLE_TYPE_OCTOROTOR))) {
				/* do not perform expensive memory tasks on multirotors in flight */
				// XXX this is over-safe, as soon as cmd is in low prio thread this can be allowed
				mavlink_log_info(mavlink_fd, "REJECTING save cmd while multicopter armed");

			} else {

				// XXX move this to LOW PRIO THREAD of commander app
				/* Read all parameters from EEPROM to RAM */

				if (((int)(cmd->param1)) == 0)	{

					/* read all parameters from EEPROM to RAM */
					int read_ret = param_load_default();

					if (read_ret == OK) {
						//warnx("[mavlink pm] Loaded EEPROM params in RAM\n");
						mavlink_log_info(mavlink_fd, "OK loading params from");
						mavlink_log_info(mavlink_fd, param_get_default_file());
						result = VEHICLE_CMD_RESULT_ACCEPTED;

					} else if (read_ret == 1) {
						mavlink_log_info(mavlink_fd, "OK no changes in");
						mavlink_log_info(mavlink_fd, param_get_default_file());
						result = VEHICLE_CMD_RESULT_ACCEPTED;

					} else {
						if (read_ret < -1) {
							mavlink_log_info(mavlink_fd, "ERR loading params from");
							mavlink_log_info(mavlink_fd, param_get_default_file());

						} else {
							mavlink_log_info(mavlink_fd, "ERR no param file named");
							mavlink_log_info(mavlink_fd, param_get_default_file());
						}

						result = VEHICLE_CMD_RESULT_FAILED;
					}

				} else if (((int)(cmd->param1)) == 1)	{

					/* write all parameters from RAM to EEPROM */
					int write_ret = param_save_default();

					if (write_ret == OK) {
						mavlink_log_info(mavlink_fd, "OK saved param file");
						mavlink_log_info(mavlink_fd, param_get_default_file());
						result = VEHICLE_CMD_RESULT_ACCEPTED;

					} else {
						if (write_ret < -1) {
							mavlink_log_info(mavlink_fd, "ERR params file does not exit:");
							mavlink_log_info(mavlink_fd, param_get_default_file());

						} else {
							mavlink_log_info(mavlink_fd, "ERR writing params to");
							mavlink_log_info(mavlink_fd, param_get_default_file());
						}

						result = VEHICLE_CMD_RESULT_FAILED;
					}

				} else {
					mavlink_log_info(mavlink_fd, "[pm] refusing unsupp. STOR request");
					result = VEHICLE_CMD_RESULT_UNSUPPORTED;
				}
			}
		}
		break;

	default: {
			mavlink_log_critical(mavlink_fd, "[cmd] refusing unsupported command");
			result = VEHICLE_CMD_RESULT_UNSUPPORTED;
			/* announce command rejection */
			ioctl(buzzer, TONE_SET_ALARM, 4);
		}
		break;
	}

	/* supported command handling stop */
	if (result == VEHICLE_CMD_RESULT_FAILED ||
	    result == VEHICLE_CMD_RESULT_DENIED ||
	    result == VEHICLE_CMD_RESULT_UNSUPPORTED) {
		ioctl(buzzer, TONE_SET_ALARM, 5);

	} else if (result == VEHICLE_CMD_RESULT_ACCEPTED) {
		tune_confirm();
	}

	/* send any requested ACKs */
	if (cmd->confirmation > 0) {
		/* send acknowledge command */
		// XXX TODO
	}

}

static void *orb_receive_loop(void *arg)  //handles status information coming from subsystems (present, enabled, health), these values do not indicate the quality (variance) of the signal
{
	/* Set thread name */
	prctl(PR_SET_NAME, "commander orb rcv", getpid());

	/* Subscribe to command topic */
	int subsys_sub = orb_subscribe(ORB_ID(subsystem_info));
	struct subsystem_info_s info;

	struct vehicle_status_s *vstatus = (struct vehicle_status_s *)arg;

	while (!thread_should_exit) {
		struct pollfd fds[1] = { { .fd = subsys_sub, .events = POLLIN } };

		if (poll(fds, 1, 5000) == 0) {
			/* timeout, but this is no problem, silently ignore */
		} else {
			/* got command */
			orb_copy(ORB_ID(subsystem_info), subsys_sub, &info);

			warnx("Subsys changed: %d\n", (int)info.subsystem_type);

			/* mark / unmark as present */
			if (info.present) {
				vstatus->onboard_control_sensors_present |= info.subsystem_type;

			} else {
				vstatus->onboard_control_sensors_present &= ~info.subsystem_type;
			}

			/* mark / unmark as enabled */
			if (info.enabled) {
				vstatus->onboard_control_sensors_enabled |= info.subsystem_type;

			} else {
				vstatus->onboard_control_sensors_enabled &= ~info.subsystem_type;
			}

			/* mark / unmark as ok */
			if (info.ok) {
				vstatus->onboard_control_sensors_health |= info.subsystem_type;

			} else {
				vstatus->onboard_control_sensors_health &= ~info.subsystem_type;
			}
		}
	}

	close(subsys_sub);

	return NULL;
}

/*
 * Provides a coarse estimate of remaining battery power.
 *
 * The estimate is very basic and based on decharging voltage curves.
 *
 * @return the estimated remaining capacity in 0..1
 */
float battery_remaining_estimate_voltage(float voltage);

PARAM_DEFINE_FLOAT(BAT_V_EMPTY, 3.2f);
PARAM_DEFINE_FLOAT(BAT_V_FULL, 4.05f);
PARAM_DEFINE_FLOAT(BAT_N_CELLS, 3);

float battery_remaining_estimate_voltage(float voltage)
{
	float ret = 0;
	static param_t bat_volt_empty;
	static param_t bat_volt_full;
	static param_t bat_n_cells;
	static bool initialized = false;
	static unsigned int counter = 0;
	static float ncells = 3;
	// XXX change cells to int (and param to INT32)

	if (!initialized) {
		bat_volt_empty = param_find("BAT_V_EMPTY");
		bat_volt_full = param_find("BAT_V_FULL");
		bat_n_cells = param_find("BAT_N_CELLS");
		initialized = true;
	}

	static float chemistry_voltage_empty = 3.2f;
	static float chemistry_voltage_full = 4.05f;

	if (counter % 100 == 0) {
		param_get(bat_volt_empty, &chemistry_voltage_empty);
		param_get(bat_volt_full, &chemistry_voltage_full);
		param_get(bat_n_cells, &ncells);
	}

	counter++;

	ret = (voltage - ncells * chemistry_voltage_empty) / (ncells * (chemistry_voltage_full - chemistry_voltage_empty));

	/* limit to sane values */
	ret = (ret < 0) ? 0 : ret;
	ret = (ret > 1) ? 1 : ret;
	return ret;
}

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr, "usage: daemon {start|stop|status} [-p <additional params>]\n\n");
	exit(1);
}

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int commander_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("commander already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		daemon_task = task_spawn("commander",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 40,
					 3000,
					 commander_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\tcommander is running\n");

		} else {
			warnx("\tcommander not started\n");
		}

		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int commander_thread_main(int argc, char *argv[])
{
	/* not yet initialized */
	commander_initialized = false;
	bool home_position_set = false;

	/* set parameters */
	failsafe_lowlevel_timeout_ms = 0;
	param_get(param_find("SYS_FAILSAVE_LL"), &failsafe_lowlevel_timeout_ms);

	param_t _param_sys_type = param_find("MAV_TYPE");
	param_t _param_system_id = param_find("MAV_SYS_ID");
	param_t _param_component_id = param_find("MAV_COMP_ID");

	/* welcome user */
	warnx("I am in command now!\n");

	/* pthreads for command and subsystem info handling */
	// pthread_t command_handling_thread;
	pthread_t subsystem_info_thread;

	/* initialize */
	if (led_init() != 0) {
		warnx("ERROR: Failed to initialize leds\n");
	}

	if (buzzer_init() != 0) {
		warnx("ERROR: Failed to initialize buzzer\n");
	}

	mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	if (mavlink_fd < 0) {
		warnx("ERROR: Failed to open MAVLink log stream, start mavlink app first.\n");
	}

	/* make sure we are in preflight state */
	memset(&current_status, 0, sizeof(current_status));
	current_status.state_machine = SYSTEM_STATE_PREFLIGHT;
	current_status.flag_system_armed = false;
	/* neither manual nor offboard control commands have been received */
	current_status.offboard_control_signal_found_once = false;
	current_status.rc_signal_found_once = false;
	/* mark all signals lost as long as they haven't been found */
	current_status.rc_signal_lost = true;
	current_status.offboard_control_signal_lost = true;
	/* allow manual override initially */
	current_status.flag_external_manual_override_ok = true;
	/* flag position info as bad, do not allow auto mode */
	current_status.flag_vector_flight_mode_ok = false;
	/* set battery warning flag */
	current_status.battery_warning = VEHICLE_BATTERY_WARNING_NONE;

	/* advertise to ORB */
	stat_pub = orb_advertise(ORB_ID(vehicle_status), &current_status);
	/* publish current state machine */
	state_machine_publish(stat_pub, &current_status, mavlink_fd);

	/* home position */
	orb_advert_t home_pub = -1;
	struct home_position_s home;
	memset(&home, 0, sizeof(home));

	if (stat_pub < 0) {
		warnx("ERROR: orb_advertise for topic vehicle_status failed (uorb app running?).\n");
		warnx("exiting.");
		exit(ERROR);
	}

	mavlink_log_info(mavlink_fd, "system is running");

	/* create pthreads */
	pthread_attr_t subsystem_info_attr;
	pthread_attr_init(&subsystem_info_attr);
	pthread_attr_setstacksize(&subsystem_info_attr, 2048);
	pthread_create(&subsystem_info_thread, &subsystem_info_attr, orb_receive_loop, &current_status);

	/* Start monitoring loop */
	uint16_t counter = 0;
	uint8_t flight_env;

	/* Initialize to 0.0V */
	float battery_voltage = 0.0f;
	bool battery_voltage_valid = false;
	bool low_battery_voltage_actions_done = false;
	bool critical_battery_voltage_actions_done = false;
	uint8_t low_voltage_counter = 0;
	uint16_t critical_voltage_counter = 0;
	int16_t mode_switch_rc_value;
	float bat_remain = 1.0f;

	uint16_t stick_off_counter = 0;
	uint16_t stick_on_counter = 0;

	/* Subscribe to manual control data */
	int sp_man_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	struct manual_control_setpoint_s sp_man;
	memset(&sp_man, 0, sizeof(sp_man));

	/* Subscribe to offboard control data */
	int sp_offboard_sub = orb_subscribe(ORB_ID(offboard_control_setpoint));
	struct offboard_control_setpoint_s sp_offboard;
	memset(&sp_offboard, 0, sizeof(sp_offboard));

	int global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	struct vehicle_global_position_s global_position;
	memset(&global_position, 0, sizeof(global_position));
	uint64_t last_global_position_time = 0;

	int local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	struct vehicle_local_position_s local_position;
	memset(&local_position, 0, sizeof(local_position));
	uint64_t last_local_position_time = 0;

	/* 
	 * The home position is set based on GPS only, to prevent a dependency between
	 * position estimator and commander. RAW GPS is more than good enough for a
	 * non-flying vehicle.
	 */
	int gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	struct vehicle_gps_position_s gps_position;
	memset(&gps_position, 0, sizeof(gps_position));

	int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	struct sensor_combined_s sensors;
	memset(&sensors, 0, sizeof(sensors));

	int differential_pressure_sub = orb_subscribe(ORB_ID(differential_pressure));
	struct differential_pressure_s differential_pressure;
	memset(&differential_pressure, 0, sizeof(differential_pressure));
	uint64_t last_differential_pressure_time = 0;

	/* Subscribe to command topic */
	int cmd_sub = orb_subscribe(ORB_ID(vehicle_command));
	struct vehicle_command_s cmd;
	memset(&cmd, 0, sizeof(cmd));

	/* Subscribe to parameters changed topic */
	int param_changed_sub = orb_subscribe(ORB_ID(parameter_update));
	struct parameter_update_s param_changed;
	memset(&param_changed, 0, sizeof(param_changed));

	/* subscribe to battery topic */
	int battery_sub = orb_subscribe(ORB_ID(battery_status));
	struct battery_status_s battery;
	memset(&battery, 0, sizeof(battery));
	battery.voltage_v = 0.0f;

	// uint8_t vehicle_state_previous = current_status.state_machine;
	float voltage_previous = 0.0f;

	uint64_t last_idle_time = 0;

	/* now initialized */
	commander_initialized = true;
	thread_running = true;

	uint64_t start_time = hrt_absolute_time();
	uint64_t failsave_ll_start_time = 0;

	bool state_changed = true;
	bool param_init_forced = true;

	while (!thread_should_exit) {

		/* Get current values */
		bool new_data;
		orb_check(sp_man_sub, &new_data);

		if (new_data) {
			orb_copy(ORB_ID(manual_control_setpoint), sp_man_sub, &sp_man);
		}

		orb_check(sp_offboard_sub, &new_data);

		if (new_data) {
			orb_copy(ORB_ID(offboard_control_setpoint), sp_offboard_sub, &sp_offboard);
		}

		orb_check(sensor_sub, &new_data);

		if (new_data) {
			orb_copy(ORB_ID(sensor_combined), sensor_sub, &sensors);
		}

		orb_check(differential_pressure_sub, &new_data);

		if (new_data) {
			orb_copy(ORB_ID(differential_pressure), differential_pressure_sub, &differential_pressure);
			last_differential_pressure_time = differential_pressure.timestamp;
		}

		orb_check(cmd_sub, &new_data);

		if (new_data) {
			/* got command */
			orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);

			/* handle it */
			handle_command(stat_pub, &current_status, &cmd);
		}

		/* update parameters */
		orb_check(param_changed_sub, &new_data);

		if (new_data || param_init_forced) {
			param_init_forced = false;
			/* parameters changed */
			orb_copy(ORB_ID(parameter_update), param_changed_sub, &param_changed);


			/* update parameters */
			if (!current_status.flag_system_armed) {
				if (param_get(_param_sys_type, &(current_status.system_type)) != OK) {
					warnx("failed setting new system type");
				}

				/* disable manual override for all systems that rely on electronic stabilization */
				if (current_status.system_type == VEHICLE_TYPE_QUADROTOR ||
				    current_status.system_type == VEHICLE_TYPE_HEXAROTOR ||
				    current_status.system_type == VEHICLE_TYPE_OCTOROTOR) {
					current_status.flag_external_manual_override_ok = false;

				} else {
					current_status.flag_external_manual_override_ok = true;
				}

				/* check and update system / component ID */
				param_get(_param_system_id, &(current_status.system_id));
				param_get(_param_component_id, &(current_status.component_id));

			}
		}

		/* update global position estimate */
		orb_check(global_position_sub, &new_data);

		if (new_data) {
			/* position changed */
			orb_copy(ORB_ID(vehicle_global_position), global_position_sub, &global_position);
			last_global_position_time = global_position.timestamp;
		}

		/* update local position estimate */
		orb_check(local_position_sub, &new_data);

		if (new_data) {
			/* position changed */
			orb_copy(ORB_ID(vehicle_local_position), local_position_sub, &local_position);
			last_local_position_time = local_position.timestamp;
		}

		/* update battery status */
		orb_check(battery_sub, &new_data);
		if (new_data) {
			orb_copy(ORB_ID(battery_status), battery_sub, &battery);
			battery_voltage = battery.voltage_v;
			battery_voltage_valid = true;

			/*
			 * Only update battery voltage estimate if system has
			 * been running for two and a half seconds.
			 */
			if (hrt_absolute_time() - start_time > 2500000) {
				bat_remain = battery_remaining_estimate_voltage(battery_voltage);
			}
		}

		/* Slow but important 8 Hz checks */
		if (counter % ((1000000 / COMMANDER_MONITORING_INTERVAL) / 8) == 0) {
			/* toggle activity (blue) led at 1 Hz in standby, 10 Hz in armed mode */
			if ((current_status.state_machine == SYSTEM_STATE_GROUND_READY ||
			     current_status.state_machine == SYSTEM_STATE_AUTO  ||
			     current_status.state_machine == SYSTEM_STATE_MANUAL)) {
				/* armed */
				led_toggle(LED_BLUE);

			} else if (counter % (1000000 / COMMANDER_MONITORING_INTERVAL) == 0) {
				/* not armed */
				led_toggle(LED_BLUE);
			}

			/* toggle error led at 5 Hz in HIL mode */
			if (current_status.flag_hil_enabled) {
				/* hil enabled */
				led_toggle(LED_AMBER);

			} else if (bat_remain < 0.3f && (low_voltage_counter > LOW_VOLTAGE_BATTERY_COUNTER_LIMIT)) {
				/* toggle error (red) at 5 Hz on low battery or error */
				led_toggle(LED_AMBER);

			} else {
				// /* Constant error indication in standby mode without GPS */
				// if (!current_status.gps_valid) {
				// 	led_on(LED_AMBER);

				// } else {
				// 	led_off(LED_AMBER);
				// }
			}

			if (counter % (1000000 / COMMANDER_MONITORING_INTERVAL) == 0) {
				/* compute system load */
				uint64_t interval_runtime = system_load.tasks[0].total_runtime - last_idle_time;

				if (last_idle_time > 0)
					current_status.load = 1000 - (interval_runtime / 1000);	//system load is time spent in non-idle

				last_idle_time = system_load.tasks[0].total_runtime;
			}
		}

		// // XXX Export patterns and threshold to parameters
		/* Trigger audio event for low battery */
		if (bat_remain < 0.1f && battery_voltage_valid && (counter % ((1000000 / COMMANDER_MONITORING_INTERVAL) / 4) == 0)) {
			/* For less than 10%, start be really annoying at 5 Hz */
			ioctl(buzzer, TONE_SET_ALARM, 0);
			ioctl(buzzer, TONE_SET_ALARM, 3);

		} else if (bat_remain < 0.1f && battery_voltage_valid && (counter % ((1000000 / COMMANDER_MONITORING_INTERVAL) / 4) == 2)) {
			ioctl(buzzer, TONE_SET_ALARM, 0);

		} else if (bat_remain < 0.2f && battery_voltage_valid && (counter % ((1000000 / COMMANDER_MONITORING_INTERVAL) / 2) == 0)) {
			/* For less than 20%, start be slightly annoying at 1 Hz */
			ioctl(buzzer, TONE_SET_ALARM, 0);
			tune_confirm();

		} else if (bat_remain < 0.2f && battery_voltage_valid && (counter % ((1000000 / COMMANDER_MONITORING_INTERVAL) / 2) == 2)) {
			ioctl(buzzer, TONE_SET_ALARM, 0);
		}

		/* Check battery voltage */
		/* write to sys_status */
		if (battery_voltage_valid) {
			current_status.voltage_battery = battery_voltage;

		} else {
			current_status.voltage_battery = 0.0f;
		}

		/* if battery voltage is getting lower, warn using buzzer, etc. */
		if (battery_voltage_valid && (bat_remain < 0.15f /* XXX MAGIC NUMBER */) && (false == low_battery_voltage_actions_done)) { //TODO: add filter, or call emergency after n measurements < VOLTAGE_BATTERY_MINIMAL_MILLIVOLTS

			if (low_voltage_counter > LOW_VOLTAGE_BATTERY_COUNTER_LIMIT) {
				low_battery_voltage_actions_done = true;
				mavlink_log_critical(mavlink_fd, "[cmd] WARNING! LOW BATTERY!");
				current_status.battery_warning = VEHICLE_BATTERY_WARNING_WARNING;
			}

			low_voltage_counter++;
		}

		/* Critical, this is rather an emergency, kill signal to sdlog and change state machine */
		else if (battery_voltage_valid && (bat_remain < 0.1f /* XXX MAGIC NUMBER */) && (false == critical_battery_voltage_actions_done && true == low_battery_voltage_actions_done)) {
			if (critical_voltage_counter > CRITICAL_VOLTAGE_BATTERY_COUNTER_LIMIT) {
				critical_battery_voltage_actions_done = true;
				mavlink_log_critical(mavlink_fd, "[cmd] EMERGENCY! CRITICAL BATTERY!");
				current_status.battery_warning = VEHICLE_BATTERY_WARNING_ALERT;
				state_machine_emergency(stat_pub, &current_status, mavlink_fd);
			}

			critical_voltage_counter++;

		} else {
			low_voltage_counter = 0;
			critical_voltage_counter = 0;
		}

		/* End battery voltage check */


		/*
		 * Check for valid position information.
		 *
		 * If the system has a valid position source from an onboard
		 * position estimator, it is safe to operate it autonomously.
		 * The flag_vector_flight_mode_ok flag indicates that a minimum
		 * set of position measurements is available.
		 */

		/* store current state to reason later about a state change */
		bool vector_flight_mode_ok = current_status.flag_vector_flight_mode_ok;
		bool global_pos_valid = current_status.flag_global_position_valid;
		bool local_pos_valid = current_status.flag_local_position_valid;
		bool airspeed_valid = current_status.flag_airspeed_valid;

		/* check for global or local position updates, set a timeout of 2s */
		if (hrt_absolute_time() - last_global_position_time < 2000000) {
			current_status.flag_global_position_valid = true;
			// XXX check for controller status and home position as well

		} else {
			current_status.flag_global_position_valid = false;
		}

		if (hrt_absolute_time() - last_local_position_time < 2000000) {
			current_status.flag_local_position_valid = true;
			// XXX check for controller status and home position as well

		} else {
			current_status.flag_local_position_valid = false;
		}

		/* Check for valid airspeed/differential pressure measurements */
		if (hrt_absolute_time() - last_differential_pressure_time < 2000000) {
			current_status.flag_airspeed_valid = true;

		} else {
			current_status.flag_airspeed_valid = false;
		}

		/*
		 * Consolidate global position and local position valid flags
		 * for vector flight mode.
		 */
		if (current_status.flag_local_position_valid ||
		    current_status.flag_global_position_valid) {
			current_status.flag_vector_flight_mode_ok = true;

		} else {
			current_status.flag_vector_flight_mode_ok = false;
		}

		/* consolidate state change, flag as changed if required */
		if (vector_flight_mode_ok != current_status.flag_vector_flight_mode_ok ||
		    global_pos_valid != current_status.flag_global_position_valid ||
		    local_pos_valid != current_status.flag_local_position_valid ||
		    airspeed_valid != current_status.flag_airspeed_valid) {
			state_changed = true;
		}

		/*
		 * Mark the position of the first position lock as return to launch (RTL)
		 * position. The MAV will return here on command or emergency.
		 *
		 * Conditions:
		 *
		 * 	1) The system aquired position lock just now
		 *	2) The system has not aquired position lock before
		 *	3) The system is not armed (on the ground)
		 */
		if (!current_status.flag_valid_launch_position &&
		    !vector_flight_mode_ok && current_status.flag_vector_flight_mode_ok &&
		    !current_status.flag_system_armed) {
			/* first time a valid position, store it and emit it */

			// XXX implement storage and publication of RTL position
			current_status.flag_valid_launch_position = true;
			current_status.flag_auto_flight_mode_ok = true;
			state_changed = true;
		}

		if (orb_check(ORB_ID(vehicle_gps_position), &new_data)) {

			orb_copy(ORB_ID(vehicle_gps_position), gps_sub, &gps_position);

			/* check for first, long-term and valid GPS lock -> set home position */
			float hdop_m = gps_position.eph_m;
			float vdop_m = gps_position.epv_m;

			/* check if gps fix is ok */
			// XXX magic number
			float hdop_threshold_m = 4.0f;
			float vdop_threshold_m = 8.0f;

			/*
			 * If horizontal dilution of precision (hdop / eph)
			 * and vertical diluation of precision (vdop / epv)
			 * are below a certain threshold (e.g. 4 m), AND
			 * home position is not yet set AND the last GPS
			 * GPS measurement is not older than two seconds AND
			 * the system is currently not armed, set home
			 * position to the current position.
			 */

			if (gps_position.fix_type == GPS_FIX_TYPE_3D
				&& (hdop_m < hdop_threshold_m)
				&& (vdop_m < vdop_threshold_m)
				&& !home_position_set
				&& (hrt_absolute_time() - gps_position.timestamp_position < 2000000)
				&& !current_status.flag_system_armed) {
				warnx("setting home position");

				/* copy position data to uORB home message, store it locally as well */
				home.lat = gps_position.lat;
				home.lon = gps_position.lon;
				home.alt = gps_position.alt;

				home.eph_m = gps_position.eph_m;
				home.epv_m = gps_position.epv_m;

				home.s_variance_m_s = gps_position.s_variance_m_s;
				home.p_variance_m = gps_position.p_variance_m;

				/* announce new home position */
				if (home_pub > 0) {
					orb_publish(ORB_ID(home_position), home_pub, &home);
				} else {
					home_pub = orb_advertise(ORB_ID(home_position), &home);
				}

				/* mark home position as set */
				home_position_set = true;
				tune_confirm();
			}
		}

		/* ignore RC signals if in offboard control mode */
		if (!current_status.offboard_control_signal_found_once && sp_man.timestamp != 0) {
			/* Start RC state check */

			if ((hrt_absolute_time() - sp_man.timestamp) < 100000) {

				// /*
				//  * Check if manual control modes have to be switched
				//  */
				// if (!isfinite(sp_man.manual_mode_switch)) {
				// 	warnx("man mode sw not finite\n");

				// 	/* this switch is not properly mapped, set default */
				// 	if ((current_status.system_type == VEHICLE_TYPE_QUADROTOR) ||
				// 		(current_status.system_type == VEHICLE_TYPE_HEXAROTOR) ||
				// 		(current_status.system_type == VEHICLE_TYPE_OCTOROTOR)) {

				// 		/* assuming a rotary wing, fall back to SAS */
				// 		current_status.manual_control_mode = VEHICLE_MANUAL_CONTROL_MODE_SAS;
				// 		current_status.flag_control_attitude_enabled = true;
				// 		current_status.flag_control_rates_enabled = true;
				// 	} else {

				// 		/* assuming a fixed wing, fall back to direct pass-through */
				// 		current_status.manual_control_mode = VEHICLE_MANUAL_CONTROL_MODE_DIRECT;
				// 		current_status.flag_control_attitude_enabled = false;
				// 		current_status.flag_control_rates_enabled = false;
				// 	}

				// } else if (sp_man.manual_mode_switch < -STICK_ON_OFF_LIMIT) {

				// 	/* bottom stick position, set direct mode for vehicles supporting it */
				// 	if ((current_status.system_type == VEHICLE_TYPE_QUADROTOR) ||
				// 		(current_status.system_type == VEHICLE_TYPE_HEXAROTOR) ||
				// 		(current_status.system_type == VEHICLE_TYPE_OCTOROTOR)) {

				// 		/* assuming a rotary wing, fall back to SAS */
				// 		current_status.manual_control_mode = VEHICLE_MANUAL_CONTROL_MODE_SAS;
				// 		current_status.flag_control_attitude_enabled = true;
				// 		current_status.flag_control_rates_enabled = true;
				// 	} else {

				// 		/* assuming a fixed wing, set to direct pass-through as requested */
				// 		current_status.manual_control_mode = VEHICLE_MANUAL_CONTROL_MODE_DIRECT;
				// 		current_status.flag_control_attitude_enabled = false;
				// 		current_status.flag_control_rates_enabled = false;
				// 	}

				// } else if (sp_man.manual_mode_switch > STICK_ON_OFF_LIMIT) {

				// 	/* top stick position, set SAS for all vehicle types */
				// 	current_status.manual_control_mode = VEHICLE_MANUAL_CONTROL_MODE_SAS;
				// 	current_status.flag_control_attitude_enabled = true;
				// 	current_status.flag_control_rates_enabled = true;

				// } else {
				// 	/* center stick position, set rate control */
				// 	current_status.manual_control_mode = VEHICLE_MANUAL_CONTROL_MODE_RATES;
				// 	current_status.flag_control_attitude_enabled = false;
				// 	current_status.flag_control_rates_enabled = true;
				// }

				// warnx("man ctrl mode: %d\n", (int)current_status.manual_control_mode);

				/*
				 * Check if manual stability control modes have to be switched
				 */
				if (!isfinite(sp_man.manual_sas_switch)) {

					/* this switch is not properly mapped, set default */
					current_status.manual_sas_mode = VEHICLE_MANUAL_SAS_MODE_ROLL_PITCH_ABS_YAW_ABS;

				} else if (sp_man.manual_sas_switch < -STICK_ON_OFF_LIMIT) {

					/* bottom stick position, set altitude hold */
					current_status.manual_sas_mode = VEHICLE_MANUAL_SAS_MODE_ALTITUDE;

				} else if (sp_man.manual_sas_switch > STICK_ON_OFF_LIMIT) {

					/* top stick position */
					current_status.manual_sas_mode = VEHICLE_MANUAL_SAS_MODE_SIMPLE;

				} else {
					/* center stick position, set default */
					current_status.manual_sas_mode = VEHICLE_MANUAL_SAS_MODE_ROLL_PITCH_ABS_YAW_ABS;
				}

				/*
				 * Check if left stick is in lower left position --> switch to standby state.
				 * Do this only for multirotors, not for fixed wing aircraft.
				 */
				if (((current_status.system_type == VEHICLE_TYPE_QUADROTOR) ||
				     (current_status.system_type == VEHICLE_TYPE_HEXAROTOR) ||
				     (current_status.system_type == VEHICLE_TYPE_OCTOROTOR)
				    ) &&
				    ((sp_man.yaw < -STICK_ON_OFF_LIMIT)) &&
				    (sp_man.throttle < STICK_THRUST_RANGE * 0.2f)) {
					if (stick_off_counter > STICK_ON_OFF_COUNTER_LIMIT) {
						update_state_machine_disarm(stat_pub, &current_status, mavlink_fd);
						stick_on_counter = 0;

					} else {
						stick_off_counter++;
						stick_on_counter = 0;
					}
				}

				/* check if left stick is in lower right position --> arm */
				if (sp_man.yaw > STICK_ON_OFF_LIMIT && sp_man.throttle < STICK_THRUST_RANGE * 0.2f) {
					if (stick_on_counter > STICK_ON_OFF_COUNTER_LIMIT) {
						update_state_machine_arm(stat_pub, &current_status, mavlink_fd);
						stick_on_counter = 0;

					} else {
						stick_on_counter++;
						stick_off_counter = 0;
					}
				}

				/* check manual override switch - switch to manual or auto mode */
				if (sp_man.manual_override_switch > STICK_ON_OFF_LIMIT) {
					/* enable manual override */
					update_state_machine_mode_manual(stat_pub, &current_status, mavlink_fd);

				} else if (sp_man.manual_override_switch < -STICK_ON_OFF_LIMIT) {
					// /* check auto mode switch for correct mode */
					// if (sp_man.auto_mode_switch > STICK_ON_OFF_LIMIT) {
					// 	/* enable guided mode */
					// 	update_state_machine_mode_guided(stat_pub, &current_status, mavlink_fd);

					// } else if (sp_man.auto_mode_switch < -STICK_ON_OFF_LIMIT) {
						// XXX hardcode to auto for now
						update_state_machine_mode_auto(stat_pub, &current_status, mavlink_fd);

					// }

				} else {
					/* center stick position, set SAS for all vehicle types */
					update_state_machine_mode_stabilized(stat_pub, &current_status, mavlink_fd);
				}

				/* handle the case where RC signal was regained */
				if (!current_status.rc_signal_found_once) {
					current_status.rc_signal_found_once = true;
					mavlink_log_critical(mavlink_fd, "DETECTED RC SIGNAL FIRST TIME.");

				} else {
					if (current_status.rc_signal_lost) mavlink_log_critical(mavlink_fd, "[cmd] RECOVERY - RC SIGNAL GAINED!");
				}

				current_status.rc_signal_cutting_off = false;
				current_status.rc_signal_lost = false;
				current_status.rc_signal_lost_interval = 0;

			} else {
				static uint64_t last_print_time = 0;

				/* print error message for first RC glitch and then every 5 s / 5000 ms) */
				if (!current_status.rc_signal_cutting_off || ((hrt_absolute_time() - last_print_time) > 5000000)) {
					/* only complain if the offboard control is NOT active */
					current_status.rc_signal_cutting_off = true;
					mavlink_log_critical(mavlink_fd, "CRITICAL - NO REMOTE SIGNAL!");
					last_print_time = hrt_absolute_time();
				}

				/* flag as lost and update interval since when the signal was lost (to initiate RTL after some time) */
				current_status.rc_signal_lost_interval = hrt_absolute_time() - sp_man.timestamp;

				/* if the RC signal is gone for a full second, consider it lost */
				if (current_status.rc_signal_lost_interval > 1000000) {
					current_status.rc_signal_lost = true;
					current_status.failsave_lowlevel = true;
					state_changed = true;
				}

				// if (hrt_absolute_time() - current_status.failsave_ll_start_time > failsafe_lowlevel_timeout_ms*1000) {
				// 	publish_armed_status(&current_status);
				// }
			}
		}




		/* End mode switch */

		/* END RC state check */


		/* State machine update for offboard control */
		if (!current_status.rc_signal_found_once && sp_offboard.timestamp != 0) {
			if ((hrt_absolute_time() - sp_offboard.timestamp) < 5000000) {

				/* decide about attitude control flag, enable in att/pos/vel */
				bool attitude_ctrl_enabled = (sp_offboard.mode == OFFBOARD_CONTROL_MODE_DIRECT_ATTITUDE ||
							      sp_offboard.mode == OFFBOARD_CONTROL_MODE_DIRECT_VELOCITY ||
							      sp_offboard.mode == OFFBOARD_CONTROL_MODE_DIRECT_POSITION);

				/* decide about rate control flag, enable it always XXX (for now) */
				bool rates_ctrl_enabled = true;

				/* set up control mode */
				if (current_status.flag_control_attitude_enabled != attitude_ctrl_enabled) {
					current_status.flag_control_attitude_enabled = attitude_ctrl_enabled;
					state_changed = true;
				}

				if (current_status.flag_control_rates_enabled != rates_ctrl_enabled) {
					current_status.flag_control_rates_enabled = rates_ctrl_enabled;
					state_changed = true;
				}

				/* handle the case where offboard control signal was regained */
				if (!current_status.offboard_control_signal_found_once) {
					current_status.offboard_control_signal_found_once = true;
					/* enable offboard control, disable manual input */
					current_status.flag_control_manual_enabled = false;
					current_status.flag_control_offboard_enabled = true;
					state_changed = true;
					tune_confirm();

					mavlink_log_critical(mavlink_fd, "DETECTED OFFBOARD SIGNAL FIRST");

				} else {
					if (current_status.offboard_control_signal_lost) {
						mavlink_log_critical(mavlink_fd, "RECOVERY OFFBOARD CONTROL");
						state_changed = true;
						tune_confirm();
					}
				}

				current_status.offboard_control_signal_weak = false;
				current_status.offboard_control_signal_lost = false;
				current_status.offboard_control_signal_lost_interval = 0;

				/* arm / disarm on request */
				if (sp_offboard.armed && !current_status.flag_system_armed) {
					update_state_machine_arm(stat_pub, &current_status, mavlink_fd);
					/* switch to stabilized mode = takeoff */
					update_state_machine_mode_stabilized(stat_pub, &current_status, mavlink_fd);

				} else if (!sp_offboard.armed && current_status.flag_system_armed) {
					update_state_machine_disarm(stat_pub, &current_status, mavlink_fd);
				}

			} else {
				static uint64_t last_print_time = 0;

				/* print error message for first RC glitch and then every 5 s / 5000 ms) */
				if (!current_status.offboard_control_signal_weak || ((hrt_absolute_time() - last_print_time) > 5000000)) {
					current_status.offboard_control_signal_weak = true;
					mavlink_log_critical(mavlink_fd, "CRIT:NO OFFBOARD CONTROL!");
					last_print_time = hrt_absolute_time();
				}

				/* flag as lost and update interval since when the signal was lost (to initiate RTL after some time) */
				current_status.offboard_control_signal_lost_interval = hrt_absolute_time() - sp_offboard.timestamp;

				/* if the signal is gone for 0.1 seconds, consider it lost */
				if (current_status.offboard_control_signal_lost_interval > 100000) {
					current_status.offboard_control_signal_lost = true;
					current_status.failsave_lowlevel_start_time = hrt_absolute_time();
					tune_confirm();

					/* kill motors after timeout */
					if (hrt_absolute_time() - current_status.failsave_lowlevel_start_time > failsafe_lowlevel_timeout_ms * 1000) {
						current_status.failsave_lowlevel = true;
						state_changed = true;
					}
				}
			}
		}


		current_status.counter++;
		current_status.timestamp = hrt_absolute_time();


		/* If full run came back clean, transition to standby */
		if (current_status.state_machine == SYSTEM_STATE_PREFLIGHT &&
		    current_status.flag_preflight_gyro_calibration == false &&
		    current_status.flag_preflight_mag_calibration == false &&
		    current_status.flag_preflight_accel_calibration == false) {
			/* All ok, no calibration going on, go to standby */
			do_state_update(stat_pub, &current_status, mavlink_fd, SYSTEM_STATE_STANDBY);
		}

		/* publish at least with 1 Hz */
		if (counter % (1000000 / COMMANDER_MONITORING_INTERVAL) == 0 || state_changed) {
			publish_armed_status(&current_status);
			orb_publish(ORB_ID(vehicle_status), stat_pub, &current_status);
			state_changed = false;
		}

		/* Store old modes to detect and act on state transitions */
		voltage_previous = current_status.voltage_battery;

		fflush(stdout);
		counter++;
		usleep(COMMANDER_MONITORING_INTERVAL);
	}

	/* wait for threads to complete */
	// pthread_join(command_handling_thread, NULL);
	pthread_join(subsystem_info_thread, NULL);

	/* close fds */
	led_deinit();
	buzzer_deinit();
	close(sp_man_sub);
	close(sp_offboard_sub);
	close(global_position_sub);
	close(sensor_sub);
	close(cmd_sub);

	warnx("exiting..\n");
	fflush(stdout);

	thread_running = false;

	return 0;
}
