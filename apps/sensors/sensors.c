/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
 *           @author Thomas Gubler <thomasgubler@student.ethz.ch>
 *           @author Julian Oes <joes@student.ethz.ch>
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
 * @file sensors.c
 * Sensor readout process.
 */

#include <nuttx/config.h>

#include <fcntl.h>
#include <sys/prctl.h>
#include <poll.h>
#include <nuttx/analog/adc.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <float.h>

#include <arch/board/up_hrt.h>
#include <arch/board/drv_bma180.h>
#include <arch/board/drv_l3gd20.h>

#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>

#include <arch/board/up_adc.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>

#include "sensors.h"

#define errno *get_errno_ptr()

#define SENSOR_INTERVAL_MICROSEC 2000

#define GYRO_HEALTH_COUNTER_LIMIT_ERROR 20   /* 40 ms downtime at 500 Hz update rate   */
#define ACC_HEALTH_COUNTER_LIMIT_ERROR  20   /* 40 ms downtime at 500 Hz update rate   */
#define MAGN_HEALTH_COUNTER_LIMIT_ERROR 100  /* 1000 ms downtime at 100 Hz update rate  */
#define BARO_HEALTH_COUNTER_LIMIT_ERROR 50   /* 500 ms downtime at 100 Hz update rate  */
#define ADC_HEALTH_COUNTER_LIMIT_ERROR  10   /* 100 ms downtime at 100 Hz update rate  */

#define GYRO_HEALTH_COUNTER_LIMIT_OK 5
#define ACC_HEALTH_COUNTER_LIMIT_OK  5
#define MAGN_HEALTH_COUNTER_LIMIT_OK 5
#define BARO_HEALTH_COUNTER_LIMIT_OK 5
#define ADC_HEALTH_COUNTER_LIMIT_OK  5

#define ADC_BATTERY_VOLATGE_CHANNEL  10

#define BAT_VOL_INITIAL 12.f
#define BAT_VOL_LOWPASS_1 0.99f
#define BAT_VOL_LOWPASS_2 0.01f
#define VOLTAGE_BATTERY_IGNORE_THRESHOLD_VOLTS 3.5f

/* PPM Settings */
#define PPM_MIN 1000
#define PPM_MAX 2000
/* Internal resolution is 10000 */
#define PPM_SCALE 10000/((PPM_MAX-PPM_MIN)/2)

#define PPM_MID (PPM_MIN+PPM_MAX)/2

static int sensors_timer_loop_counter = 0;

/* File descriptors for all sensors */
static int fd_gyro = -1;
static int fd_gyro_l3gd20 = -1;

static bool thread_should_exit = false;
static bool thread_running = false;
static int sensors_task;

static int fd_bma180 = -1;
static int fd_magnetometer = -1;
static int fd_barometer = -1;
static int fd_adc = -1;
static int fd_accelerometer = -1;

/* Private functions declared static */
static void sensors_timer_loop(void *arg);

#ifdef CONFIG_HRT_PPM
extern uint16_t ppm_buffer[];
extern unsigned ppm_decoded_channels;
extern uint64_t ppm_last_valid_decode;
#endif

/* ORB topic publishing our results */
static orb_advert_t sensor_pub;

PARAM_DEFINE_FLOAT(SENSOR_GYRO_XOFF, 0.0f);
PARAM_DEFINE_FLOAT(SENSOR_GYRO_YOFF, 0.0f);
PARAM_DEFINE_FLOAT(SENSOR_GYRO_ZOFF, 0.0f);

PARAM_DEFINE_FLOAT(SENSOR_MAG_XOFF, 0.0f);
PARAM_DEFINE_FLOAT(SENSOR_MAG_YOFF, 0.0f);
PARAM_DEFINE_FLOAT(SENSOR_MAG_ZOFF, 0.0f);

PARAM_DEFINE_FLOAT(SENSOR_ACC_XOFF, 0.0f);
PARAM_DEFINE_FLOAT(SENSOR_ACC_YOFF, 0.0f);
PARAM_DEFINE_FLOAT(SENSOR_ACC_ZOFF, 0.0f);

PARAM_DEFINE_FLOAT(RC1_MIN, 1000.0f);
PARAM_DEFINE_FLOAT(RC1_TRIM, 1500.0f);
PARAM_DEFINE_FLOAT(RC1_MAX, 2000.0f);
PARAM_DEFINE_FLOAT(RC1_REV, 1.0f);

PARAM_DEFINE_FLOAT(RC2_MIN, 1000);
PARAM_DEFINE_FLOAT(RC2_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC2_MAX, 2000);
PARAM_DEFINE_FLOAT(RC2_REV, 1.0f);

PARAM_DEFINE_FLOAT(RC3_MIN, 1000);
PARAM_DEFINE_FLOAT(RC3_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC3_MAX, 2000);
PARAM_DEFINE_FLOAT(RC3_REV, 1.0f);

PARAM_DEFINE_FLOAT(RC4_MIN, 1000);
PARAM_DEFINE_FLOAT(RC4_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC4_MAX, 2000);
PARAM_DEFINE_FLOAT(RC4_REV, 1.0f);

PARAM_DEFINE_FLOAT(RC5_MIN, 1000);
PARAM_DEFINE_FLOAT(RC5_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC5_MAX, 2000);
PARAM_DEFINE_FLOAT(RC5_REV, 1.0f);

PARAM_DEFINE_FLOAT(RC6_MIN, 1000);
PARAM_DEFINE_FLOAT(RC6_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC6_MAX, 2000);
PARAM_DEFINE_FLOAT(RC6_REV, 1.0f);

PARAM_DEFINE_FLOAT(RC7_MIN, 1000);
PARAM_DEFINE_FLOAT(RC7_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC7_MAX, 2000);
PARAM_DEFINE_FLOAT(RC7_REV, 1.0f);

PARAM_DEFINE_FLOAT(RC8_MIN, 1000);
PARAM_DEFINE_FLOAT(RC8_TRIM, 1500);
PARAM_DEFINE_FLOAT(RC8_MAX, 2000);
PARAM_DEFINE_FLOAT(RC8_REV, 1.0f);

PARAM_DEFINE_INT32(RC_TYPE, 1); // 1 = FUTABA

PARAM_DEFINE_FLOAT(BAT_V_SCALING, -1.0f);

PARAM_DEFINE_INT32(RC_MAP_ROLL, 1);
PARAM_DEFINE_INT32(RC_MAP_PITCH, 2);
PARAM_DEFINE_INT32(RC_MAP_THROTTLE, 3);
PARAM_DEFINE_INT32(RC_MAP_YAW, 4);
PARAM_DEFINE_INT32(RC_MAP_MODE_SW, 5);

#define rc_max_chan_count 8

struct sensor_parameters {
	int min[rc_max_chan_count];
	int trim[rc_max_chan_count];
	int max[rc_max_chan_count];
	int rev[rc_max_chan_count];

	float gyro_offset[3];
	float mag_offset[3];
	float acc_offset[3];

	int rc_type;

	int rc_map_roll;
	int rc_map_pitch;
	int rc_map_yaw;
	int rc_map_throttle;
	int rc_map_mode_sw;

	int battery_voltage_scaling;
};

struct sensor_parameter_handles {
	param_t min[rc_max_chan_count];
	param_t trim[rc_max_chan_count];
	param_t max[rc_max_chan_count];
	param_t rev[rc_max_chan_count];
	param_t rc_type;

	param_t gyro_offset[3];
	param_t mag_offset[3];
	param_t acc_offset[3];

	param_t rc_map_roll;
	param_t rc_map_pitch;
	param_t rc_map_yaw;
	param_t rc_map_throttle;
	param_t rc_map_mode_sw;

	param_t battery_voltage_scaling;
};

/**
 * Sensor app start / stop handling function
 *
 * @ingroup apps
 */
__EXPORT int sensors_main(int argc, char *argv[]);

/**
 * Sensor readout and publishing.
 * 
 * This function reads all onboard sensors and publishes the sensor_combined topic.
 *
 * @see sensor_combined_s
 */
int sensors_thread_main(int argc, char *argv[]);

/**
 * Print the usage
 */
static void usage(const char *reason);

/**
 * Initialize all parameter handles and values
 *
 */
static int parameters_init(struct sensor_parameter_handles *h);

/**
 * Update all parameters
 *
 */
static int parameters_update(const struct sensor_parameter_handles *h, struct sensor_parameters *p);


static int parameters_init(struct sensor_parameter_handles *h)
{
	/* min values */
	h->min[0] = param_find("RC1_MIN");
	h->min[1] = param_find("RC2_MIN");
	h->min[2] = param_find("RC3_MIN");
	h->min[3] = param_find("RC4_MIN");
	h->min[4] = param_find("RC5_MIN");
	h->min[5] = param_find("RC6_MIN");
	h->min[6] = param_find("RC7_MIN");
	h->min[7] = param_find("RC8_MIN");

	/* trim values */
	h->trim[0] = param_find("RC1_TRIM");
	h->trim[1] = param_find("RC2_TRIM");
	h->trim[2] = param_find("RC3_TRIM");
	h->trim[3] = param_find("RC4_TRIM");
	h->trim[4] = param_find("RC5_TRIM");
	h->trim[5] = param_find("RC6_TRIM");
	h->trim[6] = param_find("RC7_TRIM");
	h->trim[7] = param_find("RC8_TRIM");

	/* max values */
	h->max[0] = param_find("RC1_MAX");
	h->max[1] = param_find("RC2_MAX");
	h->max[2] = param_find("RC3_MAX");
	h->max[3] = param_find("RC4_MAX");
	h->max[4] = param_find("RC5_MAX");
	h->max[5] = param_find("RC6_MAX");
	h->max[6] = param_find("RC7_MAX");
	h->max[7] = param_find("RC8_MAX");

	/* channel reverse */
	h->rev[0] = param_find("RC1_REV");
	h->rev[1] = param_find("RC2_REV");
	h->rev[2] = param_find("RC3_REV");
	h->rev[3] = param_find("RC4_REV");
	h->rev[4] = param_find("RC5_REV");
	h->rev[5] = param_find("RC6_REV");
	h->rev[6] = param_find("RC7_REV");
	h->rev[7] = param_find("RC8_REV");

	h->rc_type = param_find("RC_TYPE");

	h->rc_map_roll 		= param_find("RC_MAP_ROLL");
	h->rc_map_pitch 	= param_find("RC_MAP_PITCH");
	h->rc_map_yaw 		= param_find("RC_MAP_YAW");
	h->rc_map_throttle 	= param_find("RC_MAP_THROTTLE");
	h->rc_map_mode_sw 	= param_find("RC_MAP_MODE_SW");

	/* gyro offsets */
	h->gyro_offset[0] = param_find("SENSOR_GYRO_XOFF");
	h->gyro_offset[1] = param_find("SENSOR_GYRO_YOFF");
	h->gyro_offset[2] = param_find("SENSOR_GYRO_ZOFF");

	/* accel offsets */
	h->acc_offset[0] = param_find("SENSOR_ACC_XOFF");
	h->acc_offset[1] = param_find("SENSOR_ACC_YOFF");
	h->acc_offset[2] = param_find("SENSOR_ACC_ZOFF");

	/* mag offsets */
	h->mag_offset[0] = param_find("SENSOR_MAG_XOFF");
	h->mag_offset[1] = param_find("SENSOR_MAG_YOFF");
	h->mag_offset[2] = param_find("SENSOR_MAG_ZOFF");

	h->battery_voltage_scaling = param_find("BAT_V_SCALING");

	return OK;
}

static int parameters_update(const struct sensor_parameter_handles *h, struct sensor_parameters *p)
{
	const unsigned int nchans = 8;

	/* min values */
	for (unsigned int i = 0; i < nchans; i++) {
		param_get(h->min[i], &(p->min[i]));
	}

	/* trim values */
	for (unsigned int i = 0; i < nchans; i++) {
		param_get(h->trim[i], &(p->trim[i]));
	}

	/* max values */
	for (unsigned int i = 0; i < nchans; i++) {
		param_get(h->max[i], &(p->max[i]));
	}

	/* channel reverse */
	for (unsigned int i = 0; i < nchans; i++) {
		param_get(h->rev[i], &(p->rev[i]));
	}

	/* remote control type */
	param_get(h->rc_type, &(p->rc_type));

	/* channel mapping */
	param_get(h->rc_map_roll, &(p->rc_map_roll));
	param_get(h->rc_map_pitch, &(p->rc_map_pitch));
	param_get(h->rc_map_yaw, &(p->rc_map_yaw));
	param_get(h->rc_map_throttle, &(p->rc_map_throttle));
	param_get(h->rc_map_mode_sw, &(p->rc_map_mode_sw));

	/* gyro offsets */
	param_get(h->gyro_offset[0], &(p->gyro_offset[0]));
	param_get(h->gyro_offset[1], &(p->gyro_offset[1]));
	param_get(h->gyro_offset[2], &(p->gyro_offset[2]));

	/* accel offsets */
	param_get(h->acc_offset[0], &(p->acc_offset[0]));
	param_get(h->acc_offset[1], &(p->acc_offset[1]));
	param_get(h->acc_offset[2], &(p->acc_offset[2]));

	/* mag offsets */
	param_get(h->mag_offset[0], &(p->mag_offset[0]));
	param_get(h->mag_offset[1], &(p->mag_offset[1]));
	param_get(h->mag_offset[2], &(p->mag_offset[2]));

	/* scaling of ADC ticks to battery voltage */
	param_get(h->battery_voltage_scaling, &(p->battery_voltage_scaling));

	return OK;
}

/**
 * Initialize all sensor drivers.
 *
 * @return 0 on success, != 0 on failure
 */
static int sensors_init(void)
{
	printf("[sensors] Sensor configuration..\n");

	/* open magnetometer */
	fd_magnetometer = open("/dev/mag", O_RDONLY);

	if (fd_magnetometer < 0) {
		fprintf(stderr, "[sensors]   MAG open fail (err #%d): %s\n", (int)*get_errno_ptr(), strerror((int)*get_errno_ptr()));
		fflush(stderr);
		/* this sensor is critical, exit on failed init */
		errno = ENOSYS;
		return ERROR;

	} else {
		printf("[sensors]   MAG open ok\n");
		/* set the queue depth to 1 */
		if (OK != ioctl(fd_magnetometer, MAGIOCSQUEUEDEPTH, 1))
			warn("failed to set queue depth for mag");

		/* start the sensor polling at 150Hz */
		if (OK != ioctl(fd_magnetometer, MAGIOCSPOLLRATE, 150))
			warn("failed to set 150Hz poll rate for mag");
	}

	/* open barometer */
	fd_barometer = open("/dev/baro", O_RDONLY);

	if (fd_barometer < 0) {
		fprintf(stderr, "[sensors]   BARO open fail (err #%d): %s\n", (int)*get_errno_ptr(), strerror((int)*get_errno_ptr()));
		fflush(stderr);

	} else {
		printf("[sensors]   BARO open ok\n");
		/* set the queue depth to 1 */
		if (OK != ioctl(fd_barometer, BAROIOCSQUEUEDEPTH, 1))
			warn("failed to set queue depth for baro");

		/* start the sensor polling at 100Hz */
		if (OK != ioctl(fd_barometer, BAROIOCSPOLLRATE, 100))
			warn("failed to set 100Hz poll rate for baro");
	}

	/* open gyro */
	fd_gyro = open("/dev/gyro", O_RDONLY);
	int errno_gyro = (int)*get_errno_ptr();

	if (!(fd_gyro < 0)) {
		printf("[sensors]   GYRO open ok\n");
		// /* set the queue depth to 1 */
		// if (OK != ioctl(fd_gyro, GYROIOCSQUEUEDEPTH, 1))
		// 	warn("failed to set queue depth for gyro");

		// /* start the sensor polling at 500Hz */
		// if (OK != ioctl(fd_gyro, GYROIOCSPOLLRATE, 500))
		// 	warn("failed to set 500Hz poll rate for gyro");
	}

	printf("just before accel\n");

	/* open accelerometer */
	fd_accelerometer = open("/dev/accel", O_RDONLY);
	int errno_accelerometer = (int)*get_errno_ptr();

	if (!(fd_accelerometer < 0)) {
		printf("[sensors]   ACCEL open ok\n");
		// /* set the queue depth to 1 */
		// if (OK != ioctl(fd_accelerometer, ACCELIOCSQUEUEDEPTH, 1))
		// 	warn("failed to set queue depth for accel");

		// /* start the sensor polling at 500Hz */
		// if (OK != ioctl(fd_accelerometer, ACCELIOCSPOLLRATE, 500))
		// 	warn("failed to set 500Hz poll rate for accel");
	}

	/* only attempt to use BMA180 if main accel is not available */
	int errno_bma180 = 0;
	if (fd_accelerometer < 0) {
		fd_bma180 = open("/dev/bma180", O_RDONLY);
		errno_bma180 = (int)*get_errno_ptr();

		if (!(fd_bma180 < 0)) {
			printf("[sensors]   ACCEL (BMA180) open ok\n");
		}
	} else {
		fd_bma180 = -1;
	}

	/* only attempt to use L3GD20 is main gyro is not available */
	int errno_gyro_l3gd20 = 0;
	if (fd_gyro < 0) {
		fd_gyro_l3gd20  = open("/dev/l3gd20", O_RDONLY);
		int errno_gyro_l3gd20 = (int)*get_errno_ptr();

		if (!(fd_gyro_l3gd20 < 0)) {
			printf("[sensors]   GYRO (L3GD20) open ok\n");
		}

		if (ioctl(fd_gyro_l3gd20 , L3GD20_SETRATE, L3GD20_RATE_760HZ_LP_30HZ) ||
			ioctl(fd_gyro_l3gd20 , L3GD20_SETRANGE, L3GD20_RANGE_500DPS)) {
			fprintf(stderr, "[sensors]   L3GD20 configuration (ioctl) fail (err #%d): %s\n", (int)*get_errno_ptr(), strerror((int)*get_errno_ptr()));
			fflush(stderr);
			/* this sensor is critical, exit on failed init */
			errno = ENOSYS;
			return ERROR;

		} else {
			printf("[sensors]   L3GD20 configuration ok\n");
		}
	} else {
		fd_gyro_l3gd20 = -1;
	}

	/* fail if no accelerometer is available */
	if (fd_accelerometer < 0 && fd_bma180 < 0) {
		/* print error message only if both failed, discard message else at all to not confuse users */
		if (fd_accelerometer < 0) {
			fprintf(stderr, "[sensors]   ACCEL: open fail (err #%d): %s\n", errno_accelerometer, strerror(errno_accelerometer));
			fflush(stderr);
			/* this sensor is redundant with BMA180 */
		}
		
		if (fd_bma180 < 0) {
			fprintf(stderr, "[sensors]   BMA180: open fail (err #%d): %s\n", errno_bma180, strerror(errno_bma180));
			fflush(stderr);
			/* this sensor is redundant with MPU-6000 */
		}

		errno = ENOSYS;
		return ERROR;
	}

	/* fail if no gyro is available */
	if (fd_gyro < 0 && fd_gyro_l3gd20 < 0) {
		/* print error message only if both failed, discard message else at all to not confuse users */
		if (fd_gyro < 0) {
			fprintf(stderr, "[sensors]   GYRO: open fail (err #%d): %s\n", errno_gyro, strerror(errno_gyro));
			fflush(stderr);
			/* this sensor is redundant with BMA180 */
		}
		
		if (fd_gyro_l3gd20 < 0) {
			fprintf(stderr, "[sensors]   L3GD20 open fail (err #%d): %s\n", errno_gyro_l3gd20, strerror(errno_gyro_l3gd20));
			fflush(stderr);
			/* this sensor is critical, exit on failed init */
		}

		errno = ENOSYS;
		return ERROR;
	}

	/* open adc */
	fd_adc = open("/dev/adc0", O_RDONLY | O_NONBLOCK);

	if (fd_adc < 0) {
		fprintf(stderr, "[sensors]   ADC: open fail (err #%d): %s\n", (int)*get_errno_ptr(), strerror((int)*get_errno_ptr()));
		fflush(stderr);
		/* this sensor is critical, exit on failed init */
		errno = ENOSYS;
		return ERROR;

	} else {
		printf("[sensors]   ADC open ok\n");
	}

	printf("[sensors] All sensors configured\n");
	return OK;
}

int sensors_thread_main(int argc, char *argv[])
{
	/* inform about start */
	printf("[sensors] Initializing..\n");
	fflush(stdout);

	int ret = OK;

	/* start sensor reading */
	if (sensors_init() != OK) {
		fprintf(stderr, "[sensors] ERROR: Failed to initialize all sensors, exiting.\n");
		/* Clean up */
		close(fd_gyro);
		close(fd_bma180);
		close(fd_gyro_l3gd20);
		close(fd_magnetometer);
		close(fd_barometer);
		close(fd_adc);

		exit(1);
	} else {
		/* flush stdout from init routine */
		fflush(stdout);
	}

	/* initialize parameters */
	struct sensor_parameters rcp;
	struct sensor_parameter_handles rch;
	parameters_init(&rch);
	parameters_update(&rch, &rcp);

	// bool gyro_healthy = false;
	// bool acc_healthy = false;
	// bool magn_healthy = false;
	// bool baro_healthy = false;
	// bool adc_healthy = false;

	bool hil_enabled = false;		/**< HIL is disabled by default	*/
	bool publishing = false;		/**< the app is not publishing by default, only if HIL is disabled on first run */

	// unsigned int mag_fail_count = 0;
	// unsigned int mag_success_count = 0;

	// unsigned int baro_fail_count = 0;
	// unsigned int baro_success_count = 0;

	// unsigned int gyro_fail_count = 0;
	// unsigned int gyro_success_count = 0;

	// unsigned int acc_fail_count = 0;
	// unsigned int acc_success_count = 0;

	// unsigned int adc_fail_count = 0;
	// unsigned int adc_success_count = 0;

	/* for PX4FMU 1.5 compatibility */
	int16_t buf_accelerometer[3];
	int16_t buf_gyro[3];

	// bool mag_calibration_enabled = false;

	#pragma pack(push,1)
	struct adc_msg4_s {
		uint8_t      am_channel1;	/**< The 8-bit ADC Channel 1 */
		int32_t      am_data1;		/**< ADC convert result 1 (4 bytes) */
		uint8_t      am_channel2;	/**< The 8-bit ADC Channel 2 */
		int32_t      am_data2;		/**< ADC convert result 2 (4 bytes) */
		uint8_t      am_channel3;	/**< The 8-bit ADC Channel 3 */
		int32_t      am_data3;		/**< ADC convert result 3 (4 bytes) */
		uint8_t      am_channel4;	/**< The 8-bit ADC Channel 4 */
		int32_t      am_data4;		/**< ADC convert result 4 (4 bytes) */
	};
	#pragma pack(pop)

	struct adc_msg4_s buf_adc;
	size_t adc_readsize = 1 * sizeof(struct adc_msg4_s);

	float battery_voltage_conversion;
	battery_voltage_conversion = rcp.battery_voltage_scaling;

	if (-1 == (int)battery_voltage_conversion) {
		/* default is conversion factor for the PX4IO / PX4IOAR board, the factor for PX4FMU standalone is different */
		battery_voltage_conversion = 3.3f * 52.0f / 5.0f / 4095.0f;
	}

	/* initialize to 100 to execute immediately */
	int paramcounter = 100;
	int read_loop_counter = 0;

	/* Empty sensor buffers, avoid junk values */
	/* Read first two values of each sensor into void */
	if (fd_bma180 > 0)(void)read(fd_bma180, buf_accelerometer, sizeof(buf_accelerometer));
	if (fd_gyro_l3gd20 > 0)(void)read(fd_gyro_l3gd20, &buf_gyro, sizeof(buf_gyro));

	/* ORB sensor subscriptions */
	int gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));
	int accel_sub = orb_subscribe(ORB_ID(sensor_accel));
	int mag_sub = orb_subscribe(ORB_ID(sensor_mag));
	int baro_sub = orb_subscribe(ORB_ID(sensor_baro));

	struct gyro_report gyro_report;
	struct accel_report accel_report;
	struct mag_report mag_report;
	struct baro_report baro_report;

	struct sensor_combined_s raw = {
		.timestamp = hrt_absolute_time(),
		.gyro_raw = {gyro_report.x_raw, gyro_report.y_raw, gyro_report.z_raw},
		.gyro_raw_counter = 0,
		.gyro_rad_s = {gyro_report.x, gyro_report.y, gyro_report.z},
		.accelerometer_raw = {accel_report.x_raw, accel_report.y_raw, accel_report.z_raw},
		.accelerometer_raw_counter = 0,
		.accelerometer_m_s2 = {accel_report.x, accel_report.y, accel_report.z},
		.magnetometer_raw = {mag_report.x_raw, mag_report.y_raw, mag_report.z_raw},
		.magnetometer_ga = {mag_report.x, mag_report.y, mag_report.z},
		.magnetometer_raw_counter = 0,
		.baro_pres_mbar = baro_report.pressure,
		.baro_alt_meter = baro_report.altitude,
		.baro_temp_celcius = baro_report.temperature,
		.baro_raw_counter = 0,
		.battery_voltage_v = BAT_VOL_INITIAL,
		.adc_voltage_v = {0.9f , 0.0f , 0.0f},
		.battery_voltage_counter = 0,
		.battery_voltage_valid = false,
	};

	/* advertise the sensor_combined topic and make the initial publication */
	sensor_pub = orb_advertise(ORB_ID(sensor_combined), &raw);
	if (sensor_pub < 0) {
		fprintf(stderr, "[sensors] ERROR: orb_advertise for topic sensor_combined failed.\n");
	} else {
		publishing = true;
	}

	/* advertise the manual_control topic */
	struct manual_control_setpoint_s manual_control = { .mode = ROLLPOS_PITCHPOS_YAWRATE_THROTTLE,
						   .roll = 0.0f,
						   .pitch = 0.0f,
						   .yaw = 0.0f,
						   .throttle = 0.0f };

	orb_advert_t manual_control_pub = orb_advertise(ORB_ID(manual_control_setpoint), &manual_control);

	if (manual_control_pub < 0) {
		fprintf(stderr, "[sensors] ERROR: orb_advertise for topic manual_control_setpoint failed.\n");
	}

	/* advertise the rc topic */
	struct rc_channels_s rc;
	memset(&rc, 0, sizeof(rc));
	orb_advert_t rc_pub = orb_advertise(ORB_ID(rc_channels), &rc);

	if (rc_pub < 0) {
		fprintf(stderr, "[sensors] ERROR: orb_advertise for topic rc_channels failed.\n");
	}

	/* subscribe to system status */
	struct vehicle_status_s vstatus;
	memset(&vstatus, 0, sizeof(vstatus));
	int vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));

	thread_running = true;

	while (!thread_should_exit) {
		
		bool gyro_updated = false;

		struct pollfd fds[4];

		/* wait for data to be ready */
		fds[0].fd = gyro_sub;
		fds[0].events = POLLIN;

		fds[1].fd = accel_sub;
		fds[1].events = POLLIN;

		fds[2].fd = mag_sub;
		fds[2].events = POLLIN;

		fds[3].fd = baro_sub;
		fds[3].events = POLLIN;

		int pret = poll(fds, 4, 500);

		if (pret <= 0) {
			/* do silently nothing */
		} else {

			/* store the time closest to all measurements */
			uint64_t current_time = hrt_absolute_time();
			raw.timestamp = current_time;

			/* Update at 5 Hz */
			if (paramcounter == ((unsigned int)(1000000 / SENSOR_INTERVAL_MICROSEC)/5)) {

				/* Check HIL state */
				orb_copy(ORB_ID(vehicle_status), vstatus_sub, &vstatus);

				/* switching from non-HIL to HIL mode */
				//printf("[sensors] Vehicle mode: %i \t AND: %i, HIL: %i\n", vstatus.mode, vstatus.mode & VEHICLE_MODE_FLAG_HIL_ENABLED, hil_enabled);
				if (vstatus.flag_hil_enabled && !hil_enabled) {
					hil_enabled = true;
					publishing = false;

					int sens_ret = close(sensor_pub);
					if (sens_ret == OK) {
						printf("[sensors] Closing sensor pub OK\n");
					} else {
						printf("[sensors] FAILED Closing sensor pub, result: %i \n", sens_ret);
					}

					/* switching from HIL to non-HIL mode */

				} else if (!publishing && !hil_enabled) {
					/* advertise the topic and make the initial publication */
					sensor_pub = orb_advertise(ORB_ID(sensor_combined), &raw);
					hil_enabled = false;
					publishing = true;
				}

				/* update parameters */
				parameters_update(&rch, &rcp);

				/* Update RC scalings and function mappings */
				rc.chan[0].scaling_factor = (1.0f / ((rcp.max[0] - rcp.min[0]) / 2.0f) * rcp.rev[0]);
				rc.chan[0].mid = rcp.trim[0];

				rc.chan[1].scaling_factor = (1.0f / ((rcp.max[1] - rcp.min[1]) / 2.0f) * rcp.rev[1]);
				rc.chan[1].mid = rcp.trim[1];

				rc.chan[2].scaling_factor = (1.0f / ((rcp.max[2] - rcp.min[2]) / 2.0f) * rcp.rev[2]);
				rc.chan[2].mid = rcp.trim[2];

				rc.chan[3].scaling_factor = (1.0f / ((rcp.max[3] - rcp.min[3]) / 2.0f) * rcp.rev[3]);
				rc.chan[3].mid = rcp.trim[3];

				rc.chan[4].scaling_factor = (1.0f / ((rcp.max[4] - rcp.min[4]) / 2.0f) * rcp.rev[4]);
				rc.chan[4].mid = rcp.trim[4];

				rc.chan[5].scaling_factor = (1.0f / ((rcp.max[5] - rcp.min[5]) / 2.0f) * rcp.rev[5]);
				rc.chan[5].mid = rcp.trim[5];

				rc.chan[6].scaling_factor = (1.0f / ((rcp.max[6] - rcp.min[6]) / 2.0f) * rcp.rev[6]);
				rc.chan[6].mid = rcp.trim[6];

				rc.chan[7].scaling_factor = (1.0f / ((rcp.max[7] - rcp.min[7]) / 2.0f) * rcp.rev[7]);
				rc.chan[7].mid = rcp.trim[7];

				rc.function[0] = rcp.rc_map_throttle - 1;
				rc.function[1] = rcp.rc_map_roll - 1;
				rc.function[2] = rcp.rc_map_pitch - 1;
				rc.function[3] = rcp.rc_map_yaw - 1;
				rc.function[4] = rcp.rc_map_mode_sw - 1;

				paramcounter = 0;
			}
			paramcounter++;

			/* --- GYRO --- */
			if (fds[0].revents & POLLIN) {

				orb_copy(ORB_ID(sensor_gyro), gyro_sub, &gyro_report);

				raw.gyro_rad_s[0] = gyro_report.x;
				raw.gyro_rad_s[1] = gyro_report.y;
				raw.gyro_rad_s[2] = gyro_report.z;

				raw.gyro_raw[0] = gyro_report.x_raw;
				raw.gyro_raw[1] = gyro_report.y_raw;
				raw.gyro_raw[2] = gyro_report.z_raw;

				raw.gyro_raw_counter++;
				/* gyro is clocking synchronous data output */
				gyro_updated = true;
			}

			/* --- ACCEL --- */
			if (fds[1].revents & POLLIN) {

				orb_copy(ORB_ID(sensor_mag), mag_sub, &mag_report);

				raw.accelerometer_m_s2[0] = gyro_report.x;
				raw.gyro_rad_s[1] = gyro_report.y;
				raw.gyro_rad_s[2] = gyro_report.z;

				raw.gyro_raw[0] = gyro_report.x_raw;
				raw.gyro_raw[1] = gyro_report.y_raw;
				raw.gyro_raw[2] = gyro_report.z_raw;

				raw.accelerometer_raw_counter++;
			}

			/* --- MAG --- */
			if (fds[2].revents & POLLIN) {

				orb_copy(ORB_ID(sensor_mag), mag_sub, &mag_report);

				raw.magnetometer_ga[0] = mag_report.x;
				raw.magnetometer_ga[1] = mag_report.y;
				raw.magnetometer_ga[2] = mag_report.z;

				raw.magnetometer_raw[0] = mag_report.x_raw;
				raw.magnetometer_raw[1] = mag_report.y_raw;
				raw.magnetometer_raw[2] = mag_report.z_raw;
				
				raw.magnetometer_raw_counter++;
			}

			/* --- BARO --- */
			if (fds[3].revents & POLLIN) {

				orb_copy(ORB_ID(sensor_baro), baro_sub, &baro_report);

				raw.baro_pres_mbar = baro_report.pressure; // Pressure in mbar
				raw.baro_alt_meter = baro_report.altitude; // Altitude in meters
				raw.baro_temp_celcius = baro_report.temperature; // Temperature in degrees celcius

				raw.baro_raw_counter++;
			}

			// /* read BMA180. If the MPU-6000 is present, the BMA180 file descriptor won't be open */
			// if (fd_bma180 > 0) {
			// 	/* try reading acc */
			// 	uint64_t start_acc = hrt_absolute_time();
			// 	ret_accelerometer = read(fd_bma180, buf_accelerometer, 6);

			// 	/* ACCELEROMETER */
			// 	if (ret_accelerometer != 6) {
			// 		acc_fail_count++;

			// 		if ((acc_fail_count % 500) == 0 || (acc_fail_count > 20 && acc_fail_count < 40)) {
			// 			fprintf(stderr, "[sensors] BMA180 ERROR #%d: %s\n", (int)*get_errno_ptr(), strerror((int)*get_errno_ptr()));
			// 		}

			// 		if (acc_healthy && acc_fail_count >= ACC_HEALTH_COUNTER_LIMIT_ERROR) {
			// 			// global_data_send_subsystem_info(&acc_present_enabled);
			// 			gyro_healthy = false;
			// 			acc_success_count = 0;
			// 		}

			// 	} else {
			// 		acc_success_count++;

			// 		if (!acc_healthy && acc_success_count >= ACC_HEALTH_COUNTER_LIMIT_OK) {

			// 			// global_data_send_subsystem_info(&acc_present_enabled_healthy);
			// 			acc_healthy = true;
			// 			acc_fail_count = 0;

			// 		}

			// 		acc_updated = true;
			// 	}

			// 	int acctime = hrt_absolute_time() - start_acc;
			// 	if (acctime > 500) printf("ACC: %d us\n", acctime);
			// }


			// /* ACCELEROMETER */
			// if (acc_updated) {
			// 	/* copy sensor readings to global data and transform coordinates into px4fmu board frame */

			// 	if (fd_bma180 > 0) {

			// 		/* assign negated value, except for -SHORT_MAX, as it would wrap there */
			// 		raw.accelerometer_raw[0] = (buf_accelerometer[1] == -32768) ? 32767 : -buf_accelerometer[1]; // x of the board is -y of the sensor
			// 		raw.accelerometer_raw[1] = (buf_accelerometer[0] == -32768) ? -32767 : buf_accelerometer[0]; // y on the board is x of the sensor
			// 		raw.accelerometer_raw[2] = (buf_accelerometer[2] == -32768) ? -32767 : buf_accelerometer[2]; // z of the board is z of the sensor


			// 		// XXX read range from sensor
			// 		float range_g = 4.0f;
			// 		/* scale from 14 bit to m/s2 */
			// 		raw.accelerometer_m_s2[0] = (((raw.accelerometer_raw[0] - rcp.acc_offset[0]) * range_g) / 8192.0f) / 9.81f;
			// 		raw.accelerometer_m_s2[1] = (((raw.accelerometer_raw[1] - rcp.acc_offset[1]) * range_g) / 8192.0f) / 9.81f;
			// 		raw.accelerometer_m_s2[2] = (((raw.accelerometer_raw[2] - rcp.acc_offset[2]) * range_g) / 8192.0f) / 9.81f;

			// 		raw.accelerometer_raw_counter++;
			// 	}
			// }

			// if (fd_gyro_l3gd20 > 0) {
			// 	/* try reading gyro */
			// 	uint64_t start_gyro = hrt_absolute_time();
			// 	ret_gyro = read(fd_gyro, buf_gyro_l3gd20, sizeof(buf_gyro_l3gd20));
			// 	int gyrotime = hrt_absolute_time() - start_gyro;

			// 	if (gyrotime > 500) printf("L3GD20 GYRO (pure read): %d us\n", gyrotime);

			// 	/* GYROSCOPE */
			// 	if (ret_gyro != sizeof(buf_gyro)) {
			// 		gyro_fail_count++;

			// 		if ((((gyro_fail_count % 20) == 0) || (gyro_fail_count > 20 && gyro_fail_count < 100)) && (int)*get_errno_ptr() != EAGAIN) {
			// 			fprintf(stderr, "[sensors] L3GD20 ERROR #%d: %s\n", (int)*get_errno_ptr(), strerror((int)*get_errno_ptr()));
			// 		}

			// 		if (gyro_healthy && gyro_fail_count >= GYRO_HEALTH_COUNTER_LIMIT_ERROR) {
			// 			// global_data_send_subsystem_info(&gyro_present_enabled);
			// 			gyro_healthy = false;
			// 			gyro_success_count = 0;
			// 		}

			// 	} else {
			// 		gyro_success_count++;

			// 		if (!gyro_healthy && gyro_success_count >= GYRO_HEALTH_COUNTER_LIMIT_OK) {
			// 			// global_data_send_subsystem_info(&gyro_present_enabled_healthy);
			// 			gyro_healthy = true;
			// 			gyro_fail_count = 0;

			// 		}

			// 		gyro_updated = true;
			// 	}

			// 	gyrotime = hrt_absolute_time() - start_gyro;

			// 	if (gyrotime > 500) printf("L3GD20 GYRO (complete): %d us\n", gyrotime);
			// }

			/* GYROSCOPE */
			// if (gyro_updated) {
			// 	/* copy sensor readings to global data and transform coordinates into px4fmu board frame */

			// 	raw.gyro_raw[0] = ((buf_gyro[1] == -32768) ? -32768 : buf_gyro[1]); // x of the board is y of the sensor
			// 	/* assign negated value, except for -SHORT_MAX, as it would wrap there */
			// 	raw.gyro_raw[1] = ((buf_gyro[0] == -32768) ? 32767 : -buf_gyro[0]); // y on the board is -x of the sensor
			// 	raw.gyro_raw[2] = ((buf_gyro[2] == -32768) ? -32768 : buf_gyro[2]); // z of the board is z of the sensor

			// 	/* scale measurements */
			// 	// XXX request scaling from driver instead of hardcoding it
			// 	/* scaling calculated as: raw * (1/(32768*(500/180*PI))) */
			// 	raw.gyro_rad_s[0] = (raw.gyro_raw[0] - rcp.gyro_offset[0]) * 0.000266316109f;
			// 	raw.gyro_rad_s[1] = (raw.gyro_raw[1] - rcp.gyro_offset[1]) * 0.000266316109f;
			// 	raw.gyro_rad_s[2] = (raw.gyro_raw[2] - rcp.gyro_offset[2]) * 0.000266316109f;

			// 	raw.gyro_raw_counter++;
			// }

			static uint64_t last_adc = 0;
			/* ADC */
			if (hrt_absolute_time() - last_adc >= 10000) {
				int ret_adc = read(fd_adc, &buf_adc, adc_readsize);
				int nsamples_adc = ret_adc / sizeof(struct adc_msg_s);

				// if (ret_adc  < 0 || ((int)(nsamples_adc * sizeof(struct adc_msg_s))) != ret_adc) {
				// 	adc_fail_count++;

				// 	if (((adc_fail_count % 20) == 0 || adc_fail_count < 10) && (int)*get_errno_ptr() != EAGAIN) {
				// 		fprintf(stderr, "[sensors] ADC ERROR #%d: %s\n", (int)*get_errno_ptr(), strerror((int)*get_errno_ptr()));
				// 	}

				// 	if (adc_healthy && adc_fail_count >= ADC_HEALTH_COUNTER_LIMIT_ERROR) {
				// 		adc_healthy = false;
				// 		adc_success_count = 0;
				// 	}

				// } else {
				// 	adc_success_count++;

				// 	if (!adc_healthy && adc_success_count >= ADC_HEALTH_COUNTER_LIMIT_OK) {
				// 		adc_healthy = true;
				// 		adc_fail_count = 0;
				// 	}

				// 	adc_updated = true;
				// }

				if (ADC_BATTERY_VOLATGE_CHANNEL == buf_adc.am_channel1) {
					/* Voltage in volts */
					raw.battery_voltage_v = (BAT_VOL_LOWPASS_1 * (raw.battery_voltage_v + BAT_VOL_LOWPASS_2 * (buf_adc.am_data1 * battery_voltage_conversion)));

					if ((buf_adc.am_data1 * battery_voltage_conversion) < VOLTAGE_BATTERY_IGNORE_THRESHOLD_VOLTS) {
						raw.battery_voltage_valid = false;
						raw.battery_voltage_v = 0.f;

					} else {
						raw.battery_voltage_valid = true;
					}

					raw.battery_voltage_counter++;
				}

				last_adc = hrt_absolute_time();
			}

			/* Inform other processes that new data is available to copy */
			if (gyro_updated && publishing) {
				/* Values changed, publish */
				orb_publish(ORB_ID(sensor_combined), sensor_pub, &raw);
			}

#ifdef CONFIG_HRT_PPM
			static uint64_t last_ppm = 0;

			/* PPM */
			if (hrt_absolute_time() - last_ppm >= 10000) {

				/* require at least two channels
				 * to consider the signal valid
				 * check that decoded measurement is up to date
				 */
				if (ppm_decoded_channels > 1 && (hrt_absolute_time() - ppm_last_valid_decode) < 45000) {
					/* Read out values from HRT */
					for (unsigned int i = 0; i < ppm_decoded_channels; i++) {
						rc.chan[i].raw = ppm_buffer[i];
						/* Set the range to +-, then scale up */
						rc.chan[i].scale = (ppm_buffer[i] - rc.chan[i].mid) * rc.chan[i].scaling_factor * 10000;
						rc.chan[i].scaled = (ppm_buffer[i] - rc.chan[i].mid) * rc.chan[i].scaling_factor;
					}

					rc.chan_count = ppm_decoded_channels;
					rc.timestamp = ppm_last_valid_decode;

					/* roll input */
					manual_control.roll = rc.chan[rc.function[ROLL]].scaled;
					if (manual_control.roll < -1.0f) manual_control.roll = -1.0f;
					if (manual_control.roll >  1.0f) manual_control.roll =  1.0f;

					/* pitch input */
					manual_control.pitch = rc.chan[rc.function[PITCH]].scaled;
					if (manual_control.pitch < -1.0f) manual_control.pitch = -1.0f;
					if (manual_control.pitch >  1.0f) manual_control.pitch =  1.0f;

					/* yaw input */
					manual_control.yaw = rc.chan[rc.function[YAW]].scaled;
					if (manual_control.yaw < -1.0f) manual_control.yaw = -1.0f;
					if (manual_control.yaw >  1.0f) manual_control.yaw =  1.0f;
					
					/* throttle input */
					manual_control.throttle = (rc.chan[rc.function[THROTTLE]].scaled+1.0f)/2.0f;
					if (manual_control.throttle < 0.0f) manual_control.throttle = 0.0f;
					if (manual_control.throttle > 1.0f) manual_control.throttle = 1.0f;

					/* mode switch input */
					manual_control.override_mode_switch = rc.chan[rc.function[OVERRIDE]].scaled;
					if (manual_control.override_mode_switch < -1.0f) manual_control.override_mode_switch = -1.0f;
					if (manual_control.override_mode_switch >  1.0f) manual_control.override_mode_switch =  1.0f;

					orb_publish(ORB_ID(rc_channels), rc_pub, &rc);
					orb_publish(ORB_ID(manual_control_setpoint), manual_control_pub, &manual_control);

				}
				last_ppm = hrt_absolute_time();
			}
#endif

			read_loop_counter++;
		}
	}

	printf("[sensors] sensor readout stopped\n");

	close(fd_gyro);
	close(fd_magnetometer);
	close(fd_barometer);
	close(fd_adc);

	/* maintained for backwards-compatibility with v1.5 */
	close(fd_gyro_l3gd20);
	close(fd_bma180);

	close(gyro_sub);
	close(accel_sub);
	close(mag_sub);
	close(baro_sub);

	printf("[sensors] exiting.\n");

	thread_running = false;

	return ret;
}

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: sensors {start|stop|status}\n");
	exit(1);
}

int sensors_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("sensors app already running\n");
		} else {
			thread_should_exit = false;
			sensors_task = task_create("sensors", SCHED_PRIORITY_MAX - 5, 4096, sensors_thread_main, (argv) ? (const char **)&argv[2] : (const char **)NULL);
		}
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (!thread_running) {
			printf("sensors app not started\n");
		} else {
			printf("stopping sensors app\n");
			thread_should_exit = true;
		}
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tsensors app is running\n");
		} else {
			printf("\tsensors app not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

