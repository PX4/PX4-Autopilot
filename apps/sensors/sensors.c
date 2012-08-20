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

#include <pthread.h>
#include <fcntl.h>
#include <sys/prctl.h>
#include <nuttx/analog/adc.h>
#include <unistd.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <float.h>

#include <arch/board/up_hrt.h>
#include <arch/board/drv_lis331.h>
#include <arch/board/drv_bma180.h>
#include <arch/board/drv_l3gd20.h>
#include <arch/board/drv_hmc5883l.h>
#include <arch/board/up_adc.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>

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

static pthread_cond_t sensors_read_ready;
static pthread_mutex_t sensors_read_ready_mutex;

static int sensors_timer_loop_counter = 0;

/* File descriptors for all sensors */
static int fd_gyro = -1;
static int fd_accelerometer = -1;
static int fd_magnetometer = -1;
static int fd_barometer = -1;
static int fd_adc = -1;

static bool thread_should_exit = false;
static bool thread_running = false;
static int sensors_task;

/* Private functions declared static */
static void sensors_timer_loop(void *arg);

#ifdef CONFIG_HRT_PPM
extern uint16_t ppm_buffer[];
extern unsigned ppm_decoded_channels;
extern uint64_t ppm_last_valid_decode;
#endif

/* file handle that will be used for publishing sensor data */
static int sensor_pub;

PARAM_DEFINE_FLOAT(sensor_gyro_xoffset, 0.0f);
PARAM_DEFINE_FLOAT(sensor_gyro_yoffset, 0.0f);
PARAM_DEFINE_FLOAT(sensor_gyro_zoffset, 0.0f);

PARAM_DEFINE_FLOAT(sensor_mag_xoff, 0.0f);
PARAM_DEFINE_FLOAT(sensor_mag_yoff, 0.0f);
PARAM_DEFINE_FLOAT(sensor_mag_zoff, 0.0f);

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

PARAM_DEFINE_INT32(RC_MAP_ROLL, 1);
PARAM_DEFINE_INT32(RC_MAP_PITCH, 2);
PARAM_DEFINE_INT32(RC_MAP_THROTTLE, 3);
PARAM_DEFINE_INT32(RC_MAP_YAW, 4);
PARAM_DEFINE_INT32(RC_MAP_MODE_SW, 5);

/**
 * Sensor readout and publishing.
 * 
 * This function reads all onboard sensors and publishes the sensor_combined topic.
 *
 * @see sensor_combined_s
 * @ingroup apps
 */
__EXPORT int sensors_main(int argc, char *argv[]);

/**
 * Print the usage
 */
static void usage(const char *reason);

/**
 * Initialize all sensor drivers.
 *
 * @return 0 on success, != 0 on failure
 */
static int sensors_init(void)
{
	printf("[sensors] Sensor configuration..\n");

	/* open magnetometer */
	fd_magnetometer = open("/dev/hmc5883l", O_RDONLY);

	if (fd_magnetometer < 0) {
		fprintf(stderr, "[sensors]   HMC5883L open fail (err #%d): %s\n", (int)*get_errno_ptr(), strerror((int)*get_errno_ptr()));
		fflush(stderr);
		/* this sensor is critical, exit on failed init */
		errno = ENOSYS;
		return ERROR;

	} else {
		printf("[sensors]   HMC5883L open ok\n");
	}

	/* open barometer */
	fd_barometer = open("/dev/ms5611", O_RDONLY);

	if (fd_barometer < 0) {
		fprintf(stderr, "[sensors]   MS5611 open fail (err #%d): %s\n", (int)*get_errno_ptr(), strerror((int)*get_errno_ptr()));
		fflush(stderr);

	} else {
		printf("[sensors]   MS5611 open ok\n");
	}

	/* open gyro */
	fd_gyro = open("/dev/l3gd20", O_RDONLY);

	if (fd_gyro < 0) {
		fprintf(stderr, "[sensors]   L3GD20 open fail (err #%d): %s\n", (int)*get_errno_ptr(), strerror((int)*get_errno_ptr()));
		fflush(stderr);
		/* this sensor is critical, exit on failed init */
		errno = ENOSYS;
		return ERROR;

	} else {
		printf("[sensors]   L3GD20 open ok\n");
	}

	/* open accelerometer */
	fd_accelerometer = open("/dev/bma180", O_RDONLY);

	if (fd_accelerometer < 0) {
		fprintf(stderr, "[sensors]   BMA180: open fail (err #%d): %s\n", (int)*get_errno_ptr(), strerror((int)*get_errno_ptr()));
		fflush(stderr);
		/* this sensor is critical, exit on failed init */
		errno = ENOSYS;
		return ERROR;

	} else {
		printf("[sensors]   BMA180 open ok\n");
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

	/* configure gyro */
	if (ioctl(fd_gyro, L3GD20_SETRATE, L3GD20_RATE_760HZ_LP_30HZ) || ioctl(fd_gyro, L3GD20_SETRANGE, L3GD20_RANGE_500DPS)) {
		fprintf(stderr, "[sensors]   L3GD20 configuration (ioctl) fail (err #%d): %s\n", (int)*get_errno_ptr(), strerror((int)*get_errno_ptr()));
		fflush(stderr);
		/* this sensor is critical, exit on failed init */
		errno = ENOSYS;
		return ERROR;

	} else {
		printf("[sensors]   L3GD20 configuration ok\n");
	}

	/* XXX Add IOCTL configuration of remaining sensors */

	printf("[sensors] All sensors configured\n");
	return OK;
}

/**
 * Callback function called by high resolution timer.
 *
 * This function signals a pthread condition and wakes up the
 * sensor main loop.
 */
static void sensors_timer_loop(void *arg)
{
	/* Inform the read thread that it is now time to read */
	sensors_timer_loop_counter++;
	/* Do not use global data broadcast because of
	 * use of printf() in call - would be fatal here
	 */
	pthread_cond_broadcast(&sensors_read_ready);
}

int sensors_thread_main(int argc, char *argv[])
{
	/* inform about start */
	printf("[sensors] Initializing..\n");
	fflush(stdout);
	int ret = OK;

	/* start sensor reading */
	if (sensors_init() != OK) {
		fprintf(stderr, "[sensors] ERROR: Failed to initialize all sensors\n");
		/* Clean up */
		close(fd_gyro);
		close(fd_accelerometer);
		close(fd_magnetometer);
		close(fd_barometer);
		close(fd_adc);

		fprintf(stderr, "[sensors] rebooting system.\n");
		fflush(stderr);
		fflush(stdout);
		usleep(100000);

		/* Sensors are critical, immediately reboot system on failure */
		reboot();
		/* Not ever reaching here */

	} else {
		/* flush stdout from init routine */
		fflush(stdout);
	}

	bool gyro_healthy = false;
	bool acc_healthy = false;
	bool magn_healthy = false;
	bool baro_healthy = false;
	bool adc_healthy = false;

	bool hil_enabled = false;		/**< HIL is disabled by default	*/
	bool publishing = false;		/**< the app is not publishing by default, only if HIL is disabled on first run */

	int magcounter = 0;
	int barocounter = 0;
	int adccounter = 0;

	unsigned int mag_fail_count = 0;
	unsigned int mag_success_count = 0;

	unsigned int baro_fail_count = 0;
	unsigned int baro_success_count = 0;

	unsigned int gyro_fail_count = 0;
	unsigned int gyro_success_count = 0;

	unsigned int acc_fail_count = 0;
	unsigned int acc_success_count = 0;

	unsigned int adc_fail_count = 0;
	unsigned int adc_success_count = 0;

	ssize_t	ret_gyro;
	ssize_t	ret_accelerometer;
	ssize_t	ret_magnetometer;
	ssize_t	ret_barometer;
	ssize_t	ret_adc;
	int 	nsamples_adc;

	int16_t buf_gyro[3];
	int16_t buf_accelerometer[3];
	int16_t buf_magnetometer[7];
	float	buf_barometer[3];

	int16_t	mag_offset[3] = {0, 0, 0};
	int16_t acc_offset[3] = {200, 0, 0};
	int16_t	gyro_offset[3] = {0, 0, 0};

	bool mag_calibration_enabled = false;

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
	battery_voltage_conversion = global_data_parameter_storage->pm.param_values[PARAM_BATTERYVOLTAGE_CONVERSION];

	if (-1 == (int)battery_voltage_conversion) {
		/* default is conversion factor for the PX4IO / PX4IOAR board, the factor for PX4FMU standalone is different */
		battery_voltage_conversion = 3.3f * 52.0f / 5.0f / 4095.0f;
	}

#ifdef CONFIG_HRT_PPM
	int ppmcounter = 0;
#endif
	/* initialize to 100 to execute immediately */
	int paramcounter = 100;
	int excessive_readout_time_counter = 0;
	int read_loop_counter = 0;

	/* Empty sensor buffers, avoid junk values */
	/* Read first two values of each sensor into void */
	(void)read(fd_gyro, buf_gyro, sizeof(buf_gyro));
	(void)read(fd_accelerometer, buf_accelerometer, sizeof(buf_accelerometer));
	(void)read(fd_magnetometer, buf_magnetometer, sizeof(buf_magnetometer));

	if (fd_barometer > 0)(void)read(fd_barometer, buf_barometer, sizeof(buf_barometer));

	struct sensor_combined_s raw = {
		.timestamp = hrt_absolute_time(),
		.gyro_raw = {buf_gyro[0], buf_gyro[1], buf_gyro[2]},
		.gyro_raw_counter = 0,
		.gyro_rad_s = {0, 0, 0},
		.accelerometer_raw = {buf_accelerometer[0], buf_accelerometer[1], buf_accelerometer[2]},
		.accelerometer_raw_counter = 0,
		.accelerometer_m_s2 = {0, 0, 0},
		.magnetometer_raw = {buf_magnetometer[0], buf_magnetometer[1], buf_magnetometer[2]},
		.magnetometer_raw_counter = 0,
		.baro_pres_mbar = 0,
		.baro_alt_meter = 0,
		.baro_temp_celcius = 0,
		.battery_voltage_v = BAT_VOL_INITIAL,
		.adc_voltage_v = {0, 0 , 0},
		.baro_raw_counter = 0,
		.battery_voltage_counter = 0,
		.battery_voltage_valid = false,
	};

	/* condition to wait for */
	pthread_mutex_init(&sensors_read_ready_mutex, NULL);
	pthread_cond_init(&sensors_read_ready, NULL);

	/* advertise the sensor_combined topic and make the initial publication */
	sensor_pub = orb_advertise(ORB_ID(sensor_combined), &raw);
	publishing = true;

	/* advertise the manual_control topic */
	struct manual_control_setpoint_s manual_control = { .mode = ROLLPOS_PITCHPOS_YAWRATE_THROTTLE,
						   .roll = 0.0f,
						   .pitch = 0.0f,
						   .yaw = 0.0f,
						   .throttle = 0.0f };

	int manual_control_pub = orb_advertise(ORB_ID(manual_control_setpoint), &manual_control);

	if (manual_control_pub < 0) {
		fprintf(stderr, "[sensors] ERROR: orb_advertise for topic manual_control_setpoint failed.\n");
	}

	/* advertise the rc topic */
	struct rc_channels_s rc;
	memset(&rc, 0, sizeof(rc));
	int rc_pub = orb_advertise(ORB_ID(rc_channels), &rc);

	if (rc_pub < 0) {
		fprintf(stderr, "[sensors] ERROR: orb_advertise for topic rc_channels failed.\n");
	}

	/* subscribe to system status */
	struct vehicle_status_s vstatus;
	memset(&vstatus, 0, sizeof(vstatus));
	int vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));


	printf("[sensors] rate: %u Hz\n", (unsigned int)(1000000 / SENSOR_INTERVAL_MICROSEC));

	struct hrt_call sensors_hrt_call;
	/* Enable high resolution timer callback to unblock main thread, run after 2 ms */
	hrt_call_every(&sensors_hrt_call, 2000, SENSOR_INTERVAL_MICROSEC, &sensors_timer_loop, NULL);

	thread_running = true;

	while (!thread_should_exit) {
		pthread_mutex_lock(&sensors_read_ready_mutex);

		struct timespec time_to_wait = {0, 0};
		/* Wait 2 seconds until timeout */
		time_to_wait.tv_nsec = 0;
		time_to_wait.tv_sec = time(NULL) + 2;

		if (pthread_cond_timedwait(&sensors_read_ready, &sensors_read_ready_mutex, &time_to_wait) == OK) {
			pthread_mutex_unlock(&sensors_read_ready_mutex);

			bool gyro_updated = false;
			bool acc_updated = false;
			bool magn_updated = false;
			bool baro_updated = false;
			bool adc_updated = false;

			/* store the time closest to all measurements */
			uint64_t current_time = hrt_absolute_time();
			raw.timestamp = current_time;

			if (paramcounter == 100) {
				// XXX paramcounter is not a good name, rename / restructure
				// XXX make counter ticks dependent on update rate of sensor main loop

				/* Check HIL state */
				orb_copy(ORB_ID(vehicle_status), vstatus_sub, &vstatus);

				/* switching from non-HIL to HIL mode */
				//printf("[sensors] Vehicle mode: %i \t AND: %i, HIL: %i\n", vstatus.mode, vstatus.mode & VEHICLE_MODE_FLAG_HIL_ENABLED, hil_enabled);
				if (vstatus.flag_hil_enabled && !hil_enabled) {
					hil_enabled = true;
					publishing = false;
					int ret = close(sensor_pub);
					printf("[sensors] Closing sensor pub: %i \n", ret);

					/* switching from HIL to non-HIL mode */

				} else if (!publishing && !hil_enabled) {
					/* advertise the topic and make the initial publication */
					sensor_pub = orb_advertise(ORB_ID(sensor_combined), &raw);
					hil_enabled = false;
					publishing = true;
				}


				/* Update RC scalings and function mappings */
				rc.chan[0].scaling_factor = (1 / ((global_data_parameter_storage->pm.param_values[PARAM_RC1_MAX] - global_data_parameter_storage->pm.param_values[PARAM_RC1_MIN]) / 2)
							     * global_data_parameter_storage->pm.param_values[PARAM_RC1_REV]);
				rc.chan[0].mid = (uint16_t)global_data_parameter_storage->pm.param_values[PARAM_RC1_TRIM];

				rc.chan[1].scaling_factor = (1 / ((global_data_parameter_storage->pm.param_values[PARAM_RC2_MAX] - global_data_parameter_storage->pm.param_values[PARAM_RC2_MIN]) / 2)
							     * global_data_parameter_storage->pm.param_values[PARAM_RC2_REV]);
				rc.chan[1].mid = (uint16_t)global_data_parameter_storage->pm.param_values[PARAM_RC2_TRIM];

				rc.chan[2].scaling_factor = (1 / ((global_data_parameter_storage->pm.param_values[PARAM_RC3_MAX] - global_data_parameter_storage->pm.param_values[PARAM_RC3_MIN]) / 2)
							     * global_data_parameter_storage->pm.param_values[PARAM_RC3_REV]);
				rc.chan[2].mid = (uint16_t)global_data_parameter_storage->pm.param_values[PARAM_RC3_TRIM];

				rc.chan[3].scaling_factor = (1 / ((global_data_parameter_storage->pm.param_values[PARAM_RC4_MAX] - global_data_parameter_storage->pm.param_values[PARAM_RC4_MIN]) / 2)
							     * global_data_parameter_storage->pm.param_values[PARAM_RC4_REV]);
				rc.chan[3].mid = (uint16_t)global_data_parameter_storage->pm.param_values[PARAM_RC4_TRIM];

				rc.chan[4].scaling_factor = (1 / ((global_data_parameter_storage->pm.param_values[PARAM_RC5_MAX] - global_data_parameter_storage->pm.param_values[PARAM_RC5_MIN]) / 2)
							     * global_data_parameter_storage->pm.param_values[PARAM_RC5_REV]);
				rc.chan[4].mid = (uint16_t)global_data_parameter_storage->pm.param_values[PARAM_RC5_TRIM];

				rc.function[0] = global_data_parameter_storage->pm.param_values[PARAM_THROTTLE_CHAN] - 1;
				rc.function[1] = global_data_parameter_storage->pm.param_values[PARAM_ROLL_CHAN] - 1;
				rc.function[2] = global_data_parameter_storage->pm.param_values[PARAM_PITCH_CHAN] - 1;
				rc.function[3] = global_data_parameter_storage->pm.param_values[PARAM_YAW_CHAN] - 1;
				rc.function[4] = global_data_parameter_storage->pm.param_values[PARAM_OVERRIDE_CHAN] - 1;

				gyro_offset[0] = global_data_parameter_storage->pm.param_values[PARAM_SENSOR_GYRO_XOFFSET];
				gyro_offset[1] = global_data_parameter_storage->pm.param_values[PARAM_SENSOR_GYRO_YOFFSET];
				gyro_offset[2] = global_data_parameter_storage->pm.param_values[PARAM_SENSOR_GYRO_ZOFFSET];

				mag_offset[0] = global_data_parameter_storage->pm.param_values[PARAM_SENSOR_MAG_XOFFSET];
				mag_offset[1] = global_data_parameter_storage->pm.param_values[PARAM_SENSOR_MAG_YOFFSET];
				mag_offset[2] = global_data_parameter_storage->pm.param_values[PARAM_SENSOR_MAG_ZOFFSET];


				paramcounter = 0;
			}

			paramcounter++;

			/* try reading gyro */
			uint64_t start_gyro = hrt_absolute_time();
			ret_gyro = read(fd_gyro, buf_gyro, sizeof(buf_gyro));
			int gyrotime = hrt_absolute_time() - start_gyro;

			if (gyrotime > 500) printf("GYRO (pure read): %d us\n", gyrotime);

			/* GYROSCOPE */
			if (ret_gyro != sizeof(buf_gyro)) {
				gyro_fail_count++;

				if ((((gyro_fail_count % 20) == 0) || (gyro_fail_count > 20 && gyro_fail_count < 100)) && (int)*get_errno_ptr() != EAGAIN) {
					fprintf(stderr, "[sensors] L3GD20 ERROR #%d: %s\n", (int)*get_errno_ptr(), strerror((int)*get_errno_ptr()));
				}

				if (gyro_healthy && gyro_fail_count >= GYRO_HEALTH_COUNTER_LIMIT_ERROR) {
					// global_data_send_subsystem_info(&gyro_present_enabled);
					gyro_healthy = false;
					gyro_success_count = 0;
				}

			} else {
				gyro_success_count++;

				if (!gyro_healthy && gyro_success_count >= GYRO_HEALTH_COUNTER_LIMIT_OK) {
					// global_data_send_subsystem_info(&gyro_present_enabled_healthy);
					gyro_healthy = true;
					gyro_fail_count = 0;

				}

				gyro_updated = true;
			}

			gyrotime = hrt_absolute_time() - start_gyro;

			if (gyrotime > 500) printf("GYRO (complete): %d us\n", gyrotime);

			/* try reading acc */
			uint64_t start_acc = hrt_absolute_time();
			ret_accelerometer = read(fd_accelerometer, buf_accelerometer, sizeof(buf_accelerometer));

			/* ACCELEROMETER */
			if (ret_accelerometer != sizeof(buf_accelerometer)) {
				acc_fail_count++;

				if (acc_fail_count & 0b111 || (acc_fail_count > 20 && acc_fail_count < 100)) {
					fprintf(stderr, "[sensors] BMA180 ERROR #%d: %s\n", (int)*get_errno_ptr(), strerror((int)*get_errno_ptr()));
				}

				if (acc_healthy && acc_fail_count >= ACC_HEALTH_COUNTER_LIMIT_ERROR) {
					// global_data_send_subsystem_info(&acc_present_enabled);
					gyro_healthy = false;
					acc_success_count = 0;
				}

			} else {
				acc_success_count++;

				if (!acc_healthy && acc_success_count >= ACC_HEALTH_COUNTER_LIMIT_OK) {

					// global_data_send_subsystem_info(&acc_present_enabled_healthy);
					acc_healthy = true;
					acc_fail_count = 0;

				}

				acc_updated = true;
			}

			int acctime = hrt_absolute_time() - start_acc;

			if (acctime > 500) printf("ACC: %d us\n", acctime);

			/* MAGNETOMETER */
			if (magcounter == 4) { /* 120 Hz */
				uint64_t start_mag = hrt_absolute_time();
				/* start calibration mode if requested */
				if (!mag_calibration_enabled && vstatus.preflight_mag_calibration) {
					ioctl(fd_magnetometer, HMC5883L_CALIBRATION_ON, 0);
					printf("[sensors] enabling mag calibration mode\n");
					mag_calibration_enabled = true;
				} else if (mag_calibration_enabled && !vstatus.preflight_mag_calibration) {
					ioctl(fd_magnetometer, HMC5883L_CALIBRATION_OFF, 0);
					printf("[sensors] disabling mag calibration mode\n");
					mag_calibration_enabled = false;
				}

				ret_magnetometer = read(fd_magnetometer, buf_magnetometer, sizeof(buf_magnetometer));
				int errcode_mag = (int) * get_errno_ptr();
				int magtime = hrt_absolute_time() - start_mag;

				if (magtime > 2000) {
					printf("MAG (pure read): %d us\n", magtime);
				}

				if (ret_magnetometer != sizeof(buf_magnetometer)) {
					mag_fail_count++;

					if (mag_fail_count & 0b111 || (mag_fail_count > 20 && mag_fail_count < 100)) {
						fprintf(stderr, "[sensors] HMC5883L ERROR #%d: %s\n", (int)*get_errno_ptr(), strerror((int)*get_errno_ptr()));
					}

					if (magn_healthy && mag_fail_count >= MAGN_HEALTH_COUNTER_LIMIT_ERROR) {
						// global_data_send_subsystem_info(&magn_present_enabled);
						magn_healthy = false;
						mag_success_count = 0;
					}

				} else {
					mag_success_count++;

					if (!magn_healthy && mag_success_count >= MAGN_HEALTH_COUNTER_LIMIT_OK) {
						// global_data_send_subsystem_info(&magn_present_enabled_healthy);
						magn_healthy = true;
						mag_fail_count = 0;
					}

					magn_updated = true;
				}

				magtime = hrt_absolute_time() - start_mag;

				if (magtime > 2000) {
					printf("MAG (overall time): %d us\n", magtime);
					fprintf(stderr, "[sensors] TIMEOUT HMC5883L ERROR #%d: %s\n", errcode_mag, strerror(errcode_mag));
				}

				magcounter = 0;
			}

			magcounter++;

			/* BAROMETER */
			if (barocounter == 5 && (fd_barometer > 0)) { /* 100 Hz */
				uint64_t start_baro = hrt_absolute_time();
				*get_errno_ptr() = 0;
				ret_barometer = read(fd_barometer, buf_barometer, sizeof(buf_barometer));

				if (ret_barometer != sizeof(buf_barometer)) {
					baro_fail_count++;

					if ((baro_fail_count & 0b1000 || (baro_fail_count > 20 && baro_fail_count < 100)) && (int)*get_errno_ptr() != EAGAIN) {
						fprintf(stderr, "[sensors] MS5611 ERROR #%d: %s\n", (int)*get_errno_ptr(), strerror((int)*get_errno_ptr()));
					}

					if (baro_healthy && baro_fail_count >= BARO_HEALTH_COUNTER_LIMIT_ERROR) {
						/* switched from healthy to unhealthy */
						baro_healthy = false;
						baro_success_count = 0;
						// global_data_send_subsystem_info(&baro_present_enabled);
					}

				} else {
					baro_success_count++;

					if (!baro_healthy && baro_success_count >= MAGN_HEALTH_COUNTER_LIMIT_OK) {
						/* switched from unhealthy to healthy */
						baro_healthy = true;
						baro_fail_count = 0;
						// global_data_send_subsystem_info(&baro_present_enabled_healthy);
					}

					baro_updated = true;
				}

				barocounter = 0;
				int barotime = hrt_absolute_time() - start_baro;

				if (barotime > 2000) printf("BARO: %d us\n", barotime);
			}

			barocounter++;

			/* ADC */
			if (adccounter == 5) {
				ret_adc = read(fd_adc, &buf_adc, adc_readsize);
				nsamples_adc = ret_adc / sizeof(struct adc_msg_s);

				if (ret_adc  < 0 || nsamples_adc * sizeof(struct adc_msg_s) != ret_adc) {
					adc_fail_count++;

					if ((adc_fail_count & 0b1000 || adc_fail_count < 10) && (int)*get_errno_ptr() != EAGAIN) {
						fprintf(stderr, "[sensors] ADC ERROR #%d: %s\n", (int)*get_errno_ptr(), strerror((int)*get_errno_ptr()));
					}

					if (adc_healthy && adc_fail_count >= ADC_HEALTH_COUNTER_LIMIT_ERROR) {
						adc_healthy = false;
						adc_success_count = 0;
					}

				} else {
					adc_success_count++;

					if (!adc_healthy && adc_success_count >= ADC_HEALTH_COUNTER_LIMIT_OK) {
						adc_healthy = true;
						adc_fail_count = 0;
					}

					adc_updated = true;
				}

				adccounter = 0;

			}

			adccounter++;



#ifdef CONFIG_HRT_PPM
			bool ppm_updated = false;

			/* PPM */
			if (ppmcounter == 5) {

				/* require at least two channels
				 * to consider the signal valid
				 * check that decoded measurement is up to date
				 */
				if (ppm_decoded_channels > 1 && (hrt_absolute_time() - ppm_last_valid_decode) < 45000) {
					/* Read out values from HRT */
					for (int i = 0; i < ppm_decoded_channels; i++) {
						rc.chan[i].raw = ppm_buffer[i];
						/* Set the range to +-, then scale up */
						rc.chan[i].scale = (ppm_buffer[i] - rc.chan[i].mid) * rc.chan[i].scaling_factor * 10000;
						rc.chan[i].scaled = (ppm_buffer[i] - rc.chan[i].mid) * rc.chan[i].scaling_factor;
					}

					rc.chan_count = ppm_decoded_channels;
					rc.timestamp = ppm_last_valid_decode;

					/* publish a few lines of code later if set to true */
					ppm_updated = true;

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
					manual_control.throttle = rc.chan[rc.function[THROTTLE]].scaled/2.0f;
					if (manual_control.throttle < 0.0f) manual_control.throttle = 0.0f;
					if (manual_control.throttle > 1.0f) manual_control.throttle = 1.0f;

					/* mode switch input */
					manual_control.override_mode_switch = rc.chan[rc.function[OVERRIDE]].scaled;
					if (manual_control.override_mode_switch < -1.0f) manual_control.override_mode_switch = -1.0f;
					if (manual_control.override_mode_switch >  1.0f) manual_control.override_mode_switch =  1.0f;

				}
				ppmcounter = 0;
			}

			ppmcounter++;
#endif

			/* Copy values of gyro, acc, magnetometer & barometer */

			/* GYROSCOPE */
			if (gyro_updated) {
				/* copy sensor readings to global data and transform coordinates into px4fmu board frame */

				raw.gyro_raw[0] = ((buf_gyro[1] == -32768) ? -32768 : buf_gyro[1]); // x of the board is y of the sensor
				/* assign negated value, except for -SHORT_MAX, as it would wrap there */
				raw.gyro_raw[1] = ((buf_gyro[0] == -32768) ? 32767 : -buf_gyro[0]); // y on the board is -x of the sensor
				raw.gyro_raw[2] = ((buf_gyro[2] == -32768) ? -32768 : buf_gyro[2]); // z of the board is z of the sensor

				/* scale measurements */
				// XXX request scaling from driver instead of hardcoding it
				/* scaling calculated as: raw * (1/(32768*(500/180*PI))) */
				raw.gyro_rad_s[0] = (raw.gyro_raw[0] - gyro_offset[0]) * 0.000266316109f;
				raw.gyro_rad_s[1] = (raw.gyro_raw[1] - gyro_offset[1]) * 0.000266316109f;
				raw.gyro_rad_s[2] = (raw.gyro_raw[2] - gyro_offset[2]) * 0.000266316109f;

				raw.gyro_raw_counter++;
			}

			/* ACCELEROMETER */
			if (acc_updated) {
				/* copy sensor readings to global data and transform coordinates into px4fmu board frame */

				/* assign negated value, except for -SHORT_MAX, as it would wrap there */
				raw.accelerometer_raw[0] = (buf_accelerometer[1] == -32768) ? 32767 : -buf_accelerometer[1]; // x of the board is -y of the sensor
				raw.accelerometer_raw[1] = (buf_accelerometer[0] == -32768) ? -32768 : buf_accelerometer[0]; // y on the board is x of the sensor
				raw.accelerometer_raw[2] = (buf_accelerometer[2] == -32768) ? -32768 : buf_accelerometer[2]; // z of the board is z of the sensor

				// XXX read range from sensor
				float range_g = 4.0f;
				/* scale from 14 bit to m/s2 */
				raw.accelerometer_m_s2[0] = (((raw.accelerometer_raw[0] - acc_offset[0]) / 8192.0f) * range_g) * 9.81f;
				raw.accelerometer_m_s2[1] = (((raw.accelerometer_raw[1] - acc_offset[1]) / 8192.0f) * range_g) * 9.81f;
				raw.accelerometer_m_s2[2] = (((raw.accelerometer_raw[2] - acc_offset[2]) / 8192.0f) * range_g) * 9.81f;

				raw.accelerometer_raw_counter++;
			}

			/* MAGNETOMETER */
			if (magn_updated) {
				/* copy sensor readings to global data and transform coordinates into px4fmu board frame */

				/* assign negated value, except for -SHORT_MAX, as it would wrap there */
				raw.magnetometer_raw[0] = (buf_magnetometer[1] == -32768) ? 32767 : buf_magnetometer[1];  // x of the board is y of the sensor
				raw.magnetometer_raw[1] = (buf_magnetometer[0] == -32768) ? 32767 : -buf_magnetometer[0]; // y on the board is -x of the sensor
				raw.magnetometer_raw[2] = (buf_magnetometer[2] == -32768) ? -32768 : buf_magnetometer[2]; // z of the board is z of the sensor

				// XXX Read out mag range via I2C on init, assuming 0.88 Ga and 12 bit res here
				raw.magnetometer_ga[0] = ((raw.magnetometer_raw[0] - mag_offset[0]) / 4096.0f) * 0.88f;
				raw.magnetometer_ga[1] = ((raw.magnetometer_raw[1] - mag_offset[1]) / 4096.0f) * 0.88f;
				raw.magnetometer_ga[2] = ((raw.magnetometer_raw[2] - mag_offset[2]) / 4096.0f) * 0.88f;

				/* store mode */
				raw.magnetometer_mode = buf_magnetometer[3];

				raw.magnetometer_raw_counter++;
			}

			/* BAROMETER */
			if (baro_updated) {
				/* copy sensor readings to global data and transform coordinates into px4fmu board frame */

				raw.baro_pres_mbar = buf_barometer[0]; // Pressure in mbar
				raw.baro_alt_meter = buf_barometer[1]; // Altitude in meters
				raw.baro_temp_celcius = buf_barometer[2]; // Temperature in degrees celcius

				raw.baro_raw_counter++;
			}

			/* ADC */
			if (adc_updated) {
				/* copy sensor readings to global data*/

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
			}

			uint64_t total_time = hrt_absolute_time() - current_time;

			/* Inform other processes that new data is available to copy */
			if ((gyro_updated || acc_updated || magn_updated || baro_updated) && publishing) {
				/* Values changed, publish */
				orb_publish(ORB_ID(sensor_combined), sensor_pub, &raw);
			}

#ifdef CONFIG_HRT_PPM

			if (ppm_updated) {
				orb_publish(ORB_ID(rc_channels), rc_pub, &rc);
				orb_publish(ORB_ID(manual_control_setpoint), manual_control_pub, &manual_control);
			}

#endif

			if (total_time > 2600) {
				excessive_readout_time_counter++;
			}

			if (total_time > 2600 && excessive_readout_time_counter > 100 && excessive_readout_time_counter % 100 == 0) {
				fprintf(stderr, "[sensors] slow update (>2600 us): %d us (#%d)\n", (int)total_time, excessive_readout_time_counter);

			} else if (total_time > 6000) {
				if (excessive_readout_time_counter < 100 || excessive_readout_time_counter % 100 == 0) fprintf(stderr, "[sensors] WARNING: Slow update (>6000 us): %d us (#%d)\n", (int)total_time, excessive_readout_time_counter);
			}


			read_loop_counter++;
#ifdef CONFIG_SENSORS_DEBUG_ENABLED

			if (read_loop_counter % 1000 == 0) printf("[sensors] read loop counter: %d\n", read_loop_counter);

			fflush(stdout);

			if (sensors_timer_loop_counter % 1000 == 0) printf("[sensors] timer/trigger loop counter: %d\n", sensors_timer_loop_counter);

#endif
		}

		if (thread_should_exit) break;
	}

	/* Never really getting here */
	printf("[sensors] sensor readout stopped\n");

	close(fd_gyro);
	close(fd_accelerometer);
	close(fd_magnetometer);
	close(fd_barometer);
	close(fd_adc);

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

