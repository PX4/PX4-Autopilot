/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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

#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>

#include <px4_tasks.h>

#include <px4_log.h>
#include <px4_getopt.h>

// TODO-JYW:
// Include references to the driver framework.  This will produce a duplicate definition
// of qurt_log.  Use a #define for the qurt_log function to avoid redefinition.  This code
// change must still be made.  Document the latter change to be clear.
// #include <df_stubs.h>
// #include <SPIDevObj.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>

#ifdef __cplusplus
extern "C" {
#endif
#include <semaphore.h>
#include <mpu9x50.h>
#ifdef __cplusplus
}
#endif

/** driver 'main' command */
extern "C" { __EXPORT int mpu9x50_main(int argc, char *argv[]); }

#define MAX_LEN_DEV_PATH 32

// TODO - need to load this from parameter file
#define SPI_INT_GPIO 65  // GPIO pin for Data Ready Interrupt
namespace mpu9x50
{

/** SPI device path that mpu9x50 is connected to */
static char _device[MAX_LEN_DEV_PATH];

/** flag indicating if mpu9x50 app is running */
static bool _is_running = false;

/** flag indicating if measurement thread should exit */
static bool _task_should_exit = false;

/** handle to the task main thread */
static px4_task_t _task_handle = -1;

/** IMU measurement data */
static struct mpu9x50_data _data;

static orb_advert_t _gyro_pub = nullptr;	/**< gyro data publication */
static int _gyro_orb_class_instance;        /**< instance handle for gyro devices */
static orb_advert_t _accel_pub = nullptr;	/**< accelerameter data publication */
static int _accel_orb_class_instance;        /**< instance handle for accel devices */
static orb_advert_t _mag_pub = nullptr;		/**< compass data publication */
static int _mag_orb_class_instance;        /**< instance handle for mag devices */
static int _params_sub;										/**< parameter update subscription */
static struct gyro_report _gyro;					/**< gyro report */
static struct accel_report _accel;				/**< accel report */
static struct mag_report _mag;						/**< mag report */
static struct gyro_calibration_s _gyro_sc;				/**< gyro scale */
static struct accel_calibration_s _accel_sc;			/**< accel scale */
static struct mag_calibration_s _mag_sc;					/**< mag scale */
static enum gyro_lpf_e _gyro_lpf = MPU9X50_GYRO_LPF_20HZ;	/**< gyro lpf enum value */
static enum acc_lpf_e _accel_lpf = MPU9X50_ACC_LPF_20HZ;	/**< accel lpf enum value */
static enum gyro_sample_rate_e _imu_sample_rate = MPU9x50_SAMPLE_RATE_500HZ;	/**< IMU sample rate enum */

struct {
	param_t accel_x_offset;
	param_t accel_x_scale;
	param_t accel_y_offset;
	param_t accel_y_scale;
	param_t accel_z_offset;
	param_t accel_z_scale;
	param_t gyro_x_offset;
	param_t gyro_x_scale;
	param_t gyro_y_offset;
	param_t gyro_y_scale;
	param_t gyro_z_offset;
	param_t gyro_z_scale;
	param_t mag_x_offset;
	param_t mag_x_scale;
	param_t mag_y_offset;
	param_t mag_y_scale;
	param_t mag_z_offset;
	param_t mag_z_scale;
	param_t gyro_lpf_enum;
	param_t accel_lpf_enum;
	param_t imu_sample_rate_enum;
} _params_handles;  /**< parameter handles */

bool exit_mreasurement = false;


/** Print out the usage information */
static void usage();

/** mpu9x50 stop measurement */
static void stop();

/** task main trampoline function */
static void	task_main_trampoline(int argc, char *argv[]);

/** mpu9x50 measurement thread primary entry point */
static void task_main(int argc, char *argv[]);

/**
 * create and advertise all publicatoins
 * @return   true on success, false otherwise
 */
static bool create_pubs();

/** update all sensor reports with the latest sensor reading */
static void update_reports();

/** publish all sensor reports */
static void publish_reports();

/** update all parameters */
static void parameters_update();

/** initialize all parameter handles and load the initial parameter values */
static void parameters_init();

/** poll parameter update */
static void parameter_update_poll();

static int64_t _isr_data_ready_timestamp = 0;

/**
 * MPU9x50 data ready interrupt service routine
 * @param[out]	context	address of the context data provided by user whill
 *							registering the interrupt servcie routine
 */
static void *data_ready_isr(void *context);

void *data_ready_isr(void *context)
{
	if (exit_mreasurement) {
		return NULL;
	}

	_isr_data_ready_timestamp = hrt_absolute_time();
	// send signal to measurement thread
	px4_task_kill(_task_handle, SIGRTMIN);

	return NULL;
}

void parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}

void parameters_update()
{
	PX4_DEBUG("mpu9x50_parameters_update");
	float v;
	int v_int;

	// accel params
	if (param_get(_params_handles.accel_x_offset, &v) == 0) {
		_accel_sc.x_offset = v;
		PX4_DEBUG("mpu9x50_parameters_update accel_x_offset %f", v);
	}

	if (param_get(_params_handles.accel_x_scale, &v) == 0) {
		_accel_sc.x_scale  = v;
		PX4_DEBUG("mpu9x50_parameters_update accel_x_scale %f", v);
	}

	if (param_get(_params_handles.accel_y_offset, &v) == 0) {
		_accel_sc.y_offset  = v;
		PX4_DEBUG("mpu9x50_parameters_update accel_y_offset %f", v);
	}

	if (param_get(_params_handles.accel_y_scale, &v) == 0) {
		_accel_sc.y_scale  = v;
		PX4_DEBUG("mpu9x50_parameters_update accel_y_scale %f", v);
	}

	if (param_get(_params_handles.accel_z_offset, &v) == 0) {
		_accel_sc.z_offset  = v;
		PX4_DEBUG("mpu9x50_parameters_update accel_z_offset %f", v);
	}

	if (param_get(_params_handles.accel_z_scale, &v) == 0) {
		_accel_sc.z_scale  = v;
		PX4_DEBUG("mpu9x50_parameters_update accel_z_scale %f", v);
	}

	// gyro params
	if (param_get(_params_handles.gyro_x_offset, &v) == 0) {
		_gyro_sc.x_offset = v;
		PX4_DEBUG("mpu9x50_parameters_update gyro_x_offset %f", v);
	}

	if (param_get(_params_handles.gyro_x_scale, &v) == 0) {
		_gyro_sc.x_scale  = v;
		PX4_DEBUG("mpu9x50_parameters_update gyro_x_scale %f", v);
	}

	if (param_get(_params_handles.gyro_y_offset, &v) == 0) {
		_gyro_sc.y_offset  = v;
		PX4_DEBUG("mpu9x50_parameters_update gyro_y_offset %f", v);
	}

	if (param_get(_params_handles.gyro_y_scale, &v) == 0) {
		_gyro_sc.y_scale  = v;
		PX4_DEBUG("mpu9x50_parameters_update gyro_y_scale %f", v);
	}

	if (param_get(_params_handles.gyro_z_offset, &v) == 0) {
		_gyro_sc.z_offset  = v;
		PX4_DEBUG("mpu9x50_parameters_update gyro_z_offset %f", v);
	}

	if (param_get(_params_handles.gyro_z_scale, &v) == 0) {
		_gyro_sc.z_scale  = v;
		PX4_DEBUG("mpu9x50_parameters_update gyro_z_scale %f", v);
	}

	// mag params
	if (param_get(_params_handles.mag_x_offset, &v) == 0) {
		_mag_sc.x_offset = v;
		PX4_DEBUG("mpu9x50_parameters_update mag_x_offset %f", v);
	}

	if (param_get(_params_handles.mag_x_scale, &v) == 0) {
		_mag_sc.x_scale  = v;
		PX4_DEBUG("mpu9x50_parameters_update mag_x_scale %f", v);
	}

	if (param_get(_params_handles.mag_y_offset, &v) == 0) {
		_mag_sc.y_offset  = v;
		PX4_DEBUG("mpu9x50_parameters_update mag_y_offset %f", v);
	}

	if (param_get(_params_handles.mag_y_scale, &v) == 0) {
		_mag_sc.y_scale  = v;
		PX4_DEBUG("mpu9x50_parameters_update mag_y_scale %f", v);
	}

	if (param_get(_params_handles.mag_z_offset, &v) == 0) {
		_mag_sc.z_offset  = v;
		PX4_DEBUG("mpu9x50_parameters_update mag_z_offset %f", v);
	}

	if (param_get(_params_handles.mag_z_scale, &v) == 0) {
		_mag_sc.z_scale  = v;
		PX4_DEBUG("mpu9x50_parameters_update mag_z_scale %f", v);
	}

	// LPF params
	if (param_get(_params_handles.gyro_lpf_enum, &v_int) == 0) {
		if (v_int >= NUM_MPU9X50_GYRO_LPF) {
			PX4_WARN("invalid gyro_lpf_enum %d use default %d", v_int, _gyro_lpf);

		} else {
			_gyro_lpf = (enum gyro_lpf_e)v_int;
			PX4_DEBUG("mpu9x50_parameters_update gyro_lpf_enum %d", _gyro_lpf);
		}
	}

	if (param_get(_params_handles.accel_lpf_enum, &v_int) == 0) {
		if (v_int >= NUM_MPU9X50_ACC_LPF) {
			PX4_WARN("invalid accel_lpf_enum %d use default %d", v_int, _accel_lpf);

		} else {
			_accel_lpf = (enum acc_lpf_e)v_int;
			PX4_DEBUG("mpu9x50_parameters_update accel_lpf_enum %d", _accel_lpf);
		}
	}

	if (param_get(_params_handles.imu_sample_rate_enum, &v_int) == 0) {
		if (v_int >= NUM_MPU9X50_SAMPLE_RATE) {
			PX4_WARN("invalid imu_sample_rate %d use default %d", v_int, _imu_sample_rate);

		} else {
			_imu_sample_rate = (enum gyro_sample_rate_e)v_int;
			PX4_DEBUG("mpu9x50_parameters_update imu_sample_rate %d", _imu_sample_rate);
		}
	}
}

void parameters_init()
{
	_params_handles.accel_x_offset		=	param_find("CAL_ACC0_XOFF");
	_params_handles.accel_x_scale		=	param_find("CAL_ACC0_XSCALE");
	_params_handles.accel_y_offset		=	param_find("CAL_ACC0_YOFF");
	_params_handles.accel_y_scale		=	param_find("CAL_ACC0_YSCALE");
	_params_handles.accel_z_offset		=	param_find("CAL_ACC0_ZOFF");
	_params_handles.accel_z_scale		=	param_find("CAL_ACC0_ZSCALE");

	_params_handles.gyro_x_offset		=	param_find("CAL_GYRO0_XOFF");
	_params_handles.gyro_x_scale		=	param_find("CAL_GYRO0_XSCALE");
	_params_handles.gyro_y_offset		=	param_find("CAL_GYRO0_YOFF");
	_params_handles.gyro_y_scale		=	param_find("CAL_GYRO0_YSCALE");
	_params_handles.gyro_z_offset		=	param_find("CAL_GYRO0_ZOFF");
	_params_handles.gyro_z_scale		=	param_find("CAL_GYRO0_ZSCALE");

	_params_handles.mag_x_offset		=	param_find("CAL_MAG0_XOFF");
	_params_handles.mag_x_scale		=	param_find("CAL_MAG0_XSCALE");
	_params_handles.mag_y_offset		=	param_find("CAL_MAG0_YOFF");
	_params_handles.mag_y_scale		=	param_find("CAL_MAG0_YSCALE");
	_params_handles.mag_z_offset		=	param_find("CAL_MAG0_ZOFF");
	_params_handles.mag_z_scale		=	param_find("CAL_MAG0_ZSCALE");

	_params_handles.gyro_lpf_enum		=	param_find("MPU_GYRO_LPF_ENM");
	_params_handles.accel_lpf_enum		=	param_find("MPU_ACC_LPF_ENM");

	_params_handles.imu_sample_rate_enum	=	param_find("MPU_SAMPLE_R_ENM");

	parameters_update();
}

bool create_pubs()
{
	// initialize the reports
	memset(&_gyro, 0, sizeof(struct gyro_report));
	memset(&_accel, 0, sizeof(struct accel_report));
	memset(&_mag, 0, sizeof(struct mag_report));

	_gyro_pub = orb_advertise_multi(ORB_ID(sensor_gyro), &_gyro,
					&_gyro_orb_class_instance, ORB_PRIO_HIGH - 1);

	if (_gyro_pub == nullptr) {
		PX4_ERR("sensor_gyro advert fail");
		return false;
	}

	_accel_pub = orb_advertise_multi(ORB_ID(sensor_accel), &_accel,
					 &_accel_orb_class_instance, ORB_PRIO_HIGH - 1);

	if (_accel_pub == nullptr) {
		PX4_ERR("sensor_accel advert fail");
		return false;
	}

	_mag_pub = orb_advertise_multi(ORB_ID(sensor_mag), &_mag,
				       &_mag_orb_class_instance, ORB_PRIO_HIGH - 1);

	if (_mag_pub == nullptr) {
		PX4_ERR("sensor_mag advert fail");
		return false;
	}

	return true;
}

void update_reports()
{
	_gyro.timestamp = _data.timestamp;
	_gyro.x = ((_data.gyro_raw[0] * _data.gyro_scaling) - _gyro_sc.x_offset) * _gyro_sc.x_scale;
	_gyro.y = ((_data.gyro_raw[1] * _data.gyro_scaling) - _gyro_sc.y_offset) * _gyro_sc.y_scale;
	_gyro.z = ((_data.gyro_raw[2] * _data.gyro_scaling) - _gyro_sc.z_offset) * _gyro_sc.z_scale;
	_gyro.temperature = _data.temperature;
	_gyro.range_rad_s = _data.gyro_range_rad_s;
	_gyro.scaling = _data.gyro_scaling;
	_gyro.x_raw = _data.gyro_raw[0];
	_gyro.y_raw = _data.gyro_raw[1];
	_gyro.z_raw = _data.gyro_raw[2];
	_gyro.temperature_raw = _data.temperature_raw;

	_accel.timestamp = _data.timestamp;
	_accel.x = ((_data.accel_raw[0] * _data.accel_scaling) - _accel_sc.x_offset) * _accel_sc.x_scale;
	_accel.y = ((_data.accel_raw[1] * _data.accel_scaling) - _accel_sc.y_offset) * _accel_sc.y_scale;
	_accel.z = ((_data.accel_raw[2] * _data.accel_scaling) - _accel_sc.z_offset) * _accel_sc.z_scale;
	_accel.temperature = _data.temperature;
	_accel.range_m_s2 = _data.accel_range_m_s2;
	_accel.scaling = _data.accel_scaling;
	_accel.x_raw = _data.accel_raw[0];
	_accel.y_raw = _data.accel_raw[1];
	_accel.z_raw = _data.accel_raw[2];
	_accel.temperature_raw = _data.temperature_raw;

	if (_data.mag_data_ready) {
		_mag.timestamp = _data.timestamp;
		_mag.x = ((_data.mag_raw[0] * _data.mag_scaling) - _mag_sc.x_offset) * _mag_sc.x_scale;
		_mag.y = ((_data.mag_raw[1] * _data.mag_scaling) - _mag_sc.y_offset) * _mag_sc.y_scale;
		_mag.z = ((_data.mag_raw[2] * _data.mag_scaling) - _mag_sc.z_offset) * _mag_sc.z_scale;
		_mag.range_ga = _data.mag_range_ga;
		_mag.scaling = _data.mag_scaling;
		_mag.temperature = _data.temperature;
		_mag.x_raw = _data.mag_raw[0];
		_mag.y_raw = _data.mag_raw[1];
		_mag.z_raw = _data.mag_raw[2];
	}
}

void publish_reports()
{
	if (orb_publish(ORB_ID(sensor_gyro), _gyro_pub, &_gyro) != OK) {
		PX4_WARN("failed to publish gyro report");

	} else {
		//PX4_DEBUG("MPU_GYRO_RAW: %d %d %d", _gyro.x_raw, _gyro.y_raw, _gyro.z_raw)
		//PX4_DEBUG("MPU_GYRO: %f %f %f", _gyro.x, _gyro.y, _gyro.z)
	}

	if (orb_publish(ORB_ID(sensor_accel), _accel_pub, &_accel) != OK) {
		PX4_WARN("failed to publish accel report");

	} else {
		//PX4_DEBUG("MPU_ACCEL_RAW: %d %d %d", _accel.x_raw, _accel.y_raw, _accel.z_raw)
		//PX4_DEBUG("MPU_ACCEL: %f %f %f", _accel.x, _accel.y, _accel.z)
	}

	if (_data.mag_data_ready) {
		if (orb_publish(ORB_ID(sensor_mag), _mag_pub, &_mag) != OK) {
			PX4_WARN("failed to publish mag report");

		} else {
			//PX4_DEBUG("MPU_MAG_RAW: %d %d %d", _mag.x_raw, _mag.y_raw, _mag.z_raw)
			//PX4_DEBUG("MPU_MAG: %f %f %f", _mag.x, _mag.y, _mag.z)
		}
	}
}

void task_main(int argc, char *argv[])
{
	PX4_WARN("enter mpu9x50 task_main");

	sigset_t set;
	int      sig = 0;
	int rv;
	exit_mreasurement = false;

	parameters_init();

	// create the mpu9x50 default configuration structure
	struct mpu9x50_config config = {
		.gyro_lpf = _gyro_lpf,
		.acc_lpf  = _accel_lpf,
		.gyro_fsr = MPU9X50_GYRO_FSR_500DPS,
		.acc_fsr  = MPU9X50_ACC_FSR_4G,
		.gyro_sample_rate = _imu_sample_rate,
		.compass_enabled = true,
		.compass_sample_rate = MPU9x50_COMPASS_SAMPLE_RATE_100HZ,
		.spi_dev_path = _device,
	};

	// TODO-JYW:
	// manually register with the DriverFramework to allow this driver to
	// be found by other modules
//	DriverFramework::StubSensor<DriverFramework::SPIDevObj> stub_sensor("mpu9x50", "/dev/accel0", "/dev/accel");

	if (mpu9x50_initialize(&config) != 0) {
		PX4_WARN("error initializing mpu9x50. Quit!");
		goto exit;
	}

	if (mpu9x50_register_interrupt(SPI_INT_GPIO, &mpu9x50::data_ready_isr, NULL)
	    //if (mpu9x50_register_interrupt(SPI_INT_GPIO, &mpu9x50::data_ready_isr, NULL)

	    != 0) {
		PX4_WARN("error registering data ready interrupt. Quit!");
		goto exit;
	}

	// create all uorb publications
	if (!create_pubs()) {
		goto exit;
	}

	// subscribe to parameter_update topic
	_params_sub = orb_subscribe(ORB_ID(parameter_update));

	// initialize signal
	sigemptyset(&set);
	sigaddset(&set, SIGRTMIN);

	while (!_task_should_exit) {
		// wait on signal
		rv = sigwait(&set, &sig);

		// check if we are waken up by the proper signal
		if (rv != 0 || sig != SIGRTMIN) {
			PX4_WARN("mpu9x50 sigwait failed rv %d sig %d", rv, sig);
			continue;
		}

		// read new IMU data and store the results in data
		if (mpu9x50_get_data(&_data) != 0) {
			PX4_WARN("mpu9x50_get_data() failed");
			continue;
		}

		// since the context switch takes long time, override the timestamp provided by mpu9x50_get_data()
		// with the ts of isr.
		// Note: This is ok for MPU sample rate of < 1000; Higer than 1000 Sample rates will be an issue
		// as the data is not consistent.
		_data.timestamp = _isr_data_ready_timestamp;

		// poll parameter update
		parameter_update_poll();

		// data is ready
		update_reports();

		// publish all sensor reports
		publish_reports();

	}

	exit_mreasurement = true;

exit:
	PX4_WARN("closing mpu9x50");
	mpu9x50_close();
}

/** mpu9x50 main entrance */
void task_main_trampoline(int argc, char *argv[])
{
	PX4_WARN("task_main_trampoline");
	task_main(argc, argv);
}

void start()
{
	ASSERT(_task_handle == -1);

	/* start the task */
	_task_handle = px4_task_spawn_cmd("mpu9x50_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX,
					  1500,
					  (px4_main_t)&task_main_trampoline,
					  nullptr);

	if (_task_handle < 0) {
		warn("mpu9x50_main task start failed");
		return;
	}

	_is_running = true;
}

void stop()
{
	// TODO - set thread exit signal to terminate the task main thread

	_is_running = false;
	_task_handle = -1;
}

void usage()
{
	PX4_WARN("missing command: try 'start', 'stop', 'status'");
	PX4_WARN("options:");
	PX4_WARN("    -D device");
}

}; // namespace mpu9x50


int mpu9x50_main(int argc, char *argv[])
{
	const char *device = NULL;
	int ch;
	int myoptind = 1;
	const char *myoptarg = NULL;

	while ((ch = px4_getopt(argc, argv, "D:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'D':
			device = myoptarg;
			break;

		default:
			mpu9x50::usage();
			return 1;
		}
	}

	// Check on required arguments
	if (device == NULL || strlen(device) == 0) {
		mpu9x50::usage();
		return 1;
	}

	memset(mpu9x50::_device, 0, MAX_LEN_DEV_PATH);
	strncpy(mpu9x50::_device, device, strlen(device));

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		if (mpu9x50::_is_running) {
			PX4_WARN("mpu9x50 already running");
			return 1;
		}

		mpu9x50::start();

	} else if (!strcmp(verb, "stop")) {
		if (mpu9x50::_is_running) {
			PX4_WARN("mpu9x50 is not running");
			return 1;
		}

		mpu9x50::stop();

	} else if (!strcmp(verb, "status")) {
		PX4_WARN("mpu9x50 is %s", mpu9x50::_is_running ? "running" : "stopped");
		return 0;

	} else {
		mpu9x50::usage();
		return 1;
	}

	return 0;
}
