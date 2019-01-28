/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "ADIS16477.hpp"
#include "ADIS16477_gyro.hpp"

#include <px4_config.h>
#include <ecl/geo/geo.h>

#define DIR_READ				0x00
#define DIR_WRITE				0x80

//  ADIS16477 registers
static constexpr uint8_t DIAG_STAT = 0x02; // Output, system error flags

static constexpr uint8_t X_GYRO_LOW = 0x04; // Output, x-axis gyroscope, low word
static constexpr uint8_t X_GYRO_OUT = 0x06; // Output, x-axis gyroscope, high word
static constexpr uint8_t Y_GYRO_LOW = 0x08; // Output, y-axis gyroscope, low word
static constexpr uint8_t Y_GYRO_OUT = 0x0A; // Output, y-axis gyroscope, high word
static constexpr uint8_t Z_GYRO_LOW = 0x0C; // Output, z-axis gyroscope, low word
static constexpr uint8_t Z_GYRO_OUT = 0x0E; // Output, z-axis gyroscope, high word

static constexpr uint8_t X_ACCL_LOW = 0x10; // Output, x-axis accelerometer, low word
static constexpr uint8_t X_ACCL_OUT = 0x12; // Output, x-axis accelerometer, high word
static constexpr uint8_t Y_ACCL_LOW = 0x14; // Output, y-axis accelerometer, low word
static constexpr uint8_t Y_ACCL_OUT = 0x16; // Output, y-axis accelerometer, high word
static constexpr uint8_t Z_ACCL_LOW = 0x18; // Output, z-axis accelerometer, low word
static constexpr uint8_t Z_ACCL_OUT = 0x1A; // Output, z-axis accelerometer, high word

static constexpr uint8_t TEMP_OUT	= 0x1A; // Output, temperature
static constexpr uint8_t TIME_STAMP	= 0x1A; // Output, time stamp

static constexpr uint8_t FILT_CTRL	= 0x5C;
static constexpr uint8_t DEC_RATE	= 0x64;

static constexpr uint8_t GLOB_CMD	= 0x68;

static constexpr uint8_t PROD_ID	= 0x72;

static constexpr uint16_t PROD_ID_ADIS16477 = 0x405D; /* ADIS16477 Identification, device number  */

static constexpr int T_STALL = 16;

#define GYROINITIALSENSITIVITY		250
#define ACCELINITIALSENSITIVITY		(1.0f / 1200.0f)
#define ACCELDYNAMICRANGE		18.0f

using namespace time_literals;

ADIS16477::ADIS16477(int bus, const char *path_accel, const char *path_gyro, uint32_t device, enum Rotation rotation) :
	SPI("ADIS16477", path_accel, bus, device, SPIDEV_MODE3, 1000000),
	ScheduledWorkItem(px4::device_bus_to_wq(this->get_device_id())),
	_gyro(new ADIS16477_gyro(this, path_gyro)),
	_sample_perf(perf_alloc(PC_ELAPSED, "adis16477_read")),
	_bad_transfers(perf_alloc(PC_COUNT, "adis16477_bad_transfers")),
	_rotation(rotation)
{
#ifdef GPIO_SPI1_RESET_ADIS16477
	// ADIS16477 configure reset
	px4_arch_configgpio(GPIO_SPI1_RESET_ADIS16477);
#endif /* GPIO_SPI1_RESET_ADIS16477 */

	_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_ADIS16477;

	_gyro->_device_id.devid = _device_id.devid;
	_gyro->_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_ADIS16477;

	// default gyro scale factors
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	// default accel scale factors
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	const unsigned sample_rate = 1000;

	// set software low pass filter for controllers
	param_t accel_cut_ph = param_find("IMU_ACCEL_CUTOFF");
	float accel_cut = ADIS16477_ACCEL_DEFAULT_RATE;

	if (accel_cut_ph != PARAM_INVALID && param_get(accel_cut_ph, &accel_cut) == PX4_OK) {
		_accel_filter_x.set_cutoff_frequency(sample_rate, accel_cut);
		_accel_filter_y.set_cutoff_frequency(sample_rate, accel_cut);
		_accel_filter_z.set_cutoff_frequency(sample_rate, accel_cut);

	} else {
		PX4_ERR("IMU_ACCEL_CUTOFF param invalid");
	}

	param_t gyro_cut_ph = param_find("IMU_GYRO_CUTOFF");
	float gyro_cut = ADIS16477_GYRO_DEFAULT_RATE;

	if (gyro_cut_ph != PARAM_INVALID && param_get(gyro_cut_ph, &gyro_cut) == PX4_OK) {
		_gyro_filter_x.set_cutoff_frequency(sample_rate, gyro_cut);
		_gyro_filter_y.set_cutoff_frequency(sample_rate, gyro_cut);
		_gyro_filter_z.set_cutoff_frequency(sample_rate, gyro_cut);

	} else {
		PX4_ERR("IMU_GYRO_CUTOFF param invalid");
	}
}

ADIS16477::~ADIS16477()
{
	/* make sure we are truly inactive */
	stop();

	/* delete the gyro subdriver */
	delete _gyro;

	if (_accel_class_instance != -1) {
		unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _accel_class_instance);
	}

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_bad_transfers);
}

int
ADIS16477::init()
{
	if (hrt_absolute_time() < 252_ms) {
		// power-on startup time (if needed)
		up_mdelay(252);
	}

	/* do SPI init (and probe) first */
	if (SPI::init() != OK) {
		/* if probe/setup failed, bail now */
		DEVICE_DEBUG("SPI setup failed");
		return PX4_ERROR;
	}

	/* Initialize offsets and scales */
	_gyro_scale.x_offset = 0.0f;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0.0f;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0.0f;
	_gyro_scale.z_scale  = 1.0f;

	_accel_scale.x_offset = 0.0f;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0.0f;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0.0f;
	_accel_scale.z_scale  = 1.0f;

	/* do CDev init for the gyro device node, keep it optional */
	int ret = _gyro->init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("gyro init failed");
		return ret;
	}

	_accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

	/* fetch an initial set of measurements for advertisement */
	measure();

	/* advertise sensor topic, measure manually to initialize valid report */
	sensor_accel_s arp = {};

	/* measurement will have generated a report, publish */
	_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp, &_accel_orb_class_instance, ORB_PRIO_MAX);

	if (_accel_topic == nullptr) {
		PX4_ERR("ADVERT FAIL");
	}

	sensor_gyro_s grp = {};
	_gyro->_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp, &_gyro->_gyro_orb_class_instance, ORB_PRIO_MAX);

	if (_gyro->_gyro_topic == nullptr) {
		PX4_ERR("ADVERT FAIL");
	}

	return ret;
}

int ADIS16477::reset()
{
	DEVICE_DEBUG("resetting");

#ifdef GPIO_SPI1_RESET_ADIS16477
	// ADIS16477 reset
	px4_arch_gpiowrite(GPIO_SPI1_RESET_ADIS16477, 0);

	// The RST line must be in a low state for at least 10 μs to ensure a proper reset initiation and recovery.
	up_udelay(10);

	px4_arch_gpiowrite(GPIO_SPI1_RESET_ADIS16477, 1);
#else
	// reset (global command bit 7)
	uint8_t value[2] = {};
	value[0] = (1 << 7);
	write_reg16(GLOB_CMD, (uint16_t)value[0]);
#endif /* GPIO_SPI1_RESET_ADIS16477 */

	// Reset Recovery Time
	up_mdelay(193);

	// configure digital filter, 16 taps
	write_reg16(FILT_CTRL, 0x04);

	// configure decimation rate
	//write_reg16(DEC_RATE, 0x00);

	return OK;
}

int
ADIS16477::probe()
{
	DEVICE_DEBUG("probe");

	reset();

	// read product id (5 attempts)
	for (int i = 0; i < 5; i++) {
		_product = read_reg16(PROD_ID);

		if (_product == PROD_ID_ADIS16477) {
			DEVICE_DEBUG("PRODUCT: %X", _product);

			if (self_test()) {
				return PX4_OK;

			} else {
				PX4_ERR("probe attempt %d: self test failed, resetting", i);
				reset();
			}

		} else {
			PX4_ERR("probe attempt %d: read product id failed, resetting", i);
			reset();
		}
	}

	return -EIO;
}

/* set sample rate for both accel and gyro */
void
ADIS16477::_set_sample_rate(uint16_t desired_sample_rate_hz)
{

}

/* set the DLPF FIR filter tap. This affects both accelerometer and gyroscope. */
void
ADIS16477::_set_dlpf_filter(uint16_t desired_filter_tap)
{
	//modify_reg16(ADIS16477_SENS_AVG, 0x0007, desired_filter_tap);

	/* Verify data write on the IMU */

	//if ((read_reg16(ADIS16477_SENS_AVG) & 0x0007) != desired_filter_tap) {
	//	DEVICE_DEBUG("failed to set IMU filter");
	//}
}

/* set IMU to factory defaults. */
void
ADIS16477::_set_factory_default()
{
	//write_reg16(ADIS16477_GLOB_CMD, 0x02);
}

/* set the gyroscope dynamic range */
void
ADIS16477::_set_gyro_dyn_range(uint16_t desired_gyro_dyn_range)
{
}

bool
ADIS16477::self_test()
{
	DEVICE_DEBUG("self test");

	// self test (global command bit 2)
	uint8_t value[2] = {};
	value[0] = (1 << 2);
	write_reg16(GLOB_CMD, (uint16_t)value[0]);

	// read DIAG_STAT to check result
	uint16_t diag_stat = read_reg16(DIAG_STAT);

	if (diag_stat != 0) {
		return false;
	}

	return true;
}

int
ADIS16477::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case SENSORIOCRESET:
		return reset();

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, ADIS16477_ACCEL_DEFAULT_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum sane rate */
					if (ticks < 1000) {
						return -EINVAL;
					}

					// adjust filters
					float cutoff_freq_hz = _accel_filter_x.get_cutoff_freq();
					float sample_rate = 1.0e6f / ticks;
					_accel_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
					_accel_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
					_accel_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz);

					float cutoff_freq_hz_gyro = _gyro_filter_x.get_cutoff_freq();
					_gyro_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_call_interval = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case ACCELIOCSSCALE: {
			/* copy scale, but only if off by a few percent */
			struct accel_calibration_s *s = (struct accel_calibration_s *) arg;
			float sum = s->x_scale + s->y_scale + s->z_scale;

			if (sum > 2.0f && sum < 4.0f) {
				memcpy(&_accel_scale, s, sizeof(_accel_scale));
				return OK;

			} else {
				return -EINVAL;
			}
		}

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

int
ADIS16477::gyro_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	/* these are shared with the accel side */
	case SENSORIOCSPOLLRATE:
	case SENSORIOCRESET:
		return ioctl(filp, cmd, arg);

	case GYROIOCSSCALE:
		/* copy scale in */
		memcpy(&_gyro_scale, (struct gyro_calibration_s *) arg, sizeof(_gyro_scale));
		return OK;

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

uint16_t
ADIS16477::read_reg16(uint8_t reg)
{
	uint16_t cmd[1];

	cmd[0] = ((reg | DIR_READ) << 8) & 0xff00;
	transferhword(cmd, nullptr, 1);
	up_udelay(T_STALL);
	transferhword(nullptr, cmd, 1);
	up_udelay(T_STALL);

	return cmd[0];
}

void
ADIS16477::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t cmd[2];
	cmd[0] = reg | 0x8;
	cmd[1] = val;
	transfer(cmd, cmd, sizeof(cmd));
}

void
ADIS16477::write_reg16(uint8_t reg, uint16_t value)
{
	uint16_t cmd[2];

	cmd[0] = ((reg | DIR_WRITE) << 8) | (0x00ff & value);
	cmd[1] = (((reg + 0x1) | DIR_WRITE) << 8) | ((0xff00 & value) >> 8);

	transferhword(cmd, nullptr, 1);
	up_udelay(T_STALL);
	transferhword(cmd + 1, nullptr, 1);
	up_udelay(T_STALL);
}

void
ADIS16477::start()
{
	/* make sure we are stopped first */
	uint32_t last_call_interval = _call_interval;
	stop();
	_call_interval = last_call_interval;

	/* start polling at the specified rate */
	ScheduleOnInterval(_call_interval, 10000);
}

void
ADIS16477::stop()
{
	ScheduleClear();
}

void
ADIS16477::Run()
{
	/* make another measurement */
	measure();
}

int
ADIS16477::measure()
{
	perf_begin(_sample_perf);

	// Fetch the full set of measurements from the ADIS16477 in one pass (burst read).
	ADISReport adis_report;
	adis_report.cmd = ((GLOB_CMD | DIR_READ) << 8) & 0xff00;

	// ADIS16477 burst report should be 176 bits
	static_assert(sizeof(adis_report) == (176 / 8), "ADIS16477 report not 176 bits");

	if (OK != transferhword((uint16_t *)&adis_report, ((uint16_t *)&adis_report), sizeof(adis_report) / sizeof(uint16_t))) {
		perf_end(_sample_perf);
		return -EIO;
	}

	// Calculate checksum and compare

	// Checksum = DIAG_STAT, Bits[15:8] + DIAG_STAT, Bits[7:0] +
	//  X_GYRO_OUT, Bits[15:8] + X_GYRO_OUT, Bits[7:0] +
	//  Y_GYRO_OUT, Bits[15:8] + Y_GYRO_OUT, Bits[7:0] +
	//  Z_GYRO_OUT, Bits[15:8] + Z_GYRO_OUT, Bits[7:0] +
	//  X_ACCL_OUT, Bits[15:8] + X_ACCL_OUT, Bits[7:0] +
	//  Y_ACCL_OUT, Bits[15:8] + Y_ACCL_OUT, Bits[7:0] +
	//  Z_ACCL_OUT, Bits[15:8] + Z_ACCL_OUT, Bits[7:0] +
	//  TEMP_OUT, Bits[15:8] + TEMP_OUT, Bits[7:0] +
	//  DATA_CNTR, Bits[15:8] + DATA_CNTR, Bits[7:0]
	uint8_t *checksum_helper = (uint8_t *)&adis_report.diag_stat;

	uint8_t checksum = 0;

	for (int i = 0; i < 18; i++) {
		checksum += checksum_helper[i];
	}

	if (adis_report.checksum != checksum) {
		DEVICE_DEBUG("adis_report.checksum: %X vs calculated: %X", adis_report.checksum, checksum);
		perf_event_count(_bad_transfers);
		perf_end(_sample_perf);
		return -EIO;
	}

	// Check all Status/Error Flag Indicators (DIAG_STAT)
	if (adis_report.diag_stat != 0) {
		perf_event_count(_bad_transfers);
		perf_end(_sample_perf);
		return -EIO;
	}

	publish_gyro(adis_report);
	publish_accel(adis_report);

	/* stop measuring */
	perf_end(_sample_perf);
	return OK;
}

bool
ADIS16477::publish_accel(const ADISReport &report)
{
	sensor_accel_s arb = {};
	arb.timestamp = hrt_absolute_time();
	arb.device_id = _device_id.devid;
	arb.error_count = perf_event_count(_bad_transfers);

	// raw sensor readings
	arb.x_raw = report.accel_x;
	arb.y_raw = report.accel_y;
	arb.z_raw = report.accel_z;
	arb.scaling = _accel_range_scale;

	float xraw_f = report.accel_x * _accel_range_scale;
	float yraw_f = report.accel_y * _accel_range_scale;
	float zraw_f = report.accel_z * _accel_range_scale;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_in_new = (xraw_f - _accel_scale.x_offset) * _accel_scale.x_scale;
	float y_in_new = (yraw_f - _accel_scale.y_offset) * _accel_scale.y_scale;
	float z_in_new = (zraw_f - _accel_scale.z_offset) * _accel_scale.z_scale;

	arb.x = _accel_filter_x.apply(x_in_new);
	arb.y = _accel_filter_y.apply(y_in_new);
	arb.z = _accel_filter_z.apply(z_in_new);

	matrix::Vector3f aval(x_in_new, y_in_new, z_in_new);
	matrix::Vector3f aval_integrated;

	bool accel_notify = _accel_int.put(arb.timestamp, aval, aval_integrated, arb.integral_dt);
	arb.x_integral = aval_integrated(0);
	arb.y_integral = aval_integrated(1);
	arb.z_integral = aval_integrated(2);

	/* Temperature report: */
	// temperature 1 LSB = 0.1°C
	arb.temperature = report.temp * 0.1;

	if (accel_notify) {
		orb_publish(ORB_ID(sensor_accel), _accel_topic, &arb);
	}

	return accel_notify;
}

bool
ADIS16477::publish_gyro(const ADISReport &report)
{
	sensor_gyro_s grb = {};
	grb.timestamp = hrt_absolute_time();
	grb.device_id = _gyro->_device_id.devid;
	grb.error_count = perf_event_count(_bad_transfers);

	/* Gyro report: */
	grb.scaling = math::radians(_gyro_range_scale);
	grb.x_raw = report.gyro_x;
	grb.y_raw = report.gyro_y;
	grb.z_raw = report.gyro_z;

	// ADIS16477-2BMLZ scale factory
	float xraw_f = report.gyro_x;
	float yraw_f = report.gyro_y;
	float zraw_f = report.gyro_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_gyro_in_new = (math::radians(xraw_f * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
	float y_gyro_in_new = (math::radians(yraw_f * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	float z_gyro_in_new = (math::radians(zraw_f * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;

	grb.x = _gyro_filter_x.apply(x_gyro_in_new);
	grb.y = _gyro_filter_y.apply(y_gyro_in_new);
	grb.z = _gyro_filter_z.apply(z_gyro_in_new);

	matrix::Vector3f gval(x_gyro_in_new, y_gyro_in_new, z_gyro_in_new);
	matrix::Vector3f gval_integrated;

	bool gyro_notify = _gyro_int.put(grb.timestamp, gval, gval_integrated, grb.integral_dt);
	grb.x_integral = gval_integrated(0);
	grb.y_integral = gval_integrated(1);
	grb.z_integral = gval_integrated(2);

	/* Temperature report: */
	// temperature 1 LSB = 0.1°C
	grb.temperature = report.temp * 0.1f;

	if (gyro_notify) {
		orb_publish(ORB_ID(sensor_gyro), _gyro->_gyro_topic, &grb);
	}

	return gyro_notify;
}

void
ADIS16477::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfers);

	PX4_INFO("DEVICE ID:\nACCEL:\t%d\nGYRO:\t%d\n\n", _device_id.devid, _gyro->_device_id.devid);
}
