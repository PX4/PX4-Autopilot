/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file ADIS16448.cpp
 */

#include "ADIS16448.h"

ADIS16448::ADIS16448(int bus, const char *path_accel, const char *path_gyro, const char *path_mag, uint32_t device,
		     enum Rotation rotation) :
	SPI("ADIS16448", path_accel, bus, device, SPIDEV_MODE3, SPI_BUS_SPEED),
	_accel(new ADIS16448_accel(this, path_accel)),
	_gyro(new ADIS16448_gyro(this, path_gyro)),
	_mag(new ADIS16448_mag(this, path_mag)),
	_accel_int(1000000 / ADIS16448_ACCEL_MAX_OUTPUT_RATE, false),
	_gyro_int(1000000 / ADIS16448_GYRO_MAX_OUTPUT_RATE, true)
{
	// Disable debug() calls.
	_debug_enabled = false;

	_accel->_device_id.devid = _device_id.devid;
	_accel->_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_ADIS16448;

	_gyro->_device_id.devid = _device_id.devid;
	_gyro->_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_ADIS16448;

	_mag->_device_id.devid = _device_id.devid;
	_mag->_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_ADIS16448;

	memset(&_call, 0, sizeof(_call));
}

ADIS16448::~ADIS16448()
{
	// Ensure the driver is inactive.
	stop();

	// Delete subdriver instances.
	delete _accel;
	delete _gyro;
	delete _mag;

	if (_accel_reports != nullptr) {
		delete _accel_reports;
	}

	// Free any existing reports.
	if (_gyro_reports != nullptr) {
		delete _gyro_reports;
	}

	if (_mag_reports != nullptr) {
		delete _mag_reports;
	}

	// Delete the perf counter.
	perf_free(_sample_perf);
	perf_free(_accel_reads);
	perf_free(_gyro_reads);
	perf_free(_mag_reads);
	perf_free(_bad_transfers);
}

int
ADIS16448::init()
{
	// Do SPI init (and probe) first.
	int ret = 0;
	ret = SPI::init();

	// If probe/setup failed, return result.
	if (ret != OK) {
		DEVICE_DEBUG("SPI setup failed");
		return ret;
	}

	// Allocate basic report buffers.
	_accel_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_accel_s));
	_gyro_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_gyro_s));
	_mag_reports = new ringbuffer::RingBuffer(2, sizeof(mag_report));

	if (_accel_reports == nullptr ||
	    _gyro_reports == nullptr ||
	    _mag_reports == nullptr) {
		DEVICE_DEBUG("ring buffer allocation failed");
		return EFAULT;
	}

	ret = reset();

	if (ret != OK) {
		return ret;
	}

	// Initialize offset and scale values.
	_accel_cal.x_offset = 0;
	_accel_cal.y_offset = 0;
	_accel_cal.z_offset = 0;
	_accel_cal.x_scale = 1.0f;
	_accel_cal.y_scale = 1.0f;
	_accel_cal.z_scale = 1.0f;

	_gyro_cal.x_offset = 0;
	_gyro_cal.y_offset = 0;
	_gyro_cal.z_offset = 0;
	_gyro_cal.x_scale = 1.0f;
	_gyro_cal.y_scale = 1.0f;
	_gyro_cal.z_scale = 1.0f;

	_mag_cal.x_offset = 0;
	_mag_cal.y_offset = 0;
	_mag_cal.z_offset = 0;
	_mag_cal.x_scale = 1.0f;
	_mag_cal.y_scale = 1.0f;
	_mag_cal.z_scale = 1.0f;


	// CDev init for the gyro and magnetometer device nodes, keep it optional.
	// If probe/setup failed return result.
	ret = _accel->init();

	if (ret != OK) {
		DEVICE_DEBUG("accel init failed");
		return ret;
	}

	ret = _gyro->init();

	if (ret != OK) {
		DEVICE_DEBUG("gyro init failed");
		return ret;
	}

	ret = _mag->init();

	if (ret != OK) {
		DEVICE_DEBUG("mag init failed");
		return ret;
	}

	// Obtain an initial set of measurements for advertisement.
	measure();

	// Advertise sensor topic, measure manually to initialize valid report.
	// Measurement will have generated a report, publish it now.
	sensor_accel_s arp;
	_accel_reports->get(&arp);
	_accel->_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
			       &_accel->_accel_orb_class_instance, ORB_PRIO_MAX);

	if (_accel->_accel_topic == nullptr) {
		PX4_WARN("ACCEL ADVERT FAIL");
	}

	sensor_gyro_s grp;
	_gyro_reports->get(&grp);
	_gyro->_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
			     &_gyro->_gyro_orb_class_instance, ORB_PRIO_MAX);

	if (_gyro->_gyro_topic == nullptr) {
		PX4_WARN("GYRO ADVERT FAIL");
	}

	struct mag_report mrp;

	_mag_reports->get(&mrp);

	_mag->_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &mrp,
					       &_mag->_mag_orb_class_instance, ORB_PRIO_MAX);

	if (_mag->_mag_topic == nullptr) {
		PX4_WARN("MAG ADVERT FAIL");
	}

	return OK;
}

int ADIS16448::reset()
{
	// Set digital FIR filter tap.
	_set_dlpf_filter(BITS_FIR_16_TAP_CFG);

	// Set IMU sample rate.
	_set_sample_rate(_sample_rate);

	_accel_range_scale = ACCEL_INITIAL_SENSITIVITY * CONSTANTS_ONE_G;
	_accel_range_m_s2  = ACCEL_DYNAMIC_RANGE * CONSTANTS_ONE_G;

	// Set gyroscope scale to default value.
	_set_gyro_dyn_range(GYRO_INITIAL_SENSITIVITY);

	// _gyro_range_scale  = GYRO_INITIAL_SENSITIVITY * M_DEG_TO_RAD_F;
	// _gyro_range_rad_s  = GYRO_DYNAMIC_RANGE * M_DEG_TO_RAD_F;

	_mag_range_scale   = MAG_INITIAL_SENSITIVITY;
	_mag_range_mgauss  = MAG_DYNAMIC_RANGE;

	// Settling time.
	up_udelay(50000);

	return OK;
}

int
ADIS16448::probe()
{
	uint16_t serial_number;

	// Retry 5 time to get the ADIS16448 PRODUCT ID number.
	for (size_t i = 0; i < 5; i++) {
		// Read product ID.
		_product_ID = read_reg16(ADIS16448_PRODUCT_ID);

		if (_product_ID != 0) {
			break;
		}
	}

	// Recognize product serial number.
	serial_number = (read_reg16(ADIS16334_SERIAL_NUMBER) & 0xfff);

	// Verify product ID.
	switch (_product_ID) {
	case ADIS16448_Product:
		DEVICE_DEBUG("ADIS16448 is detected ID: 0x%02x, Serial: 0x%02x", _product_ID, serial_number);
		modify_reg16(ADIS16448_GPIO_CTRL, 0x0200, 0x0002);  // Turn on ADIS16448 adaptor board led.
		return OK;
	}

	DEVICE_DEBUG("unexpected ID 0x%02x", _product_ID);
	return -EIO;
}

void
ADIS16448::_set_sample_rate(uint16_t desired_sample_rate_hz)
{
	uint16_t smpl_prd = 0;

	if (desired_sample_rate_hz <= 51) {
		smpl_prd = BITS_SMPL_PRD_16_TAP_CFG;

	} else if (desired_sample_rate_hz <= 102) {
		smpl_prd = BITS_SMPL_PRD_8_TAP_CFG;

	} else if (desired_sample_rate_hz <= 204) {
		smpl_prd = BITS_SMPL_PRD_4_TAP_CFG;

	} else if (desired_sample_rate_hz <= 409) {
		smpl_prd = BITS_SMPL_PRD_2_TAP_CFG;

	} else {
		smpl_prd = BITS_SMPL_PRD_NO_TAP_CFG;
	}

	modify_reg16(ADIS16448_SMPL_PRD, 0x1f00, smpl_prd);

	if ((read_reg16(ADIS16448_SMPL_PRD) & 0x1f00) != smpl_prd) {
		DEVICE_DEBUG("failed to set IMU sample rate");
	}
}

void
ADIS16448::_set_dlpf_filter(uint16_t desired_filter_tap)
{
	// Set the DLPF FIR filter tap. This affects both accelerometer and gyroscope.
	modify_reg16(ADIS16448_SENS_AVG, 0x0007, desired_filter_tap);

	// Verify data write on the IMU.
	if ((read_reg16(ADIS16448_SENS_AVG) & 0x0007) != desired_filter_tap) {
		DEVICE_DEBUG("failed to set IMU filter");
	}
}

void
ADIS16448::_set_factory_defaults()
{
	write_reg16(ADIS16448_GLOB_CMD, 0x02);
}

void
ADIS16448::_set_gyro_dyn_range(uint16_t desired_gyro_dyn_range)
{
	uint16_t gyro_range_selection = 0;

	if (desired_gyro_dyn_range <= 250) {
		gyro_range_selection = BITS_GYRO_DYN_RANGE_250_CFG;

	} else if (desired_gyro_dyn_range <= 500) {
		gyro_range_selection = BITS_GYRO_DYN_RANGE_500_CFG;

	} else {
		gyro_range_selection = BITS_GYRO_DYN_RANGE_1000_CFG;
	}

	modify_reg16(ADIS16448_SENS_AVG, 0x0700, gyro_range_selection);

	// Verify data write on the IMU.
	if ((read_reg16(ADIS16448_SENS_AVG) & 0x0700) != gyro_range_selection) {
		DEVICE_DEBUG("failed to set gyro range");

	} else {
		_gyro_range_scale  = (float)(gyro_range_selection >> 8) / 100.0f;
		_gyro_range_rad_s  = (float)(gyro_range_selection >> 8) * 250.0f * M_DEG_TO_RAD_F;
	}
}

int
ADIS16448::self_test()
{
	// Return 0 on success, otherwise 1.
	if (perf_event_count(_sample_perf) == 0) {
		measure();
		return OK;

	} else {
		return 1;
	}
}

ssize_t
ADIS16448::accel_read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned int count = buflen / sizeof(sensor_accel_s);

	// Buffer must be large enough.
	if (count < 1) {
		return -ENOSPC;
	}

	// If automatic measurement is not enabled, get a fresh measurement into the buffer.
	if (_call_interval == 0) {
		_accel_reports->flush();
		measure();
	}

	// If no data, error (we could block here).
	if (_accel_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_accel_reads);

	// Copy reports out of our buffer to the caller.
	sensor_accel_s *arp = reinterpret_cast<sensor_accel_s *>(buffer);

	ssize_t transferred = 0;

	while (count--) {
		if (!_accel_reports->get(arp)) {
			break;

		} else {
			transferred++;
			arp++;
		}
	}

	// Return the number of bytes transferred.
	return (transferred * sizeof(sensor_accel_s));
}

ssize_t
ADIS16448::gyro_read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_gyro_s);

	// Buffer must be large enough.
	if (count < 1) {
		return -ENOSPC;
	}

	// If automatic measurement is not enabled, get a fresh measurement into the buffer.
	if (_call_interval == 0) {
		_gyro_reports->flush();
		measure();
	}

	// If no data, error (we could block here).
	if (_gyro_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_gyro_reads);

	// Copy reports out of our buffer to the caller.
	sensor_gyro_s *grp = reinterpret_cast<sensor_gyro_s *>(buffer);
	ssize_t transferred = 0;

	while (count--) {
		if (!_gyro_reports->get(grp)) {
			break;

		} else {
			transferred++;
			grp++;
		}
	}

	// Return the number of bytes transferred.
	return (transferred * sizeof(sensor_gyro_s));
}

ssize_t
ADIS16448::mag_read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(mag_report);

	// Buffer must be large enough.
	if (count < 1) {
		return -ENOSPC;
	}

	// If automatic measurement is not enabled, get a fresh measurement into the buffer.
	if (_call_interval == 0) {
		_mag_reports->flush();
		measure();
	}

	// If no data, error (we could block here).
	if (_mag_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_mag_reads);

	// Copy reports out of our buffer to the caller.
	mag_report *mrp = reinterpret_cast<mag_report *>(buffer);
	ssize_t transferred = 0;

	while (count--) {
		if (!_mag_reports->get(mrp)) {
			break;

		} else {
			transferred++;
			mrp++;
		}
	}

	// Return the number of bytes transferred.
	return (transferred * sizeof(sensor_mag_s));
}

int
ADIS16448::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case DEVIOCGDEVICEID:
		return (int)CDev::ioctl(filp, cmd, arg);

	case ACCELIOCSSCALE:
		return accel_ioctl(filp, cmd, arg);

	case GYROIOCSSCALE:
		return gyro_ioctl(filp, cmd, arg);

	case MAGIOCSSCALE:
	case MAGIOCGSCALE:
		return mag_ioctl(filp, cmd, arg);

	case SENSORIOCRESET:
		return reset();

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			// Zero would be bad.
			case 0:
				return -EINVAL;

			// Set default polling rate.
			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, ADIS16448_ACCEL_DEFAULT_RATE);

			// Adjust to a legal polling interval in Hz.
			default: {
					// Convert Hz to hrt interval via microseconds.
					unsigned int ticks = 1.0e6f / arg;

					// Check against maximum sane rate.
					if (ticks < 1000) {
						return -EINVAL;
					}

					float sample_rate = 1.0e6f / ticks;

					// Adjust filters.
					float cutoff_freq_hz_accel = _accel_filter_x.get_cutoff_freq();
					_accel_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_accel);
					_accel_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_accel);
					_accel_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_accel);


					float cutoff_freq_hz_gyro = _gyro_filter_x.get_cutoff_freq();
					_gyro_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);


					float cutoff_freq_hz_mag = _mag_filter_x.get_cutoff_freq();
					_mag_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_mag);
					_mag_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_mag);
					_mag_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_mag);

					// Update interval for next measurement.
					// XXX this is a bit shady, but no other way to adjust...
					_call.period = _call_interval = ticks;

					// Start internal state machine polling if needed.
					if (_call_interval == 0) {
						start();
					}

					return OK;
				}
			}
		}

	default:
		// Give it to the superclass.
		return SPI::ioctl(filp, cmd, arg);
	}
}

int
ADIS16448::accel_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	// Copy scale, but only if off by a few percent.
	struct accel_calibration_s *s = (struct accel_calibration_s *) arg;
	float sum = s->x_scale + s->y_scale + s->z_scale;

	if (sum > 2.0f && sum < 4.0f) {
		// Copy scale in.
		memcpy(&_accel_cal, (struct accel_calibration_s *) arg, sizeof(_accel_cal));
		return OK;

	} else {
		return -EINVAL;
	}
}

int
ADIS16448::gyro_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	// Copy scale in.
	memcpy(&_gyro_cal, (struct gyro_calibration_s *) arg, sizeof(_gyro_cal));
	return OK;
}

int
ADIS16448::mag_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case MAGIOCSSCALE:
		// Copy scale in.
		memcpy(&_mag_cal, (struct mag_calibration_s *) arg, sizeof(_mag_cal));
		return OK;

	case MAGIOCGSCALE:
		// Copy scale out.
		memcpy((struct mag_calibration_s *) arg, &_mag_cal, sizeof(_mag_cal));
		return OK;

	default:
		return OK;
	}
}

void
ADIS16448::print_calibration_data()
{
	uint16_t XACCL_OFF = read_reg16(ADIS16448_XACCL_OFF);
	uint16_t YACCL_OFF = read_reg16(ADIS16448_YACCL_OFF);
	uint16_t ZACCL_OFF = read_reg16(ADIS16448_ZACCL_OFF);

	uint16_t XGYRO_OFF = read_reg16(ADIS16448_XGYRO_OFF);
	uint16_t YGYRO_OFF = read_reg16(ADIS16448_YGYRO_OFF);
	uint16_t ZGYRO_OFF = read_reg16(ADIS16448_ZGYRO_OFF);

	uint16_t XMAGN_HIC = read_reg16(ADIS16448_XMAGN_HIC);
	uint16_t YMAGN_HIC = read_reg16(ADIS16448_YMAGN_HIC);
	uint16_t ZMAGN_HIC = read_reg16(ADIS16448_ZMAGN_HIC);

	uint16_t XMAGN_SIC = read_reg16(ADIS16448_XMAGN_SIC);
	uint16_t YMAGN_SIC = read_reg16(ADIS16448_YMAGN_SIC);
	uint16_t ZMAGN_SIC = read_reg16(ADIS16448_ZMAGN_SIC);

	PX4_INFO("single calibration value read:");
	PX4_INFO("XACCL_OFF =:  \t%8.4x\t", XACCL_OFF);
	PX4_INFO("YACCL_OFF =:  \t%8.4x\t", YACCL_OFF);
	PX4_INFO("ZACCL_OFF =:  \t%8.4x\t", ZACCL_OFF);

	PX4_INFO("XGYRO_OFF =:  \t%8.4x\t", XGYRO_OFF);
	PX4_INFO("YGYRO_OFF =:  \t%8.4x\t", YGYRO_OFF);
	PX4_INFO("ZGYRO_OFF =:  \t%8.4x\t", ZGYRO_OFF);

	PX4_INFO("XMAGN_HIC =:  \t%8.4x\t", XMAGN_HIC);
	PX4_INFO("YMAGN_HIC =:  \t%8.4x\t", YMAGN_HIC);
	PX4_INFO("ZMAGN_HIC =:  \t%8.4x\t", ZMAGN_HIC);

	PX4_INFO("XMAGN_SIC =:  \t%8.4x\t", XMAGN_SIC);
	PX4_INFO("YMAGN_SIC =:  \t%8.4x\t", YMAGN_SIC);
	PX4_INFO("ZMAGN_SIC =:  \t%8.4x\t", ZMAGN_SIC);
}

void
ADIS16448::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_accel_reads);
	perf_print_counter(_gyro_reads);
	perf_print_counter(_mag_reads);
	perf_print_counter(_bad_transfers);

	_accel_reports->print_info("accel queue");
	_gyro_reports->print_info("gyro queue");
	_mag_reports->print_info("mag queue");

	printf("DEVICE ID:\nACCEL:\t%d\nGYRO:\t%d\nMAG:\t%d\n", _accel->_device_id.devid,
	       _gyro->_device_id.devid, _mag->_device_id.devid);
}

void
ADIS16448::modify_reg16(unsigned reg, uint16_t clearbits, uint16_t setbits)
{
	uint16_t val;

	val = read_reg16(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg16(reg, val);
}

uint16_t
ADIS16448::read_reg16(unsigned reg)
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
ADIS16448::write_reg16(unsigned reg, uint16_t value)
{
	uint16_t cmd[2];

	cmd[0] = ((reg | DIR_WRITE) << 8) | (0x00ff & value);
	cmd[1] = (((reg + 0x1) | DIR_WRITE) << 8) | ((0xff00 & value) >> 8);

	transferhword(cmd, nullptr, 1);
	up_udelay(T_STALL);
	transferhword(cmd + 1, nullptr, 1);
	up_udelay(T_STALL);
}

int16_t
ADIS16448::convert_12bit_to_int16(int16_t word)
{
	int16_t outputbuffer = 0;

	if ((word >> 11) & 0x1) {
		outputbuffer = (word & 0xfff) | 0xf000;

	} else {
		outputbuffer = (word & 0x0fff);
	}

	return (outputbuffer);
}

void
ADIS16448::start()
{
	// Make sure we are stopped first.
	uint32_t last_call_interval = _call_interval;
	stop();
	_call_interval = last_call_interval;

	// Discard any stale data in the buffers.
	_accel_reports->flush();
	_gyro_reports->flush();
	_mag_reports->flush();

	// Start polling at the specified rate.
	hrt_call_every(&_call, 1000, _call_interval, (hrt_callout)&ADIS16448::measure_trampoline, this);
}

void
ADIS16448::stop()
{
	hrt_cancel(&_call);
}

int
ADIS16448::measure()
{
	struct ADISReport adis_report;

	struct Report {
		int16_t accel_x;
		int16_t accel_y;
		int16_t accel_z;
		int16_t gyro_x;
		int16_t gyro_y;
		int16_t gyro_z;
		int16_t mag_x;
		int16_t mag_y;
		int16_t mag_z;
		int16_t baro;
		int16_t temp;
	} report;

	// Start measuring.
	perf_begin(_sample_perf);

	//Fetch the full set of measurements from the ADIS16448 in one pass (burst read).
	adis_report.cmd = ((ADIS16448_GLOB_CMD | DIR_READ) << 8) & 0xff00;

	if (OK != transferhword((uint16_t *)&adis_report, ((uint16_t *)&adis_report), sizeof(adis_report) / sizeof(uint16_t))) {
		return -EIO;
	}

	report.accel_x = (int16_t) adis_report.accel_x;
	report.accel_y = (int16_t) adis_report.accel_y;
	report.accel_z = (int16_t) adis_report.accel_z;

	report.gyro_x  = (int16_t) adis_report.gyro_x;
	report.gyro_y  = (int16_t) adis_report.gyro_y;
	report.gyro_z  = (int16_t) adis_report.gyro_z;

	report.mag_x   = (int16_t) adis_report.mag_x;
	report.mag_y   = (int16_t) adis_report.mag_y;
	report.mag_z   = (int16_t) adis_report.mag_z;

	report.baro    = (int16_t) adis_report.baro;
	report.temp    = convert_12bit_to_int16(adis_report.temp);

	if (report.accel_x == 0 && report.accel_y == 0 && report.accel_z == 0 &&
	    report.gyro_x == 0 && report.gyro_y == 0 && report.gyro_z == 0 &&
	    report.mag_x == 0 && report.mag_y == 0 && report.mag_z == 0 &&
	    report.baro == 0 && report.temp == 0) {
		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		return -EIO;
	}

	//Report buffers.
	sensor_accel_s  arb;
	sensor_gyro_s   grb;
	sensor_mag_s    mrb;

	// Set timestamp values for accel, gyro, and mag.
	arb.timestamp = grb.timestamp = mrb.timestamp = hrt_absolute_time();
	arb.error_count = grb.error_count = mrb.error_count = perf_event_count(_bad_transfers);

	// Accel report:
	arb.x_raw = report.accel_x;
	arb.y_raw = report.accel_y;
	arb.z_raw = report.accel_z;

	float xraw_f = report.accel_x;
	float yraw_f = report.accel_y;
	float zraw_f = report.accel_z;

	// Apply user specified rotation.
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_accel_in_new = ((xraw_f * _accel_range_scale) - _accel_cal.x_offset) * _accel_cal.x_scale;
	float y_accel_in_new = ((yraw_f * _accel_range_scale) - _accel_cal.y_offset) * _accel_cal.y_scale;
	float z_accel_in_new = ((zraw_f * _accel_range_scale) - _accel_cal.z_offset) * _accel_cal.z_scale;

	if (FW_FILTER) {
		arb.x = _accel_filter_x.apply(x_accel_in_new);
		arb.y = _accel_filter_y.apply(y_accel_in_new);
		arb.z = _accel_filter_z.apply(z_accel_in_new);

	} else {
		arb.x = x_accel_in_new;
		arb.y = y_accel_in_new;
		arb.z = z_accel_in_new;
	}

	arb.scaling = _accel_range_scale;

	// Gyro report:
	grb.x_raw = report.gyro_x;
	grb.y_raw = report.gyro_y;
	grb.z_raw = report.gyro_z;

	xraw_f = report.gyro_x;
	yraw_f = report.gyro_y;
	zraw_f = report.gyro_z;

	// Apply user specified rotation.
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_gyro_in_new = ((xraw_f * _gyro_range_scale) * M_DEG_TO_RAD_F - _gyro_cal.x_offset) * _gyro_cal.x_scale;
	float y_gyro_in_new = ((yraw_f * _gyro_range_scale) * M_DEG_TO_RAD_F - _gyro_cal.y_offset) * _gyro_cal.y_scale;
	float z_gyro_in_new = ((zraw_f * _gyro_range_scale) * M_DEG_TO_RAD_F - _gyro_cal.z_offset) * _gyro_cal.z_scale;

	if (FW_FILTER) {
		grb.x = _gyro_filter_x.apply(x_gyro_in_new);
		grb.y = _gyro_filter_y.apply(y_gyro_in_new);
		grb.z = _gyro_filter_z.apply(z_gyro_in_new);

	} else {
		grb.x = x_gyro_in_new;
		grb.y = y_gyro_in_new;
		grb.z = z_gyro_in_new;
	}

	grb.scaling = _gyro_range_scale * M_DEG_TO_RAD_F;

	// Mag report:
	mrb.x_raw = report.mag_x;
	mrb.y_raw = report.mag_y;
	mrb.z_raw = report.mag_z;

	xraw_f = report.mag_x;
	yraw_f = report.mag_y;
	zraw_f = report.mag_z;

	// Apply user specified rotation.
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_mag_new = ((xraw_f * _mag_range_scale) - _mag_cal.x_offset) * _mag_cal.x_scale;
	float y_mag_new = ((yraw_f * _mag_range_scale) - _mag_cal.y_offset) * _mag_cal.y_scale;
	float z_mag_new = ((zraw_f * _mag_range_scale) - _mag_cal.z_offset) * _mag_cal.z_scale;

	if (FW_FILTER) {
		mrb.x = _mag_filter_x.apply(x_mag_new) / 1000.0f;
		mrb.y = _mag_filter_y.apply(y_mag_new) / 1000.0f;
		mrb.z = _mag_filter_z.apply(z_mag_new) / 1000.0f;

	} else {
		mrb.x = x_mag_new / 1000.0f;
		mrb.y = y_mag_new / 1000.0f;
		mrb.z = z_mag_new / 1000.0f;
	}

	mrb.scaling  = _mag_range_scale / 1000.0f;  // Convert to milligauss.

	// Temperature report:
	arb.temperature = (report.temp * 0.07386f) + 31.0f;
	grb.temperature = arb.temperature;
	mrb.temperature = arb.temperature;

	matrix::Vector3f aval(x_accel_in_new, y_accel_in_new, z_accel_in_new);
	matrix::Vector3f aval_integrated;

	bool accel_notify = _accel_int.put(arb.timestamp, aval, aval_integrated, arb.integral_dt);
	arb.x_integral = aval_integrated(0);
	arb.y_integral = aval_integrated(1);
	arb.z_integral = aval_integrated(2);

	matrix::Vector3f gval(x_gyro_in_new, y_gyro_in_new, z_gyro_in_new);
	matrix::Vector3f gval_integrated;

	bool gyro_notify = _gyro_int.put(grb.timestamp, gval, gval_integrated, grb.integral_dt);
	grb.x_integral = gval_integrated(0);
	grb.y_integral = gval_integrated(1);
	grb.z_integral = gval_integrated(2);

	// Return device ID.
	arb.device_id = _accel->_device_id.devid;
	grb.device_id = _gyro->_device_id.devid;
	mrb.device_id = _mag->_device_id.devid;

	_accel_reports->force(&arb);
	_gyro_reports ->force(&grb);
	_mag_reports  ->force(&mrb);

	// Notify anyone waiting for data.
	if (accel_notify) {
		_accel->parent_poll_notify();

		if (!(_pub_blocked)) {
			// Publish it.
			orb_publish(ORB_ID(sensor_accel), _accel->_accel_topic, &arb);
		}
	}

	if (gyro_notify) {
		_gyro->parent_poll_notify();

		if (!(_pub_blocked)) {
			// Publish it.
			orb_publish(ORB_ID(sensor_gyro), _gyro->_gyro_topic, &grb);
		}
	}

	_mag->parent_poll_notify();

	if (!(_pub_blocked) && ((adis_report.status >> 7) & 0x1)) { // Mag data validity bit (bit 8 DIAG_STAT).
		// Publish it.
		orb_publish(ORB_ID(sensor_mag), _mag->_mag_topic, &mrb);
	}

	// Stop measuring.
	perf_end(_sample_perf);
	return OK;
}

void
ADIS16448::measure_trampoline(void *arg)
{
	ADIS16448 *dev = reinterpret_cast<ADIS16448 *>(arg);

	// Make another measurement.
	dev->measure();
}


/**
 * Local functions in support of the shell command.
 */
namespace adis16448
{

ADIS16448 *g_dev;

void info();
void info_cal();
void reset();
void start(enum Rotation rotation);
void test();
void usage();

/**
 * Start the driver.
 */
void
start(enum Rotation rotation)
{
	int fd = NULL;

	if (g_dev != nullptr) {
		// If already started, the still command succeeded.
		PX4_INFO("already started");
	}

	// Create the driver.
#if defined(PX4_SPI_BUS_EXT)
	g_dev = new ADIS16448(PX4_SPI_BUS_EXT, ADIS16448_DEVICE_PATH_ACCEL, ADIS16448_DEVICE_PATH_GYRO,
			      ADIS16448_DEVICE_PATH_MAG, PX4_SPIDEV_EXT_MPU, rotation);
#else
	PX4_ERR("External SPI not available");
#endif

	if (g_dev != nullptr) {
		if (g_dev->init() == OK) {
			// Set the poll rate to default, starts automatic data collection.
			fd = open(ADIS16448_DEVICE_PATH_ACCEL, O_RDONLY);

			if (fd >= 0) {

				if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) >= 0) {
					close(fd);
					exit(0);
				}
			}
		}

		delete g_dev;
		g_dev = nullptr;
	}

	PX4_ERR("driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	sensor_accel_s a_report{};
	sensor_gyro_s  g_report{};
	mag_report     m_report;

	// Open the accel driver file descriptor.
	int fd_accel = open(ADIS16448_DEVICE_PATH_ACCEL, O_RDONLY);

	if (fd_accel < 0) {
		err(1, "%s open failed", ADIS16448_DEVICE_PATH_ACCEL);
	}

	// Open the gyro driver file descriptor.
	int fd_gyro = open(ADIS16448_DEVICE_PATH_GYRO, O_RDONLY);

	if (fd_gyro < 0) {
		err(1, "%s open failed", ADIS16448_DEVICE_PATH_GYRO);
	}

	// Open the mag driver file descriptor.
	int fd_mag = open(ADIS16448_DEVICE_PATH_MAG, O_RDONLY);

	if (fd_mag < 0) {
		err(1, "%s open failed", ADIS16448_DEVICE_PATH_MAG);
	}

	// Do a simple demand read.
	int sz = read(fd_accel, &a_report, sizeof(a_report));

	if (sz != sizeof(a_report)) {
		PX4_INFO("ret: %d, expected: %d", sz, sizeof(a_report));
		PX4_ERR("Self Test: Immediate accel read failed");
	}

	print_message(a_report);

	// Do a simple demand read.
	sz = read(fd_gyro, &g_report, sizeof(g_report));

	if (sz != sizeof(g_report)) {
		PX4_INFO("ret: %d, expected: %d", sz, sizeof(g_report));
		PX4_ERR("Self Test: Immediate gyro read failed");
	}

	print_message(g_report);

	// Do a simple demand read.
	sz = read(fd_mag, &m_report, sizeof(m_report));

	if (sz != sizeof(m_report)) {
		PX4_INFO("ret: %d, expected: %d", sz, sizeof(m_report));
		PX4_ERR("Self Test: Immediate mag read failed");
	}

	print_message(m_report);

	// XXX add poll-rate tests here too.
	close(fd_accel);
	close(fd_gyro);
	close(fd_mag);

	reset();
	PX4_INFO("Self Test: PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(ADIS16448_DEVICE_PATH_ACCEL, O_RDONLY);

	if (fd < 0) {
		err(1, "open failed");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	close(fd);

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		PX4_INFO("driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

/**
 * Print a sensor calibration info.
 */
void
info_cal()
{
	if (g_dev == nullptr) {
		PX4_INFO("driver not running");
	}

	g_dev->print_calibration_data();

	exit(0);
}

void
usage()
{
	PX4_INFO("missing command: try 'start', 'test', 'info', 'info_cal', 'reset',\n");
	PX4_INFO("options:");
	PX4_INFO("    -R rotation");
}

} // namespace


/**
 * Driver 'main' command.
 */
extern "C" int adis16448_main(int argc, char *argv[])
{
	enum Rotation rotation = ROTATION_NONE;
	int ch;

	/* start options */
	while ((ch = getopt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;

		default:
			adis16448::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	// Start/load the driver.
	if (!strcmp(verb, "start")) {
		adis16448::start(rotation);
	}

	// Test the driver/device.
	if (!strcmp(verb, "test")) {
		adis16448::test();
	}

	// Reset the driver.
	if (!strcmp(verb, "reset")) {
		adis16448::reset();
	}

	// Print driver information.
	if (!strcmp(verb, "info")) {
		adis16448::info();
	}

	// Print sensor calibration information.
	if (!strcmp(verb, "info_cal")) {
		adis16448::info_cal();
	}

	adis16448::usage();
	exit(1);
}
