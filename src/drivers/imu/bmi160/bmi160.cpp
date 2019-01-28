#include "bmi160.hpp"
#include "bmi160_gyro.hpp"
#include <ecl/geo/geo.h>

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */
const uint8_t BMI160::_checked_registers[BMI160_NUM_CHECKED_REGISTERS] = {    BMIREG_CHIP_ID,
									      BMIREG_ACC_CONF,
									      BMIREG_ACC_RANGE,
									      BMIREG_GYR_CONF,
									      BMIREG_GYR_RANGE,
									      BMIREG_INT_EN_1,
									      BMIREG_INT_OUT_CTRL,
									      BMIREG_INT_MAP_1,
									      BMIREG_IF_CONF,
									      BMIREG_NV_CONF
									 };

BMI160::BMI160(int bus, const char *path_accel, const char *path_gyro, uint32_t device, enum Rotation rotation) :
	SPI("BMI160", path_accel, bus, device, SPIDEV_MODE3, BMI160_BUS_SPEED),
	ScheduledWorkItem(px4::device_bus_to_wq(this->get_device_id())),
	_gyro(new BMI160_gyro(this, path_gyro)),
	_whoami(0),
	_call_interval(0),
	_accel_reports(nullptr),
	_accel_scale{},
	_accel_range_scale(0.0f),
	_accel_range_m_s2(0.0f),
	_accel_topic(nullptr),
	_accel_orb_class_instance(-1),
	_accel_class_instance(-1),
	_gyro_reports(nullptr),
	_gyro_scale{},
	_gyro_range_scale(0.0f),
	_gyro_range_rad_s(0.0f),
	_dlpf_freq(0),
	_accel_sample_rate(BMI160_ACCEL_DEFAULT_RATE),
	_gyro_sample_rate(BMI160_GYRO_DEFAULT_RATE),
	_accel_reads(perf_alloc(PC_COUNT, "bmi160_accel_read")),
	_gyro_reads(perf_alloc(PC_COUNT, "bmi160_gyro_read")),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmi160_read")),
	_bad_transfers(perf_alloc(PC_COUNT, "bmi160_bad_transfers")),
	_bad_registers(perf_alloc(PC_COUNT, "bmi160_bad_registers")),
	_good_transfers(perf_alloc(PC_COUNT, "bmi160_good_transfers")),
	_reset_retries(perf_alloc(PC_COUNT, "bmi160_reset_retries")),
	_duplicates(perf_alloc(PC_COUNT, "bmi160_duplicates")),
	_register_wait(0),
	_reset_wait(0),
	_accel_filter_x(BMI160_ACCEL_DEFAULT_RATE, BMI160_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_y(BMI160_ACCEL_DEFAULT_RATE, BMI160_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_z(BMI160_ACCEL_DEFAULT_RATE, BMI160_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_x(BMI160_GYRO_DEFAULT_RATE, BMI160_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_y(BMI160_GYRO_DEFAULT_RATE, BMI160_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_z(BMI160_GYRO_DEFAULT_RATE, BMI160_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_int(1000000 / BMI160_ACCEL_MAX_PUBLISH_RATE),
	_gyro_int(1000000 / BMI160_GYRO_MAX_PUBLISH_RATE, true),
	_rotation(rotation),
	_checked_next(0),
	_last_temperature(0),
	_last_accel{},
	_got_duplicate(false)
{
	// disable debug() calls
	_debug_enabled = false;

	_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_BMI160;

	/* Prime _gyro with parents devid. */
	_gyro->_device_id.devid = _device_id.devid;
	_gyro->_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_BMI160;

	// default accel scale factors
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	// default gyro scale factors
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;
}


BMI160::~BMI160()
{
	/* make sure we are truly inactive */
	stop();

	/* delete the gyro subdriver */
	delete _gyro;

	/* free any existing reports */
	if (_accel_reports != nullptr) {
		delete _accel_reports;
	}

	if (_gyro_reports != nullptr) {
		delete _gyro_reports;
	}

	if (_accel_class_instance != -1) {
		unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _accel_class_instance);
	}

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_accel_reads);
	perf_free(_gyro_reads);
	perf_free(_bad_transfers);
	perf_free(_bad_registers);
	perf_free(_good_transfers);
	perf_free(_reset_retries);
	perf_free(_duplicates);
}

int
BMI160::init()
{
	int ret;

	/* do SPI init (and probe) first */
	ret = SPI::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("SPI setup failed");
		return ret;
	}

	/* allocate basic report buffers */
	_accel_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_accel_s));

	if (_accel_reports == nullptr) {
		goto out;
	}

	_gyro_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_gyro_s));

	if (_gyro_reports == nullptr) {
		goto out;
	}

	if (reset() != OK) {
		goto out;
	}

	/* Initialize offsets and scales */
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;


	/* do CDev init for the gyro device node, keep it optional */
	ret = _gyro->init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("gyro init failed");
		return ret;
	}

	_accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

	measure();

	/* advertise sensor topic, measure manually to initialize valid report */
	sensor_accel_s arp;
	_accel_reports->get(&arp);

	/* measurement will have generated a report, publish */
	_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
					   &_accel_orb_class_instance, (external()) ? ORB_PRIO_MAX - 1 : ORB_PRIO_HIGH - 1);

	if (_accel_topic == nullptr) {
		warnx("ADVERT FAIL");
	}


	/* advertise sensor topic, measure manually to initialize valid report */
	sensor_gyro_s grp;
	_gyro_reports->get(&grp);

	_gyro->_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
			     &_gyro->_gyro_orb_class_instance, (external()) ? ORB_PRIO_MAX - 1 : ORB_PRIO_HIGH - 1);

	if (_gyro->_gyro_topic == nullptr) {
		warnx("ADVERT FAIL");
	}

out:
	return ret;
}


int BMI160::reset()
{
	write_reg(BMIREG_CONF, (1 << 1)); //Enable NVM programming

	write_checked_reg(BMIREG_ACC_CONF,      BMI_ACCEL_US | BMI_ACCEL_BWP_NORMAL); //Normal operation, no decimation
	write_checked_reg(BMIREG_ACC_RANGE,     0);
	write_checked_reg(BMIREG_GYR_CONF,      BMI_GYRO_BWP_NORMAL);   //Normal operation, no decimation
	write_checked_reg(BMIREG_GYR_RANGE,     0);
	write_checked_reg(BMIREG_INT_EN_1,      BMI_DRDY_INT_EN); //Enable DRDY interrupt
	write_checked_reg(BMIREG_INT_OUT_CTRL,  BMI_INT1_EN);   //Enable interrupts on pin INT1
	write_checked_reg(BMIREG_INT_MAP_1,     BMI_DRDY_INT1); //DRDY interrupt on pin INT1
	write_checked_reg(BMIREG_IF_CONF,       BMI_SPI_4_WIRE |
			  BMI_AUTO_DIS_SEC); //Disable secondary interface; Work in SPI 4-wire mode
	write_checked_reg(BMIREG_NV_CONF,       BMI_SPI); //Disable I2C interface

	set_accel_range(BMI160_ACCEL_DEFAULT_RANGE_G);
	accel_set_sample_rate(BMI160_ACCEL_DEFAULT_RATE);

	set_gyro_range(BMI160_GYRO_DEFAULT_RANGE_DPS);
	gyro_set_sample_rate(BMI160_GYRO_DEFAULT_RATE);


	//_set_dlpf_filter(BMI160_ACCEL_DEFAULT_ONCHIP_FILTER_FREQ); //NOT CONSIDERING FILTERING YET

	//Enable Accelerometer in normal mode
	write_reg(BMIREG_CMD, BMI_ACCEL_NORMAL_MODE);
	up_udelay(4100);
	//usleep(4100);

	//Enable Gyroscope in normal mode
	write_reg(BMIREG_CMD, BMI_GYRO_NORMAL_MODE);
	up_udelay(80300);
	//usleep(80300);

	uint8_t retries = 10;

	while (retries--) {
		bool all_ok = true;

		for (uint8_t i = 0; i < BMI160_NUM_CHECKED_REGISTERS; i++) {
			if (read_reg(_checked_registers[i]) != _checked_values[i]) {
				write_reg(_checked_registers[i], _checked_values[i]);
				all_ok = false;
			}
		}

		if (all_ok) {
			break;
		}
	}

	_accel_reads = 0;
	_gyro_reads = 0;

	return OK;
}

int
BMI160::probe()
{
	/* look for device ID */
	_whoami = read_reg(BMIREG_CHIP_ID);

	// verify product revision
	switch (_whoami) {
	case BMI160_WHO_AM_I:
		memset(_checked_values, 0, sizeof(_checked_values));
		memset(_checked_bad, 0, sizeof(_checked_bad));
		_checked_values[0] = _whoami;
		_checked_bad[0] = _whoami;
		return OK;
	}

	DEVICE_DEBUG("unexpected whoami 0x%02x", _whoami);
	return -EIO;
}

int
BMI160::accel_set_sample_rate(float frequency)
{
	uint8_t setbits = 0;
	uint8_t clearbits = (BMI_ACCEL_RATE_25_8 | BMI_ACCEL_RATE_1600);

	if ((int)frequency == 0) {
		frequency = 1600;
	}

	if (frequency <= 25 / 32) {
		setbits |= BMI_ACCEL_RATE_25_32;
		_accel_sample_rate = 25 / 32;

	} else if (frequency <= 25 / 16) {
		setbits |= BMI_ACCEL_RATE_25_16;
		_accel_sample_rate = 25 / 16;

	} else if (frequency <= 25 / 8) {
		setbits |= BMI_ACCEL_RATE_25_8;
		_accel_sample_rate = 25 / 8;

	} else if (frequency <= 25 / 4) {
		setbits |= BMI_ACCEL_RATE_25_4;
		_accel_sample_rate = 25 / 4;

	} else if (frequency <= 25 / 2) {
		setbits |= BMI_ACCEL_RATE_25_2;
		_accel_sample_rate = 25 / 2;

	} else if (frequency <= 25) {
		setbits |= BMI_ACCEL_RATE_25;
		_accel_sample_rate = 25;

	} else if (frequency <= 50) {
		setbits |= BMI_ACCEL_RATE_50;
		_accel_sample_rate = 50;

	} else if (frequency <= 100) {
		setbits |= BMI_ACCEL_RATE_100;
		_accel_sample_rate = 100;

	} else if (frequency <= 200) {
		setbits |= BMI_ACCEL_RATE_200;
		_accel_sample_rate = 200;

	} else if (frequency <= 400) {
		setbits |= BMI_ACCEL_RATE_400;
		_accel_sample_rate = 400;

	} else if (frequency <= 800) {
		setbits |= BMI_ACCEL_RATE_800;
		_accel_sample_rate = 800;

	} else if (frequency > 800) {
		setbits |= BMI_ACCEL_RATE_1600;
		_accel_sample_rate = 1600;

	} else {
		return -EINVAL;
	}

	modify_reg(BMIREG_ACC_CONF, clearbits, setbits);

	return OK;
}

int
BMI160::gyro_set_sample_rate(float frequency)
{
	uint8_t setbits = 0;
	uint8_t clearbits = (BMI_GYRO_RATE_200 | BMI_GYRO_RATE_25);

	if ((int)frequency == 0) {
		frequency = 3200;
	}

	if (frequency <= 25) {
		setbits |= BMI_GYRO_RATE_25;
		_gyro_sample_rate = 25;

	} else if (frequency <= 50) {
		setbits |= BMI_GYRO_RATE_50;
		_gyro_sample_rate = 50;

	} else if (frequency <= 100) {
		setbits |= BMI_GYRO_RATE_100;
		_gyro_sample_rate = 100;

	} else if (frequency <= 200) {
		setbits |= BMI_GYRO_RATE_200;
		_gyro_sample_rate = 200;

	} else if (frequency <= 400) {
		setbits |= BMI_GYRO_RATE_400;
		_gyro_sample_rate = 400;

	} else if (frequency <= 800) {
		setbits |= BMI_GYRO_RATE_800;
		_gyro_sample_rate = 800;

	} else if (frequency <= 1600) {
		setbits |= BMI_GYRO_RATE_1600;
		_gyro_sample_rate = 1600;

	} else if (frequency > 1600) {
		setbits |= BMI_GYRO_RATE_3200;
		_gyro_sample_rate = 3200;

	} else {
		return -EINVAL;
	}

	modify_reg(BMIREG_GYR_CONF, clearbits, setbits);

	return OK;
}

void
BMI160::_set_dlpf_filter(uint16_t bandwidth)
{
	_dlpf_freq = 0;
	bandwidth = bandwidth;   //TO BE IMPLEMENTED
	/*uint8_t setbits = BW_SCAL_ODR_BW_XL;
	uint8_t clearbits = BW_XL_50_HZ;

	if (bandwidth == 0) {
	    _dlpf_freq = 408;
	    clearbits = BW_SCAL_ODR_BW_XL | BW_XL_50_HZ;
	    setbits = 0;
	}

	if (bandwidth <= 50) {
	    setbits |= BW_XL_50_HZ;
	    _dlpf_freq = 50;

	} else if (bandwidth <= 105) {
	    setbits |= BW_XL_105_HZ;
	    _dlpf_freq = 105;

	} else if (bandwidth <= 211) {
	    setbits |= BW_XL_211_HZ;
	    _dlpf_freq = 211;

	} else if (bandwidth <= 408) {
	    setbits |= BW_XL_408_HZ;
	    _dlpf_freq = 408;

	}
	modify_reg(CTRL_REG6_XL, clearbits, setbits);*/
}

ssize_t
BMI160::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_accel_s);

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_call_interval == 0) {
		_accel_reports->flush();
		measure();
	}

	/* if no data, error (we could block here) */
	if (_accel_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_accel_reads);

	/* copy reports out of our buffer to the caller */
	sensor_accel_s *arp = reinterpret_cast<sensor_accel_s *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_accel_reports->get(arp)) {
			break;
		}

		transferred++;
		arp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(sensor_accel_s));
}

int
BMI160::self_test()
{
	if (perf_event_count(_sample_perf) == 0) {
		measure();
	}

	/* return 0 on success, 1 else */
	return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

/*
  deliberately trigger an error in the sensor to trigger recovery
 */
void
BMI160::test_error()
{
	write_reg(BMIREG_CMD, BMI160_SOFT_RESET);
	::printf("error triggered\n");
	print_registers();
}

ssize_t
BMI160::gyro_read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(sensor_gyro_s);

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_call_interval == 0) {
		_gyro_reports->flush();
		measure();
	}

	/* if no data, error (we could block here) */
	if (_gyro_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_gyro_reads);

	/* copy reports out of our buffer to the caller */
	sensor_gyro_s *grp = reinterpret_cast<sensor_gyro_s *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_gyro_reports->get(grp)) {
			break;
		}

		transferred++;
		grp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(sensor_gyro_s));
}


int
BMI160::ioctl(struct file *filp, int cmd, unsigned long arg)
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
				if (BMI160_GYRO_DEFAULT_RATE > BMI160_ACCEL_DEFAULT_RATE) {
					return ioctl(filp, SENSORIOCSPOLLRATE, BMI160_GYRO_DEFAULT_RATE);

				} else {
					return ioctl(filp, SENSORIOCSPOLLRATE,
						     BMI160_ACCEL_DEFAULT_RATE); //Polling at the highest frequency. We may get duplicate values on the sensors
				}

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
BMI160::gyro_ioctl(struct file *filp, int cmd, unsigned long arg)
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

uint8_t
BMI160::read_reg(unsigned reg)
{
	uint8_t cmd[2] = { (uint8_t)(reg | DIR_READ), 0};

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

uint16_t
BMI160::read_reg16(unsigned reg)
{
	uint8_t cmd[3] = { (uint8_t)(reg | DIR_READ), 0, 0 };

	transfer(cmd, cmd, sizeof(cmd));

	return (uint16_t)(cmd[1] << 8) | cmd[2];
}

void
BMI160::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	transfer(cmd, nullptr, sizeof(cmd));
}

void
BMI160::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

void
BMI160::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < BMI160_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
			_checked_bad[i] = value;
		}
	}
}

int
BMI160::set_accel_range(unsigned max_g)
{
	uint8_t setbits = 0;
	uint8_t clearbits = BMI_ACCEL_RANGE_2_G | BMI_ACCEL_RANGE_16_G;
	float lsb_per_g;
	float max_accel_g;

	if (max_g == 0) {
		max_g = 16;
	}

	if (max_g <= 2) {
		max_accel_g = 2;
		setbits |= BMI_ACCEL_RANGE_2_G;
		lsb_per_g = 16384;

	} else if (max_g <= 4) {
		max_accel_g = 4;
		setbits |= BMI_ACCEL_RANGE_4_G;
		lsb_per_g = 8192;

	} else if (max_g <= 8) {
		max_accel_g = 8;
		setbits |= BMI_ACCEL_RANGE_8_G;
		lsb_per_g = 4096;

	} else if (max_g <= 16) {
		max_accel_g = 16;
		setbits |= BMI_ACCEL_RANGE_16_G;
		lsb_per_g = 2048;

	} else {
		return -EINVAL;
	}

	_accel_range_scale = (CONSTANTS_ONE_G / lsb_per_g);
	_accel_range_m_s2 = max_accel_g * CONSTANTS_ONE_G;

	modify_reg(BMIREG_ACC_RANGE, clearbits, setbits);

	return OK;
}

int
BMI160::set_gyro_range(unsigned max_dps)
{
	uint8_t setbits = 0;
	uint8_t clearbits = BMI_GYRO_RANGE_125_DPS | BMI_GYRO_RANGE_250_DPS;
	float lsb_per_dps;
	float max_gyro_dps;

	if (max_dps == 0) {
		max_dps = 2000;
	}

	if (max_dps <= 125) {
		max_gyro_dps = 125;
		lsb_per_dps = 262.4;
		setbits |= BMI_GYRO_RANGE_125_DPS;

	} else if (max_dps <= 250) {
		max_gyro_dps = 250;
		lsb_per_dps = 131.2;
		setbits |= BMI_GYRO_RANGE_250_DPS;

	} else if (max_dps <= 500) {
		max_gyro_dps = 500;
		lsb_per_dps = 65.6;
		setbits |= BMI_GYRO_RANGE_500_DPS;

	} else if (max_dps <= 1000) {
		max_gyro_dps = 1000;
		lsb_per_dps = 32.8;
		setbits |= BMI_GYRO_RANGE_1000_DPS;

	} else if (max_dps <= 2000) {
		max_gyro_dps = 2000;
		lsb_per_dps = 16.4;
		setbits |= BMI_GYRO_RANGE_2000_DPS;

	} else {
		return -EINVAL;
	}

	_gyro_range_rad_s = (max_gyro_dps / 180.0f * M_PI_F);
	_gyro_range_scale = (M_PI_F / (180.0f * lsb_per_dps));

	modify_reg(BMIREG_GYR_RANGE, clearbits, setbits);

	return OK;
}

void
BMI160::start()
{
	/* make sure we are stopped first */
	stop();

	/* discard any stale data in the buffers */
	_accel_reports->flush();
	_gyro_reports->flush();

	/* start polling at the specified rate */
	ScheduleOnInterval(_call_interval - BMI160_TIMER_REDUCTION, 10000);

	reset();
}

void
BMI160::stop()
{
	ScheduleClear();
}

void
BMI160::Run()
{
	/* make another measurement */
	measure();
}

void
BMI160::check_registers(void)
{
	uint8_t v;

	if ((v = read_reg(_checked_registers[_checked_next])) !=
	    _checked_values[_checked_next]) {
		_checked_bad[_checked_next] = v;

		/*
		  if we get the wrong value then we know the SPI bus
		  or sensor is very sick. We set _register_wait to 20
		  and wait until we have seen 20 good values in a row
		  before we consider the sensor to be OK again.
		 */
		perf_count(_bad_registers);

		/*
		  try to fix the bad register value. We only try to
		  fix one per loop to prevent a bad sensor hogging the
		  bus.
		 */
		if (_register_wait == 0 || _checked_next == 0) {
			// if the product_id is wrong then reset the
			// sensor completely
			write_reg(BMIREG_CMD, BMI160_SOFT_RESET);
			_reset_wait = hrt_absolute_time() + 10000;
			_checked_next = 0;

		} else {
			write_reg(_checked_registers[_checked_next], _checked_values[_checked_next]);
			// waiting 3ms between register writes seems
			// to raise the chance of the sensor
			// recovering considerably
			_reset_wait = hrt_absolute_time() + 3000;
		}

		_register_wait = 20;
	}

	_checked_next = (_checked_next + 1) % BMI160_NUM_CHECKED_REGISTERS;
}

void
BMI160::measure()
{
	if (hrt_absolute_time() < _reset_wait) {
		// we're waiting for a reset to complete
		return;
	}

	struct BMIReport bmi_report;

	struct Report {
		int16_t		accel_x;
		int16_t		accel_y;
		int16_t		accel_z;
		int16_t		temp;
		int16_t		gyro_x;
		int16_t		gyro_y;
		int16_t		gyro_z;
	} report;

	/* start measuring */
	perf_begin(_sample_perf);

	/*
	 * Fetch the full set of measurements from the BMI160 in one pass.
	 */
	bmi_report.cmd = BMIREG_GYR_X_L | DIR_READ;

	uint8_t		status = read_reg(BMIREG_STATUS);

	if (OK != transfer((uint8_t *)&bmi_report, ((uint8_t *)&bmi_report), sizeof(bmi_report))) {
		return;
	}

	check_registers();

	if ((!(status & (0x80))) && (!(status & (0x04)))) {
		perf_end(_sample_perf);
		perf_count(_duplicates);
		_got_duplicate = true;
		return;
	}

	_last_accel[0] = bmi_report.accel_x;
	_last_accel[1] = bmi_report.accel_y;
	_last_accel[2] = bmi_report.accel_z;
	_got_duplicate = false;

	uint8_t temp_l = read_reg(BMIREG_TEMP_0);
	uint8_t temp_h = read_reg(BMIREG_TEMP_1);

	report.temp = ((temp_h << 8) + temp_l);

	report.accel_x = bmi_report.accel_x;
	report.accel_y = bmi_report.accel_y;
	report.accel_z = bmi_report.accel_z;

	report.gyro_x = bmi_report.gyro_x;
	report.gyro_y = bmi_report.gyro_y;
	report.gyro_z = bmi_report.gyro_z;

	if (report.accel_x == 0 &&
	    report.accel_y == 0 &&
	    report.accel_z == 0 &&
	    report.temp == 0 &&
	    report.gyro_x == 0 &&
	    report.gyro_y == 0 &&
	    report.gyro_z == 0) {
		// all zero data - probably a SPI bus error
		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		// note that we don't call reset() here as a reset()
		// costs 20ms with interrupts disabled. That means if
		// the bmi160 does go bad it would cause a FMU failure,
		// regardless of whether another sensor is available,
		return;
	}

	perf_count(_good_transfers);

	if (_register_wait != 0) {
		// we are waiting for some good transfers before using
		// the sensor again. We still increment
		// _good_transfers, but don't return any data yet
		_register_wait--;
		return;
	}

	/*
	 * Report buffers.
	 */
	sensor_accel_s arb{};
	sensor_gyro_s grb{};

	/*
	 * Adjust and scale results to m/s^2.
	 */
	grb.timestamp = arb.timestamp = hrt_absolute_time();

	// report the error count as the sum of the number of bad
	// transfers and bad register reads. This allows the higher
	// level code to decide if it should use this sensor based on
	// whether it has had failures
	grb.error_count = arb.error_count = perf_event_count(_bad_transfers) + perf_event_count(_bad_registers);

	/*
	 * 1) Scale raw value to SI units using scaling from datasheet.
	 * 2) Subtract static offset (in SI units)
	 * 3) Scale the statically calibrated values with a linear
	 *    dynamically obtained factor
	 *
	 * Note: the static sensor offset is the number the sensor outputs
	 * 	 at a nominally 'zero' input. Therefore the offset has to
	 * 	 be subtracted.
	 *
	 *	 Example: A gyro outputs a value of 74 at zero angular rate
	 *	 	  the offset is 74 from the origin and subtracting
	 *		  74 from all measurements centers them around zero.
	 */


	/* NOTE: Axes have been swapped to match the board a few lines above. */

	arb.x_raw = report.accel_x;
	arb.y_raw = report.accel_y;
	arb.z_raw = report.accel_z;

	float xraw_f = report.accel_x;
	float yraw_f = report.accel_y;
	float zraw_f = report.accel_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_in_new = ((xraw_f * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
	float y_in_new = ((yraw_f * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
	float z_in_new = ((zraw_f * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;

	arb.x = _accel_filter_x.apply(x_in_new);
	arb.y = _accel_filter_y.apply(y_in_new);
	arb.z = _accel_filter_z.apply(z_in_new);

	matrix::Vector3f aval(x_in_new, y_in_new, z_in_new);
	matrix::Vector3f aval_integrated;

	bool accel_notify = _accel_int.put(arb.timestamp, aval, aval_integrated, arb.integral_dt);
	arb.x_integral = aval_integrated(0);
	arb.y_integral = aval_integrated(1);
	arb.z_integral = aval_integrated(2);

	arb.scaling = _accel_range_scale;

	_last_temperature = 23 + report.temp * 1.0f / 512.0f;

	arb.temperature = _last_temperature;

	/* return device ID */
	arb.device_id = _device_id.devid;

	grb.x_raw = report.gyro_x;
	grb.y_raw = report.gyro_y;
	grb.z_raw = report.gyro_z;

	xraw_f = report.gyro_x;
	yraw_f = report.gyro_y;
	zraw_f = report.gyro_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_gyro_in_new = ((xraw_f * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
	float y_gyro_in_new = ((yraw_f * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	float z_gyro_in_new = ((zraw_f * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;

	grb.x = _gyro_filter_x.apply(x_gyro_in_new);
	grb.y = _gyro_filter_y.apply(y_gyro_in_new);
	grb.z = _gyro_filter_z.apply(z_gyro_in_new);

	matrix::Vector3f gval(x_gyro_in_new, y_gyro_in_new, z_gyro_in_new);
	matrix::Vector3f gval_integrated;

	bool gyro_notify = _gyro_int.put(arb.timestamp, gval, gval_integrated, grb.integral_dt);
	grb.x_integral = gval_integrated(0);
	grb.y_integral = gval_integrated(1);
	grb.z_integral = gval_integrated(2);

	grb.scaling = _gyro_range_scale;

	grb.temperature = _last_temperature;

	/* return device ID */
	grb.device_id = _gyro->_device_id.devid;

	_accel_reports->force(&arb);
	_gyro_reports->force(&grb);

	/* notify anyone waiting for data */
	if (accel_notify) {
		poll_notify(POLLIN);
	}

	if (gyro_notify) {
		_gyro->parent_poll_notify();
	}

	if (accel_notify && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_accel), _accel_topic, &arb);
	}

	if (gyro_notify && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_gyro), _gyro->_gyro_topic, &grb);
	}

	/* stop measuring */
	perf_end(_sample_perf);
}

void
BMI160::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_accel_reads);
	perf_print_counter(_gyro_reads);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_bad_registers);
	perf_print_counter(_good_transfers);
	perf_print_counter(_reset_retries);
	perf_print_counter(_duplicates);
	_accel_reports->print_info("accel queue");
	_gyro_reports->print_info("gyro queue");
	::printf("checked_next: %u\n", _checked_next);

	for (uint8_t i = 0; i < BMI160_NUM_CHECKED_REGISTERS; i++) {
		uint8_t v = read_reg(_checked_registers[i]);

		if (v != _checked_values[i]) {
			::printf("reg %02x:%02x should be %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_values[i]);
		}

		if (v != _checked_bad[i]) {
			::printf("reg %02x:%02x was bad %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_bad[i]);
		}
	}

	::printf("temperature: %.1f\n", (double)_last_temperature);
}

void
BMI160::print_registers()
{
	printf("BMI160 registers\n");

	for (uint8_t reg = 0x40; reg <= 0x47; reg++) {
		uint8_t v = read_reg(reg);
		printf("%02x:%02x ", (unsigned)reg, (unsigned)v);

		if (reg % 13 == 0) {
			printf("\n");
		}
	}

	printf("\n");
}
