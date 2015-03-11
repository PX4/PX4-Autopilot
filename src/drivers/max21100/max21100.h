/*
 * max21100.h
 *
 *  Created on: 25 Jan 2015
 *      Author: matt
 */

#ifndef MAX21100_H_
#define MAX21100_H_

typedef struct max21100_reg_t{
	unsigned int bank;
	unsigned int address;
} max21100_reg;

class MAX21100_gyro;

class MAX21100 : public device::SPI
{
public:
	MAX21100(int bus, const char *path_accel, const char *path_gyro, spi_dev_e device, enum Rotation rotation);
	virtual ~MAX21100();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

	void			print_registers();

	/**
	 * Test behaviour against factory offsets
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			factory_self_test();

	// deliberately cause a sensor error
	void 			test_error();

protected:
	virtual int		probe();

	friend class MAX21100_gyro;

	virtual ssize_t		gyro_read(struct file *filp, char *buffer, size_t buflen);
	virtual int		gyro_ioctl(struct file *filp, int cmd, unsigned long arg);

private:
	MAX21100_gyro		*_gyro;
	uint8_t			_product;	/** product code */

	struct hrt_call		_call;
	unsigned		_call_interval;

	RingBuffer		*_accel_reports;

	struct accel_scale	_accel_scale;
	float			_accel_range_scale;
	float			_accel_range_m_s2;
	orb_advert_t		_accel_topic;
	orb_id_t		_accel_orb_id;
	int			_accel_class_instance;

	RingBuffer		*_gyro_reports;

	struct gyro_scale	_gyro_scale;
	float			_gyro_range_scale;
	float			_gyro_range_rad_s;

	unsigned		_sample_rate;
	perf_counter_t		_accel_reads;
	perf_counter_t		_gyro_reads;
	perf_counter_t		_sample_perf;
	perf_counter_t		_bad_transfers;
	perf_counter_t		_bad_registers;
	perf_counter_t		_good_transfers;
	perf_counter_t		_reset_retries;
	perf_counter_t		_system_latency_perf;
	perf_counter_t		_controller_latency_perf;

	uint8_t			_register_wait;
	uint64_t		_reset_wait;

	math::LowPassFilter2p	_accel_filter_x;
	math::LowPassFilter2p	_accel_filter_y;
	math::LowPassFilter2p	_accel_filter_z;
	math::LowPassFilter2p	_gyro_filter_x;
	math::LowPassFilter2p	_gyro_filter_y;
	math::LowPassFilter2p	_gyro_filter_z;

	enum Rotation		_rotation;

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
#define MAX21100_NUM_CHECKED_REGISTERS 9
	static const uint8_t	_checked_registers[MAX21100_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_values[MAX21100_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_next;

	// use this to avoid processing measurements when in factory
	// self test
	volatile bool		_in_factory_test;

	/**
	 * Start automatic measurement.
	 */
	void			start();

	/**
	 * Stop automatic measurement.
	 */
	void			stop();

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	int			reset();

	/**
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 *
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		measure_trampoline(void *arg);

	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	void			measure();

	/**
	 * Read a register
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(max21100_reg reg);
	uint16_t		read_reg16(max21100_reg reg);

	/**
	 * Write a register
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(max21100_reg reg, uint8_t value);

	/**
	 * Modify a register
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(max21100_reg reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_checked_reg(max21100_reg reg, uint8_t value);

	/**
	 * Set the measurement range.
	 *
	 * @param max_g		The maximum G value the range must support.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_range(unsigned max_g);

	/**
	 * Swap a 16-bit value read from the MAX21100 to native byte order.
	 */
	uint16_t		swap16(uint16_t val) { return (val >> 8) | (val << 8);	}

	/**
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	 int 			self_test();

	/**
	 * Accel self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			accel_self_test();

	/**
	 * Gyro self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	 int 			gyro_self_test();

	/*
	  set low pass filter frequency
	 */
	void _set_dlpf_filter(uint16_t frequency_hz);

	/*
	  set sample rate (approximate) - 1kHz to 5Hz
	*/
	void _set_sample_rate(uint16_t desired_sample_rate_hz);

	/*
	  check that key registers still have the right value
	 */
	void check_registers(void);

	/* do not allow to copy this class due to pointer data members */
	MAX21100(const MAX21100&);
	MAX21100 operator=(const MAX21100&);

#pragma pack(push, 1)
	/**
	 * Report conversation within the MAX21100, including command byte and
	 * interrupt status.
	 */
	struct MPUReport {
		uint8_t		cmd;
		uint8_t		status;
		uint8_t		accel_x[2];
		uint8_t		accel_y[2];
		uint8_t		accel_z[2];
		uint8_t		temp[2];
		uint8_t		gyro_x[2];
		uint8_t		gyro_y[2];
		uint8_t		gyro_z[2];
	};
#pragma pack(pop)
};


/**
 * Helper class implementing the gyro driver node.
 */
class MAX21100_gyro : public device::CDev
{
public:
	MAX21100_gyro(MAX21100 *parent, const char *path);
	~MAX21100_gyro();

	virtual ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual int		init();

protected:
	friend class MAX21100;

	void			parent_poll_notify();

private:
	MAX21100			*_parent;
	orb_advert_t		_gyro_topic;
	orb_id_t		_gyro_orb_id;
	int			_gyro_class_instance;

	/* do not allow to copy this class due to pointer data members */
	MAX21100_gyro(const MAX21100_gyro&);
	MAX21100_gyro operator=(const MAX21100_gyro&);
};



#endif /* MAX21100_H_ */
