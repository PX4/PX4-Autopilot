
/*
  the MPU9250 can only handle high SPI bus speeds on the sensor and
  interrupt status registers. All other registers have a maximum 1MHz
  SPI speed
 */
#define MPU9250_LOW_BUS_SPEED				1000*1000
#define MPU9250_HIGH_BUS_SPEED				11*1000*1000

class MPU9250_mag;

class MPU9250 : public device::SPI
{
public:
	MPU9250(int bus, const char *path_accel, const char *path_gyro, const char *path_mag, spi_dev_e device);
	virtual ~MPU9250();

	virtual int		init();

	void			set_frequency_high();
	void			print_registers();
	bool			measure();

	void			start(int rate);
	void			stop();
	int				reset();

protected:
	virtual int		probe();

	friend class MPU9250_mag;

private:
	MPU9250_mag     *_mag;
	uint8_t			_whoami;	/** whoami result */

	struct hrt_call		_call;

	bool check_duplicate(uint8_t *accel_data);

	// keep last accel reading for duplicate detection
	uint8_t			_last_accel_data[6];
	bool			_got_duplicate;

	static void		measure_trampoline(void *arg);

	uint8_t			read_reg(unsigned reg, uint32_t speed = MPU9250_LOW_BUS_SPEED);
	uint16_t		read_reg16(unsigned reg);
	void			write_reg(unsigned reg, uint8_t value);
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	uint16_t		swap16(uint16_t val) { return (val >> 8) | (val << 8);	}

	void _set_sample_rate(unsigned desired_sample_rate_hz);

	/* do not allow to copy this class due to pointer data members */
	MPU9250(const MPU9250 &);
	MPU9250 operator=(const MPU9250 &);

#pragma pack(push, 1)
	/**
	 * Report conversation within the mpu, including command byte and
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
		struct ak8963_regs mag;
	};
#pragma pack(pop)
};

struct Report {
	int16_t		accel_x;
	int16_t		accel_y;
	int16_t		accel_z;
	int16_t		temp;
	int16_t		gyro_x;
	int16_t		gyro_y;
	int16_t		gyro_z;
	int16_t		mag_x;
	int16_t		mag_y;
	int16_t		mag_z;
};
