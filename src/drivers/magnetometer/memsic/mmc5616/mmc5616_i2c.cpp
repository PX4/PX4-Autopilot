#include "mmc5616.hpp"
#include <drivers/device/i2c.h>

class MMC5616_I2C : public device::I2C
{
public:
	MMC5616_I2C(const I2CSPIDriverConfig &config);
	virtual ~MMC5616_I2C() = default;

	virtual int read(unsigned address, void *data, unsigned count) override;
	virtual int write(unsigned address, void *data, unsigned count) override;

protected:
	virtual int probe();
};

MMC5616_I2C::MMC5616_I2C(const I2CSPIDriverConfig &config) :
	I2C(config)
{
}

int MMC5616_I2C::probe()
{
	uint8_t chip_id, product_id;

	read(static_cast<uint8_t>(Register::R_CHIP_ID), &chip_id, sizeof(chip_id));
	read(static_cast<uint8_t>(Register::R_PRODUCT_ID), &product_id, sizeof(product_id));

	if (chip_id != EXPECTED_CHIP_ID) {
		DEVICE_DEBUG("unexpected Chip ID: 0x%02x (0x%02x)", chip_id, EXPECTED_CHIP_ID);
	}

	if (product_id != EXPECTED_PRODUCT_ID) {
		DEVICE_DEBUG("unexpected Product ID: 0x%02x (0x%02x)", product_id, EXPECTED_PRODUCT_ID);
		return -EIO;
	}

	PX4_DEBUG("Detected device: ChipID 0x%02x, ProductID 0x%02x", chip_id, product_id);

	return OK;
}

int MMC5616_I2C::read(unsigned address, void *data, unsigned count)
{
	uint8_t cmd = address;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}

int MMC5616_I2C::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address;
	memcpy(&buf[1], data, count);

	return transfer(&buf[0], count + 1, nullptr, 0);
}

device::Device *MMC5616_I2C_interface(const I2CSPIDriverConfig &config)
{
	return new MMC5983MA_I2C(config);
}
