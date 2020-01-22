#include "ADS1115.h"
#include <drivers/device/i2c.h>

/* PX4 universal low-level driver wrapper for ADS1115 */
class ADS1115_Drv : public device::I2C, public I2C_Interface
{
public:
	ADS1115_Drv(uint8_t bus, uint8_t address, const char *devname);
	~ADS1115_Drv() override = default;
	int init() override;

	void readReg(uint8_t addr, uint8_t *buf, size_t len) override;

	void writeReg(uint8_t addr, uint8_t *buf, size_t len) override;

protected:
	ADS1115 *_ads1115 = nullptr;

private:

};

ADS1115_Drv::ADS1115_Drv(uint8_t bus, uint8_t address, const char *devname) :
	I2C("ads1115", devname, bus, address, 400000)
{
	_ads1115 = new ADS1115(this);

	if (_ads1115 == nullptr) {
		PX4_ERR("ads1115 logical driver alloc failed");
	}
}

int ADS1115_Drv::init()
{
	if (_ads1115 == nullptr) {
		return -ENOMEM;
	}

	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("I2C init failed");
		return ret;
	}

	ret = _ads1115->init();

	if (ret != PX4_OK) {
		PX4_ERR("ADS1115 init failed");
		return ret;
	}

	return PX4_OK;
}

void ADS1115_Drv::readReg(uint8_t addr, uint8_t *buf, size_t len)
{
	transfer(&addr, 1, buf, len);
}

void ADS1115_Drv::writeReg(uint8_t addr, uint8_t *buf, size_t len)
{
	uint8_t buffer[len + 1];
	buffer[0] = addr;
	memcpy(buffer + 1, buf, sizeof(uint8_t)*len);
	transfer(buffer, len + 1, nullptr, 0);
}

extern "C" int ads1115_main(int argc, char *argv[])
{
	return 0;
}