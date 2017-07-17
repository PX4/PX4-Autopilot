#include "INA219.hpp"

//-------------------------------------------------------------------------//
/**
 * Constructor
 * @param bus  i2c device bus
 */
linux_ina219::INA219::INA219(int bus, int address) {
	this->__device_bus = bus;
	this->__device_address = address;
}

linux_ina219::INA219::~INA219() {

}
//-----------------------------------------------------------------------------------------------------------//
/**
 * 开启设备，并设置为i2c slave模式
 */
int linux_ina219::INA219::open_fd() {
	char bus_file[64];
	snprintf(bus_file, sizeof(bus_file), "/dev/i2c-%d", this->__device_bus);

	if ((this->__device_fd = open(bus_file, O_RDWR)) < 0) {
		PX4_ERR("Couldn't open I2C Bus %d [open_fd():open %d]",
				this->__device_bus, errno);
		return -1;
	}

	if (ioctl(this->__device_fd, I2C_SLAVE, this->__device_address) < 0) {
		PX4_ERR("I2C slave %d failed [open_fd():ioctl %d]", address, errno);
		return -1;
	}
	return this->__device_fd;
}
//---------------------------------------------------------------------------------------------------------//
/**
 * 关闭设备
 */
int linux_ina219::INA219::close_fd() {
	::close(this->__device_fd);
}

//---------------------------------------------------------------------------------------------------------//
/**
 * 从8位的地址寄存器中读取16位数据
 */
int linux_ina219::INA219::read16(uint8_t reg, uint8_t *value, int length) {

	return 0;
}
//---------------------------------------------------------------------------------------------------------//
/**
 * 将16位数据，写入8位地址
 */
int linux_ina219::INA219::write16(uint8_t reg, uint16_t *value, int length) {
	int status = this->open_fd();
	if (status <= 0)
		return status;
	uint8_t write_buffer[length + 1];
	write_buffer[0] = reg;
	if (nullptr != value && value) {
		memcpy(&write_buffer[1], in_buffer, length);
	}

	if (length + 1 > MAX_LEN_TRANSMIT_BUFFER_IN_BYTES) {
		PX4_ERR("error: caller's buffer exceeds max len");
		return -1;
	}

	int retry_count = 0;
	int retries = 2;
	int result = -1;
	do {
		ssize_t bytes_written = ::write(this->__device_fd,
				(char *) write_buffer, sizeof(write_buffer));

		if (bytes_written != (ssize_t) sizeof(write_buffer)) {
			PX4_ERR("Error: i2c write failed. Reported %zd bytes written",
					bytes_written);

		} else {
			result = 0;
			break;
		}

	} while (retry_count++ < _retries);
	this->close_fd();
	return result;
}

