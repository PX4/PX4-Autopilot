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
//-----------------------------------------------------------------------------//
linux_ina219::INA219::~INA219() {
	if (0 < this->__device_fd)
		this->close_fd();
}
//----------------------------------------------------------------------------//
/**
 * ina219 32v1A校准
 */
void linux_ina219::INA219::calibration32v2a() {
	if (-1 == this->open_fd())
		return;
	//32V 2A
	__ina219_currentDivider_mA = 10; // Current LSB = 100uA per bit (1000/100 = 10)
	__ina219_powerDivider_mW = 2;     // Power LSB = 1mW per bit (2/1)
	__ina219_calValue = 4096;
	// Set Calibration register to 'Cal' calculated above
	this->write16(INA219_REG_CALIBRATION, &__ina219_calValue,sizeof(__ina219_calValue));

	// Set Config register to take into account the settings above
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
	INA219_CONFIG_GAIN_8_320MV |
	INA219_CONFIG_BADCRES_12BIT |
	INA219_CONFIG_SADCRES_12BIT_1S_532US |
	INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	this->write16(INA219_REG_CONFIG, &config, sizeof(config));
}
//-----------------------------------------------------------------------------------------------------------//
/**
 * ina219 32v1A校准
 */
void linux_ina219::INA219::calibration32v1a() {
	__ina219_currentDivider_mA = 25;  // Current LSB = 40uA per bit (1000/40 = 25)
	__ina219_powerDivider_mW = 1;         // Power LSB = 800�W per bit
	// Set Calibration register to 'Cal' calculated above
	__ina219_calValue=10240;
	this->write16(INA219_REG_CALIBRATION, &this->__ina219_calValue, (size_t)sizeof(__ina219_calValue));

	// Set Config register to take into account the settings above
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
	INA219_CONFIG_GAIN_8_320MV |
	INA219_CONFIG_BADCRES_12BIT |
	INA219_CONFIG_SADCRES_12BIT_1S_532US |
	INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	this->write16(INA219_REG_CONFIG, &config,sizeof(config));
}
//-----------------------------------------------------------------------------------------------------------//
/**
 * ina219 16v 400ma 校准
 */
void linux_ina219::INA219::calibration16v400ma(){
	__ina219_calValue = 8192;
	__ina219_currentDivider_mA = 20;  // Current LSB = 50uA per bit (1000/50 = 20)
	__ina219_powerDivider_mW = 1;
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
	                    INA219_CONFIG_GAIN_1_40MV |
	                    INA219_CONFIG_BADCRES_12BIT |
	                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
	INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
	this->write16(INA219_REG_CONFIG, &config,sizeof(config));
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
		PX4_ERR("I2C slave %d failed ", this->__device_address);
		return -1;
	}
	return this->__device_fd;
}
//---------------------------------------------------------------------------------------------------------//
float linux_ina219::INA219::getShuntVoltage(){
	  uint16_t value;
	  int status = this->read16(INA219_REG_SHUNTVOLTAGE, &value,sizeof(value));
	  if(0>status)
		  return -1;
	  return (float)value;
}
//---------------------------------------------------------------------------------------------------------//
float linux_ina219::INA219::getCurrent(){
	 uint16_t value;

	  // Sometimes a sharp load will reset the INA219, which will
	  // reset the cal register, meaning CURRENT and POWER will
	  // not be available ... avoid this by always setting a cal
	  // value even if it's an unfortunate extra step
	  this->write16(INA219_REG_CALIBRATION, &__ina219_calValue, sizeof(__ina219_calValue));

	  // Now we can safely read the CURRENT register!
	  this->read16(INA219_REG_CURRENT, &value, sizeof(value));
	  return (float)value;
}


//---------------------------------------------------------------------------------------------------------//
float linux_ina219::INA219::getBusVoltage(){
	  uint16_t value;
	  uint16_t result;
	  int status = this->read16(INA219_REG_BUSVOLTAGE, &value,sizeof(value));
	  // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
	  result = (int16_t)((value >> 3) * 4);
	  if(0>status)
		  return -1;
	  return value*0.001;
}
//----------------------------------------------------------------------------------------------------------//
float linux_ina219::INA219::getCurrentMa(){
	 uint16_t value;

	  // Sometimes a sharp load will reset the INA219, which will
	  // reset the cal register, meaning CURRENT and POWER will
	  // not be available ... avoid this by always setting a cal
	  // value even if it's an unfortunate extra step
	 int status = this->write16(INA219_REG_CALIBRATION, &__ina219_calValue,sizeof(__ina219_calValue));
	 if(status<0)
		 return -1;
	 status = this->read16(INA219_REG_CURRENT, &value,sizeof(value));
	 if(status<0)
		 return -1;
	 float result = (float)value/__ina219_currentDivider_mA;
	 return result;
}

//---------------------------------------------------------------------------------------------------------//
/**
 * close fd
 * 关闭设备
 */
int linux_ina219::INA219::close_fd() {
	::close(this->__device_fd);
	return 0;
}

//---------------------------------------------------------------------------------------------------------//
/**
 * 从8位的地址寄存器中读取16位数据
 * @param uint8_t reg 8位的寄存器地址
 * @param uint16_t* value 目标数据位置
 * @param length 数据长度
 *
 */
int linux_ina219::INA219::read16(uint8_t reg, uint16_t *value, size_t length) {
	int status = this->open_fd();
	if (status <= 0)
		return status;
	//缓冲区
	uint8_t write_buffer[length + 1];
	write_buffer[0] = reg;
	int _retry_count = 0;
	int _retries = 2;
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

	} while (_retry_count++ < _retries);
	//未能读取切换到寄存器地址，直接返回
	if (-1==result){
		PX4_INFO("未能切换到寄存器地址");
		return result;
	}
	//将输入读入value变量中
	ssize_t bytes_read = 0;
	bytes_read = ::read(this->__device_fd, (uint8_t*) value, length);
	this->close_fd();
	if (bytes_read != (ssize_t) length) {
		PX4_ERR("Can not read data from ina219, @ %d", reg);
		return -1;
	}
	return 0;
}
//---------------------------------------------------------------------------------------------------------//
/**
 * 将16位数据，写入8位地址
 */
int linux_ina219::INA219::write16(uint8_t reg, uint16_t *value, size_t length) {
	int status = this->open_fd();
	if (status <= 0)
		return status;
	uint8_t write_buffer[length + 1];
	write_buffer[0] = reg;
	if (nullptr != value && value) {
		memcpy(&write_buffer[1], value, length);
	}

	if (length + 1 > 128) {
		PX4_ERR("error: caller's buffer exceeds max len");
		return -1;
	}

	int _retry_count = 0;
	int _retries = 2;
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

	} while (_retry_count++ < _retries);
	this->close_fd();
	return result;
}

