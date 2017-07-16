#include "linux_ina219.hpp"

//-------------------------------------------------------------------------//
/**
 * Constructor
 * @param bus  i2c device bus
 */
linux_ina219::INA219::INA219(int bus){
	this->__device_bus = bus;
}

linux_ina219::INA219::~INA219(){

}

int linux_ina219::INA219::open(){

	return -1;
}

int linux_ina219::INA219::read16(uint8_t reg ,uint8_t *value, int length){

	return 0;
}

int linux_ina219::INA219::write16(uint8_t reg,uint8_t *value,int length){

	return 0;
}


