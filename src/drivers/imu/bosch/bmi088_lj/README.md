# BMI08X Sensor API

## Table of Contents
 - [Introduction](#Intro)
 - [Integration details](#Integration)
 - [Driver files information](#file)
 - [Sensor interfaces](#interface)
 - [Integration Examples](#examples)

### Introduction<a name=Intro></a>
This package contains Bosch Sensortec's BMI08X Sensor API.


### Integration details<a name=Integration></a>
- Integrate _bmi08a.c_, _bmi08g.c_,_bmi08x_defs.h_ and _bmi08x.h_ in your project.

Update variant of bmi08x_dev to BMI085_VARIANT to use the BMI085 sensor feature

```
 dev.variant = BMI085_VARIANT;
```

Update variant of bmi08x_dev to BMI088_VARIANT to use the BMI085 sensor feature

```
 dev.variant = BMI088_VARIANT;
```

- User has to include _bmi08x.h_ in the code to call sensor APIs as shown below :

```
 #include "bmi08x.h"
```

### Driver files information<a name=file></a>
- *_bmi08a.c_*
   * This file has function definitions of bmi08x accel generic API interfaces.
- *_bmi08g.c_*
   * This file has function definitions of bmi08x gyro generic API interfaces.   
- *_bmi08x.h_*
   * This header file has necessary include files,bmi08x function declarations, required to make API calls.
 - *_bmi08x_defs.h_*
   * This header file has necessary include files, macro definitions, typedefs and data structure definitions.
 
### Sensor interfaces<a name=interface></a>
- I2C interface
- SPI interface  
_Note: By default, the interface is I2C._

### Integration examples<a name=examples></a>
#### Initializing BMI085 sensors
 /* below code shows bmi085 integration steps */
To initialize BMI085 sensors, an instance of the bmi08x structure should be created. The following parameters are required to be updated in the structure, by the user, to initialize bmi085 sensors.

Parameters          | Details
--------------------|--------------------------------------------------------
_intf_ptr_accel_    | Interface pointer that can hold Accel device address
_intf_ptr_gyro_     | Interface pointer that can hold Gyro device address    
_intf_              | I2C or SPI 
_read_              | read through I2C/SPI interface
_write_             | write through I2C/SPI interface
_delay_us_          | delay in microseconds
_variant_           | BMI085_VARIANT

##### _Initialize through SPI interface_

int8_t rslt;

uint8_t acc_dev_addr = 0;
uint8_t gyro_dev_addr = 0;

struct bmi08x_dev dev = {

        .intf_ptr_accel = &acc_dev_addr,
        .intf_ptr_gyro = &gyro_dev_addr,
        .intf = BMI08X_SPI_INTF,  
        .read = user_spi_read,  
        .write = user_spi_write,  
        .delay_us = user_delay,
        .variant = BMI085_VARIANT 
};

/* Initialize the SPI */

/* Initializing the bmi085 sensors the below functions */

/* To Initialize accel sensor */

rslt = bmi08a_init(&dev);

/* To Initialize gyro sensor */

rslt = bmi08g_init(&dev);

##### _Initialize through I2C interface_

/* I2C slave address depends on the hardware configuration for details please refer Data sheet */

int8_t rslt;

uint8_t acc_dev_addr = BMI08X_ACCEL_I2C_ADDR_PRIMARY; /* User has define this macro depends on the I2C slave address */

uint8_t gyro_dev_addr = BMI08X_GYRO_I2C_ADDR_PRIMARY; /* User has define this macro depends on the I2C slave address */

struct bmi08x_dev dev = {

        .intf_ptr_accel = &acc_dev_addr,
        .intf_ptr_gyro = &gyro_dev_addr,
        .intf = BMI08X_I2C_INTF,  
        .read = user_i2c_read,  
        .write = user_i2c_write,  
        .delay_ms = user_delay,
        .variant = BMI085_VARIANT
};

/* Initialize the I2C */

/* Initializing the bmi085 sensors the below functions */

/* To Initialize accel sensor */

rslt = bmi08a_init(&dev);

/* To Initialize gyro sensor */

rslt = bmi08g_init(&dev);

#### Read Chip ID from the accel


int8_t rslt;
uint8_t data = 0;

/* Initialize the device instance as per the initialization example */

if(rslt == BMI08X_OK) 
{

    /* Read accel chip id */
    rslt = bmi08a_get_regs(BMI08X_ACCEL_CHIP_ID_REG, &data, 1, &dev);
}

#### Get the accel power mode

int8_t rslt;

/* Initialize the device instance as per the initialization example */

/* Read the accel power mode */

rslt = bmi08a_get_power_mode(&dev);

/* Power mode will be updated in the dev.accel_cfg.power */
	

#### Get the accelerometer configurations

int8_t rslt;

/* Initialize the device instance as per the initialization example */

/* Read the accel sensor config parameters (odr,bw,range) */

rslt = bmi08a_get_meas_conf(&dev);

/* Config parameters will be updated in the  dev.accel_cfg.odr,dev.accel_cfg.bw and dev.accel_cfg.range */
	

#### Configuring the accelerometer

int8_t rslt;

/* Initialize the device instance as per the initialization example */

/* Assign the desired configurations */

dev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;

dev.accel_cfg.odr = BMI08X_ACCEL_ODR_100_HZ;

dev.accel_cfg.range = BMI085_ACCEL_RANGE_4G;

dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;


rslt = bmi08a_set_power_mode(&dev);

/* Wait for 10ms to switch between the power modes - delay taken care inside the function */

rslt = bmi08a_set_meas_conf(&dev);
	

#### Get accelerometer data


int8_t rslt;
struct bmi08x_sensor_data user_accel_bmi085;

/* Initialize the device instance as per the initialization example */

/* Read the sensor data into the sensor data instance */

rslt = bmi08a_get_data(&user_accel_bmi085, &dev);


#### Interrupt Configuring for accel data ready interrupt

/* Mapping data ready interrupt to interrupt channel */

int8_t rslt;
struct bmi08x_accel_int_channel_cfg int_config;

/* Initialize the device instance as per the initialization example */

/* Interrupt configurations */

int_config.int_channel = BMI08X_INT_CHANNEL_1;

int_config.int_type = BMI08X_ACCEL_DATA_RDY_INT;

int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;

int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

/* Configure the controller port pin for the interrupt and assign the ISR */
	
/* Setting the interrupt configuration */

rslt = bmi08a_set_int_config(&int_config, &dev);

	
void interrupt_handler(void)
{
	/* ISR functionality */
}

/* Unmapping data ready interrupt to interrupt channel */

int8_t rslt;

struct bmi08x_accel_int_channel_cfg int_config;

/* Initialize the device instance as per the initialization example */

/* Interrupt configurations */

int_config.int_channel = BMI08X_INT_CHANNEL_1;

int_config.int_type = BMI08X_ACCEL_DATA_RDY_INT;

int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;

int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;


/* Setting the interrupt configuration */

rslt = bmi08a_set_int_config(&int_config, &dev);

/* Configure the controller port pin for disabling the interrupt */

void interrupt_handler(void)
{
	/* ISR functionality */
}


#### Get the sensor time

int8_t rslt;

uint32_t user_sampling_time;

/* Initialize the device instance as per the initialization example */
	
/* Read the sensor time */
rslt = bmi08a_get_sensor_time(&dev, &user_sampling_time);
	

#### Read Chip ID from the gyro

int8_t rslt;
uint8_t data = 0;

/* Initialize the device instance as per the initialization example */

if(rslt == BMI08X_OK) 
{

    /* Read gyro chip id */
    rslt = bmi08g_get_regs(BMI08X_GYRO_CHIP_ID_REG, &data, 1, &dev);
}
			

#### Get the gyro power mode


int8_t rslt;

/* Initialize the device instance as per the initialization example */

/* Read the gyro power mode */

rslt = bmi08g_get_power_mode(&dev)

/* Power mode will be updated in the dev.gyro_cfg.power */
	

#### Get the gyro sensor config


int8_t rslt;

/* Initialize the device instance as per the initialization example */

/* Read the gyro sensor config parameters (odr,bw,range) */

rslt = bmi08g_get_meas_conf(&dev);

/* Config parameters will be updated in the dev.gyro_cfg.odr,dev.gyro_cfg.bw and dev.gyro_cfg.range */
	

#### Configuring the gyro

int8_t rslt;

/* Initialize the device instance as per the initialization example */
	
dev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;

rslt = bmi08g_set_power_mode(&dev);

/* Wait for 30ms to switch between the power modes - delay taken care inside the function */
	
/* Assign the desired configurations */

dev.gyro_cfg.odr = BMI08X_GYRO_BW_23_ODR_200_HZ;

dev.gyro_cfg.range = BMI08X_GYRO_RANGE_1000_DPS;

dev.gyro_cfg.bw = BMI08X_GYRO_BW_23_ODR_200_HZ;

rslt = bmi08g_set_meas_conf(&dev);
	

#### Get gyro data

int8_t rslt;

struct bmi08x_sensor_data user_gyro_bmi085;

/* Initialize the device instance as per the initialization example */

/* Read the sensor data into the sensor data instance */

rslt = bmi08g_get_data(&user_gyro_bmi085, &dev);


#### Interrupt Configuring for gyro data ready interrupt

int8_t rslt;
struct bmi08x_gyro_int_channel_cfg int_config;

/* Initialize the device instance as per the initialization example */

/* Mapping data ready interrupt to interrupt channel */

int_config.int_channel = BMI08X_INT_CHANNEL_3;

int_config.int_type = BMI08X_GYRO_DATA_RDY_INT;

int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;

int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

int_config.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

/* Setting the interrupt configuration */

rslt = bmi08g_set_int_config(&int_config, &dev);

/* Configure the controller port pin for the interrupt and assign the ISR */
	
void interrupt_handler(void)
{
	/* ISR functionality */
}

/* Disabling gyro data ready interrupt */

struct bmi08x_gyro_int_channel_cfg int_config;

/* Initialize the device instance as per the initialization example */

int_config.int_channel = BMI08X_INT_CHANNEL_3;

int_config.int_type = BMI08X_GYRO_DATA_RDY_INT;

int_config.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;

int_config.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

int_config.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;


/* Setting the interrupt configuration */

rslt = bmi08g_set_int_config(&int_config, &dev);