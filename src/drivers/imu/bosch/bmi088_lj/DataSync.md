# BMI085 data synchronization

##	Introduction
BMI085 is a system-in-package inertial measurement unit which offers accurate acceleration and angular rate measurements. Due to system-in-package approach (two sensors in single package), the gyroscope and acceleration data is acquired in a non-synchronized manner. However, the synchronization between accelerometer and gyroscope can be easily achieved. This document describes how synchronization between accelerometer and gyroscope can be achieved in a typical application such as augmented or virtual reality.

To achieve data synchronization on BMI085, the data ready interrupt signal from the gyroscope of the BMI085 needs to be connected to one of the interrupt pins of the BMI085 accelerometer (which can be configured as input pins). The internal signal processing unit of the accelerometer uses the data ready signal from the gyroscope to synchronize and interpolate the data of the accelerometer, considering the group delay of the sensors. The accelerometer part can then notify the host of available data. With this technique, it is possible to achieve synchronized data and provide accelerometer data at an ODR of 2 kHz.

_Note: data synchronization is designed for applications requiring high bandwidth, for which BMI085 is intended, but it works also with BMI088. However, for some applications it is desirable to have a very low bandwidth, and partly having different bandwidth settings for accelerometer and gyroscope. Here the synchronization feature does not make sense._

##	Concept
Synchronized data means that the acquisition of the gyroscope and accelerometer data is happening at the same time and the signals have same propagation time. The time between motion to register read-out depends on the physical propagation time mainly caused by signal filtering path. The synchronization between accelerometer and gyroscope data to a common point of time and a common group delay can be realized with the approach described in the following sections.

The hardware interrupts pins (INT1 / INT3) of the BMI085 are used for data synchronization purposes and must be connected. The interrupt pin INT2 can be used for data ready notification to the host by BMI085.

##	Technical realization
The data synchronization feature requires physical interrupt pin’s connection of the sensors on the pcb and a special configuration of the BMI085. Requirements and the steps are described below.

### Connection diagram

```
       MCU                    BMI085
     +-----------+             +---------------+
     |           |           8 |               | 16
     |       SCK +------------>| SCK      INT1 |<-----+
     |           |           9 |               | 12   |
     |      MOSI +------------>| SDO      INT3 |>-----+
     |           |          15 |               | 
     |      MISO +<------+-----| SDO1          | 
     |           |       |  10 |               |
     |           |       +-----| SDO2          |
     |           |          14 |               |
     |   ACC_CSB +------------>| CSB1          |
     |           |           5 |               |
     |  GYRO_CSB +------------>| CSB2          | 
     |           |           1 |               |
     |  DATA_RDY +<------------| INT2       PS +------+
     |           |             |               |      |
     +-----------+             +---------------+      |
                                                      |
                                                     ---
```

For latency-critical multisensory applications, it is recommended to use SPI interface for fastest sensor data read (recommended SPI clock speed is >2MHz).

### Configuring BMI085 for data synchronization

Include the bmi08x header 

``` c
#include "bmi08x.h"
```

Update variant of bmi08x_dev to BMI085_VARIANT to use the BMI085 sensor feature

```
 dev.variant = BMI085_VARIANT;
```

To initialize BMI085 for data synchronization, an instance of the bmi08x structure should be created. The following parameters are required to be updated in the structure by the user:


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

The following code is simplified code, for example no checking of return values is added, to make it easier to read the code.

int8_t rslt;

uint8_t acc_dev_addr = 0;
uint8_t gyro_dev_addr = 0;

struct bmi08x_dev dev = {

        .intf_ptr_accel = &acc_dev_addr,
        .intf_ptr_gyro = &gyro_dev_addr,
        .intf = BMI08X_SPI_INTF,  
        .read = user_spi_read,  
        .write = user_spi_write,  
        .delay_us = user_delay_milli_sec,
        .variant = BMI085_VARIANT
};

/* Initializing the bmi085 sensors the below functions */

/* To Initialize accel sensor */

rslt = bmi08a_init(&dev);

/* To Initialize gyro sensor */

rslt = bmi08g_init(&dev);

/* Reset the accelerometer and wait for 1 ms - delay taken care inside the function */

rslt = bmi08a_soft_reset(&bmi08xdev);


/*! Max read/write length (maximum supported length is 32).
 To be set by the user */

bmi08xdev.read_write_len = 32;

/*set accel power mode */

bmi08xdev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;

rslt = bmi08a_set_power_mode(&bmi08xdev);

bmi08xdev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;

bmi08g_set_power_mode(&bmi08xdev);

/* API uploads the bmi08x config file onto the device and wait for 150ms 
   to enable the data synchronization - delay taken care inside the function */

rslt = bmi08a_load_config_file(&bmi08xdev);

/*assign accel range setting*/

bmi08xdev.accel_cfg.range = BMI085_ACCEL_RANGE_4G;

/*assign gyro range setting*/

bmi08xdev.gyro_cfg.range = BMI08X_GYRO_RANGE_2000_DPS;

/*! Mode (0 = off, 1 = 400Hz, 2 = 1kHz, 3 = 2kHz) */

sync_cfg.mode = BMI08X_ACCEL_DATA_SYNC_MODE_2000HZ;

rslt = bmi08a_configure_data_synchronization(sync_cfg, &bmi08xdev);


/*set accel interrupt pin configuration*/

/*configure host data ready interrupt */

int_config.accel_int_config_1.int_channel = BMI08X_INT_CHANNEL_1;

int_config.accel_int_config_1.int_type = BMI08X_ACCEL_SYNC_INPUT;

int_config.accel_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

int_config.accel_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;

int_config.accel_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

/*configure Accel syncronization input interrupt pin */

int_config.accel_int_config_2.int_channel = BMI08X_INT_CHANNEL_2;

int_config.accel_int_config_2.int_type = BMI08X_ACCEL_SYNC_DATA_RDY_INT;

int_config.accel_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

int_config.accel_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;

int_config.accel_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

/*set gyro interrupt pin configuration*/

int_config.gyro_int_config_1.int_channel = BMI08X_INT_CHANNEL_3;

int_config.gyro_int_config_1.int_type = BMI08X_GYRO_DATA_RDY_INT;

int_config.gyro_int_config_1.int_pin_cfg.enable_int_pin = BMI08X_ENABLE;

int_config.gyro_int_config_1.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;

int_config.gyro_int_config_1.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

int_config.gyro_int_config_2.int_channel = BMI08X_INT_CHANNEL_4;

int_config.gyro_int_config_2.int_type = BMI08X_GYRO_DATA_RDY_INT;

int_config.gyro_int_config_2.int_pin_cfg.enable_int_pin = BMI08X_DISABLE;

int_config.gyro_int_config_2.int_pin_cfg.lvl = BMI08X_INT_ACTIVE_HIGH;

int_config.gyro_int_config_2.int_pin_cfg.output_mode = BMI08X_INT_MODE_PUSH_PULL;

/* Enable synchronization interrupt pin */

rslt = bmi08a_set_data_sync_int_config(&int_config, &bmi08xdev);

#### Read out raw accel data

/* Declare an instance of the sensor data structure for accel */

static struct bmi08x_sensor_data accel_bmi085;

rslt = bmi08a_get_data(&accel_bmi085, &bmi08xdev);


#### Read out synchronized  data

/* Declare an instance of the sensor data structure for accel */

static struct bmi08x_sensor_data accel_bmi085;

/* Declare an instance of the sensor data structure for gyro */

static struct bmi08x_sensor_data gyro_bmi085;

rslt = bmi08a_get_synchronized_data(&accel_bmi085,&gyro_bmi085, &bmi08xdev);

