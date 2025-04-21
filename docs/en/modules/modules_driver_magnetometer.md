# Modules Reference: Magnetometer (Driver)
## ak09916
Source: [drivers/magnetometer/akm/ak09916](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/magnetometer/akm/ak09916)

<a id="ak09916_usage"></a>
### Usage
```
ak09916 <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 12
     [-R <val>]  Rotation
                 default: 0

   stop

   status        print status info
```
## ak8963
Source: [drivers/magnetometer/akm/ak8963](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/magnetometer/akm/ak8963)

<a id="ak8963_usage"></a>
### Usage
```
ak8963 <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 12
     [-R <val>]  Rotation
                 default: 0

   stop

   status        print status info
```
## bmm150
Source: [drivers/magnetometer/bosch/bmm150](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/magnetometer/bosch/bmm150)

<a id="bmm150_usage"></a>
### Usage
```
bmm150 <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 16
     [-R <val>]  Rotation
                 default: 0

   stop

   status        print status info
```
## bmm350
Source: [drivers/magnetometer/bosch/bmm350](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/magnetometer/bosch/bmm350)

<a id="bmm350_usage"></a>
### Usage
```
bmm350 <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 20
     [-R <val>]  Rotation
                 default: 0

   stop

   status        print status info
```
## hmc5883
Source: [drivers/magnetometer/hmc5883](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/magnetometer/hmc5883)

<a id="hmc5883_usage"></a>
### Usage
```
hmc5883 <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-s]        Internal SPI bus(es)
     [-S]        External SPI bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-c <val>]  chip-select pin (for internal SPI) or index (for external SPI)
     [-m <val>]  SPI mode
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-R <val>]  Rotation
                 default: 0
     [-T]        Enable temperature compensation

   stop

   status        print status info
```
## iis2mdc
Source: [drivers/magnetometer/st/iis2mdc](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/magnetometer/st/iis2mdc)

<a id="iis2mdc_usage"></a>
### Usage
```
iis2mdc <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 48

   stop

   status        print status info
```
## ist8308
Source: [drivers/magnetometer/isentek/ist8308](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/magnetometer/isentek/ist8308)

<a id="ist8308_usage"></a>
### Usage
```
ist8308 <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 12
     [-R <val>]  Rotation
                 default: 0

   stop

   status        print status info
```
## ist8310
Source: [drivers/magnetometer/isentek/ist8310](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/magnetometer/isentek/ist8310)

<a id="ist8310_usage"></a>
### Usage
```
ist8310 <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 14
     [-R <val>]  Rotation
                 default: 0

   stop

   status        print status info
```
## iis2mdc
Source: [drivers/magnetometer/iis2mdc](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/magnetometer/iis2mdc)

<a id="iis2mdc_usage"></a>
### Usage
```
iis2mdc <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-s]        Internal SPI bus(es)
     [-S]        External SPI bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-c <val>]  chip-select pin (for internal SPI) or index (for external SPI)
     [-m <val>]  SPI mode
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 30
     [-R <val>]  Rotation
                 default: 0

   stop

   status        print status info
```
## lis3mdl
Source: [drivers/magnetometer/lis3mdl](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/magnetometer/lis3mdl)

<a id="lis3mdl_usage"></a>
### Usage
```
lis3mdl <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-s]        Internal SPI bus(es)
     [-S]        External SPI bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-c <val>]  chip-select pin (for internal SPI) or index (for external SPI)
     [-m <val>]  SPI mode
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 30
     [-R <val>]  Rotation
                 default: 0

   reset

   stop

   status        print status info
```
## lsm9ds1_mag
Source: [drivers/magnetometer/lsm9ds1_mag](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/magnetometer/lsm9ds1_mag)

<a id="lsm9ds1_mag_usage"></a>
### Usage
```
lsm9ds1_mag <command> [arguments...]
 Commands:
   start
     [-s]        Internal SPI bus(es)
     [-S]        External SPI bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-c <val>]  chip-select pin (for internal SPI) or index (for external SPI)
     [-m <val>]  SPI mode
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-R <val>]  Rotation
                 default: 0

   stop

   status        print status info
```
## mmc5983ma
Source: [drivers/magnetometer/memsic/mmc5983ma](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/magnetometer/memsic/mmc5983ma)

<a id="mmc5983ma_usage"></a>
### Usage
```
mmc5983ma <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-s]        Internal SPI bus(es)
     [-S]        External SPI bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-c <val>]  chip-select pin (for internal SPI) or index (for external SPI)
     [-m <val>]  SPI mode
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 48
     [-R <val>]  Rotation
                 default: 0

   reset

   stop

   status        print status info
```
## qmc5883l
Source: [drivers/magnetometer/qmc5883l](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/magnetometer/qmc5883l)

<a id="qmc5883l_usage"></a>
### Usage
```
qmc5883l <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 13
     [-R <val>]  Rotation
                 default: 0

   stop

   status        print status info
```
## rm3100
Source: [drivers/magnetometer/rm3100](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/magnetometer/rm3100)

<a id="rm3100_usage"></a>
### Usage
```
rm3100 <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-s]        Internal SPI bus(es)
     [-S]        External SPI bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-c <val>]  chip-select pin (for internal SPI) or index (for external SPI)
     [-m <val>]  SPI mode
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-R <val>]  Rotation
                 default: 0

   stop

   status        print status info
```
## vcm1193l
Source: [drivers/magnetometer/vtrantech/vcm1193l](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/magnetometer/vtrantech/vcm1193l)

<a id="vcm1193l_usage"></a>
### Usage
```
vcm1193l <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-R <val>]  Rotation
                 default: 0

   stop

   status        print status info
```
