# Modules Reference: Magnetometer (Driver)

## ak09916

Source: [drivers/magnetometer/akm/ak09916](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/magnetometer/akm/ak09916)

### Usage {#ak09916_usage}

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

### Usage {#ak8963_usage}

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

### Usage {#bmm150_usage}

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

### Usage {#bmm350_usage}

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

### Usage {#hmc5883_usage}

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

### Usage {#iis2mdc_usage}

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

### Usage {#ist8308_usage}

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

### Usage {#ist8310_usage}

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

## lis3mdl

Source: [drivers/magnetometer/lis3mdl](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/magnetometer/lis3mdl)

### Usage {#lis3mdl_usage}

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

### Usage {#lsm9ds1_mag_usage}

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

### Usage {#mmc5983ma_usage}

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

### Usage {#qmc5883l_usage}

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

### Usage {#rm3100_usage}

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

### Usage {#vcm1193l_usage}

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
