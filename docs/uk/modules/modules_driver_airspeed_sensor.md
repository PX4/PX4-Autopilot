# Modules Reference: Airspeed Sensor (Driver)

## asp5033

Source: [drivers/differential_pressure/asp5033](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/differential_pressure/asp5033)

### Опис

Драйвер для включення зовнішнього [ASP5033]
(https://www.qio-tek.com/index.php/product/qiotek-asp5033-dronecan-airspeed-and-compass-module/)
TE підключається через I2C.
За замовчуванням це не включено до прошивки. It can be included with terminal command: "make <your_board> boardconfig"
or in default.px4board with adding the line: "CONFIG_DRIVERS_DIFFERENTIAL_PRESSURE_ASP5033=y"
It can be enabled with the "SENS_EN_ASP5033" parameter set to 1.

<a id="asp5033_usage"></a>

### Використання

```
asp5033 <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 109

   stop

   status        print status info
```

## auav

Source: [drivers/differential_pressure/auav](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/differential_pressure/auav)

<a id="auav_usage"></a>

### Використання

```
auav <command> [arguments...]
 Commands:
   start
     [-D]        Differential pressure sensing
     [-A]        Absolute pressure sensing
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 38

   stop

   status        print status info
```

## ets_airspeed

Source: [drivers/differential_pressure/ets](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/differential_pressure/ets)

<a id="ets_airspeed_usage"></a>

### Використання

```
ets_airspeed <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 117

   stop

   status        print status info
```

## ms4515

Source: [drivers/differential_pressure/ms4515](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/differential_pressure/ms4515)

<a id="ms4515_usage"></a>

### Використання

```
ms4515 <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 70

   stop

   status        print status info
```

## ms4525do

Source: [drivers/differential_pressure/ms4525do](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/differential_pressure/ms4525do)

<a id="ms4525do_usage"></a>

### Використання

```
ms4525do <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 40

   stop

   status        print status info
```

## ms5525dso

Source: [drivers/differential_pressure/ms5525dso](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/differential_pressure/ms5525dso)

<a id="ms5525dso_usage"></a>

### Використання

```
ms5525dso <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 118

   stop

   status        print status info
```

## sdp3x

Source: [drivers/differential_pressure/sdp3x](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/differential_pressure/sdp3x)

<a id="sdp3x_usage"></a>

### Використання

```
sdp3x <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 33
     [-k]        if initialization (probing) fails, keep retrying periodically

   stop

   status        print status info
```
