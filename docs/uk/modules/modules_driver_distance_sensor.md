# Modules Reference: Distance Sensor (Driver)

## afbrs50

Source: [drivers/distance_sensor/broadcom/afbrs50](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/broadcom/afbrs50)

### Опис

Драйвер для Broadcom AFBRS50.

### Приклади

Спроба запустити драйвер на вказаному послідовному пристрої.

```
afbrs50 start
```

Stop driver

```
afbrs50 stop
```

<a id="afbrs50_usage"></a>

### Використання

```
afbrs50 <command> [arguments...]
 Commands:
   start         Start driver
     -d <val>    Serial device
     [-r <val>]  Sensor rotation - downward facing by default
                 default: 25

   test          Test driver

   stop          Stop driver
```

## gy_us42

Source: [drivers/distance_sensor/gy_us42](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/gy_us42)

<a id="gy_us42_usage"></a>

### Використання

```
gy_us42 <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-R <val>]  Sensor rotation - downward facing by default
                 default: 25

   stop

   status        print status info
```

## leddar_one

Source: [drivers/distance_sensor/leddar_one](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/leddar_one)

### Опис

Драйвер послідовної шини для LiDAR LeddarOne.

Більшість плат налаштовано на ввімкнення/запуск драйвера на вказаному UART за допомогою параметра SENS_LEDDAR1_CFG.

Інформація про налаштування/використання: https://docs.px4.io/main/en/sensor/leddar_one.html

### Приклади

Спроба запустити драйвер на вказаному послідовному пристрої.

```
leddar_one start -d /dev/ttyS1
```

Stop driver

```
leddar_one stop
```

<a id="leddar_one_usage"></a>

### Використання

```
leddar_one <command> [arguments...]
 Commands:
   start         Start driver
     -d <val>    Serial device
     [-r <val>]  Sensor rotation - downward facing by default
                 default: 25

   stop          Stop driver
```

## lightware_laser_i2c

Source: [drivers/distance_sensor/lightware_laser_i2c](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/lightware_laser_i2c)

### Опис

Драйвер шини I2C для LIDAR-далекомірів Lightware серії SFxx: SF10/a, SF10/b, SF10/c, SF11/c, SF/LW20.

Інформація про налаштування/використання: https://docs.px4.io/main/en/sensor/sfxx_lidar.html

<a id="lightware_laser_i2c_usage"></a>

### Використання

```
lightware_laser_i2c <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 102
     [-R <val>]  Sensor rotation - downward facing by default
                 default: 25

   stop

   status        print status info
```

## lightware_laser_serial

Source: [drivers/distance_sensor/lightware_laser_serial](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/lightware_laser_serial)

### Опис

Драйвер послідовної шини для лазерних далекомірів LightWare SF02/F, SF10/a, SF10/b, SF10/c, SF11/c.

Більшість плат налаштовано на увімкнення/запуск драйвера на вказаному UART за допомогою параметра SENS_SF0X_CFG.

Інформація про налаштування/використання: https://docs.px4.io/main/en/sensor/sfxx_lidar.html

### Приклади

Спроба запустити драйвер на вказаному послідовному пристрої.

```
lightware_laser_serial start -d /dev/ttyS1
```

Stop driver

```
lightware_laser_serial stop
```

<a id="lightware_laser_serial_usage"></a>

### Використання

```
lightware_laser_serial <command> [arguments...]
 Commands:
   start         Start driver
     -d <val>    Serial device
     [-R <val>]  Sensor rotation - downward facing by default
                 default: 25

   stop          Stop driver
```

## lightware_sf45_serial

Source: [drivers/distance_sensor/lightware_sf45_serial](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/lightware_sf45_serial)

### Опис

Драйвер послідовної шини для лазерного далекоміра Lightware SF45/b.

### Приклади

Спроба запустити драйвер на вказаному послідовному пристрої.

```
lightware_sf45_serial start -d /dev/ttyS1
```

Stop driver

```
lightware_sf45_serial stop
```

<a id="lightware_sf45_serial_usage"></a>

### Використання

```
lightware_sf45_serial <command> [arguments...]
 Commands:
   start         Start driver
     -d <val>    Serial device

   stop          Stop driver
```

## ll40ls

Source: [drivers/distance_sensor/ll40ls](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/ll40ls)

### Опис

Драйвер шини I2C для далекомірів LidarLite.

Датчик/драйвер має бути увімкнений за допомогою параметра SENS_EN_LL40LS.

Інформація про налаштування/використання: https://docs.px4.io/main/en/sensor/lidar_lite.html

<a id="ll40ls_usage"></a>

### Використання

```
ll40ls <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 98
     [-R <val>]  Sensor rotation - downward facing by default
                 default: 25

   regdump

   stop

   status        print status info
```

## mappydot

Source: [drivers/distance_sensor/mappydot](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/mappydot)

<a id="mappydot_usage"></a>

### Використання

```
mappydot <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)

   stop

   status        print status info
```

## mb12xx

Source: [drivers/distance_sensor/mb12xx](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/mb12xx)

<a id="mb12xx_usage"></a>

### Використання

```
mb12xx <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)

   set_address
     [-a <val>]  I2C address
                 default: 112

   stop

   status        print status info
```

## pga460

Source: [drivers/distance_sensor/pga460](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/pga460)

### Опис

Драйвер ультразвукового далекоміра, який здійснює зв'язок з пристроєм і публікує відстань через uORB.

### Імплементація

Цей драйвер реалізовано як завдання NuttX. Ця реалізація була обрана через необхідність опитування на повідомлення
через UART, що не підтримується у work_queue. Цей драйвер безперервно вимірює дальність коли він
працює. На рівні драйверів реалізовано простий алгоритм виявлення хибних показань, що має на меті покращити
якість даних, що публікуються. Драйвер взагалі не публікуватиме дані, якщо вважатиме, що дані датчика
недійсними або нестабільними.

<a id="pga460_usage"></a>

### Використання

```
pga460 <command> [arguments...]
 Commands:
   start
     [device_path] The pga460 sensor device path, (e.g: /dev/ttyS6)

   status

   stop

   help
```

## srf02

Source: [drivers/distance_sensor/srf02](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/srf02)

<a id="srf02_usage"></a>

### Використання

```
srf02 <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 112
     [-R <val>]  Sensor rotation - downward facing by default
                 default: 25

   stop

   status        print status info
```

## srf05

Source: [drivers/distance_sensor/srf05](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/srf05)

### Опис

Драйвер для далекомірів HY-SRF05 / HC-SR05 та HC-SR04.

Датчик/драйвер потрібно увімкнути за допомогою параметра SENS_EN_HXSRX0X.

<a id="srf05_usage"></a>

### Використання

```
srf05 <command> [arguments...]
 Commands:
   start         Start driver
     [-R <val>]  Sensor rotation - downward facing by default
                 default: 25

   status        Print driver status information

   stop          Stop driver

   stop

   status        print status info
```

## teraranger

Source: [drivers/distance_sensor/teraranger](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/teraranger)

### Опис

Драйвер шини I2C для далекомірів TeraRanger.

Датчик/драйвер має бути увімкнений за допомогою параметра SENS_EN_TRANGER.

Інформація про налаштування/використання: https://docs.px4.io/main/en/sensor/rangefinders.html#teraranger-rangefinders

<a id="teraranger_usage"></a>

### Використання

```
teraranger <command> [arguments...]
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
     [-R <val>]  Sensor rotation - downward facing by default
                 default: 25

   stop

   status        print status info
```

## tf02pro

Source: [drivers/distance_sensor/tf02pro](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/tf02pro)

<a id="tf02pro_usage"></a>

### Використання

```
tf02pro <command> [arguments...]
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
     [-R <val>]  Sensor rotation - downward facing by default
                 default: 25

   stop

   status        print status info
```

## tfmini

Source: [drivers/distance_sensor/tfmini](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/tfmini)

### Опис

Серійний драйвер шини для Benewake TFmini LiDAR.

Більшість плат налаштовано на ввімкнення/вимкнення драйвера на вказаному UART за допомогою параметра SENS_TFMINI_CFG.

Інформація про налаштування/використання: https://docs.px4.io/main/en/sensor/tfmini.html

### Приклади

Спроба запустити драйвер на вказаному послідовному пристрої.

```
tfmini start -d /dev/ttyS1
```

Stop driver

```
tfmini stop
```

<a id="tfmini_usage"></a>

### Використання

```
tfmini <command> [arguments...]
 Commands:
   start         Start driver
     -d <val>    Serial device
     [-R <val>]  Sensor rotation - downward facing by default
                 default: 25

   status        Driver status

   stop          Stop driver

   test          Test driver (basic functional tests)

   status        Print driver status
```

## ulanding_radar

Source: [drivers/distance_sensor/ulanding_radar](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/ulanding_radar)

### Опис

Серійний драйвер шини для радара Aerotenna uLanding.

Setup/usage information: https://docs.px4.io/main/en/sensor/ulanding_radar.html

### Приклади

Спроба запустити драйвер на вказаному послідовному пристрої.

```
ulanding_radar start -d /dev/ttyS1
```

Stop driver

```
ulanding_radar stop
```

<a id="ulanding_radar_usage"></a>

### Використання

```
ulanding_radar <command> [arguments...]
 Commands:
   start         Start driver
     -d <val>    Serial device
                 values: <file:dev>, default: /dev/ttyS3
     [-R <val>]  Sensor rotation - downward facing by default
                 default: 25

   stop          Stop driver
```

## vl53l0x

Source: [drivers/distance_sensor/vl53l0x](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/vl53l0x)

<a id="vl53l0x_usage"></a>

### Використання

```
vl53l0x <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 41
     [-R <val>]  Sensor rotation - downward facing by default
                 default: 25

   stop

   status        print status info
```

## vl53l1x

Source: [drivers/distance_sensor/vl53l1x](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/vl53l1x)

<a id="vl53l1x_usage"></a>

### Використання

```
vl53l1x <command> [arguments...]
 Commands:
   start
     [-I]        Internal I2C bus(es)
     [-X]        External I2C bus(es)
     [-b <val>]  board-specific bus (default=all) (external SPI: n-th bus
                 (default=1))
     [-f <val>]  bus frequency in kHz
     [-q]        quiet startup (no message if no device found)
     [-a <val>]  I2C address
                 default: 41
     [-R <val>]  Sensor rotation - downward facing by default
                 default: 25

   stop

   status        print status info
```
