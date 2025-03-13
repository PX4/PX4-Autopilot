# 모듈 참조: 거리 센서(드라이버)

## afbrs50

Source: [drivers/distance_sensor/broadcom/afbrs50](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/broadcom/afbrs50)

### 설명

Broadcom AFBRS50용 드라이버입니다.

### 예

지정된 직렬 장치에서 드라이버를 시작하려고 합니다.

```
afbrs50 start
```

드라이버를 중지합니다.

```
afbrs50 stop
```

<a id="afbrs50_usage"></a>

### 사용법

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

### 사용법

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

### 설명

LeddarOne LiDAR 직렬 버스 드라이버입니다.

대부분의 보드는 SENS_LEDDAR1_CFG 매개변수를 사용하여, 지정된 UART에서 드라이버를 활성화/시작하도록 설정합니다.

Setup/usage information: https://docs.px4.io/main/en/sensor/leddar_one.html

### 예

지정된 직렬 장치에서 드라이버를 시작하려고 합니다.

```
leddar_one start -d /dev/ttyS1
```

드라이버를 중지합니다.

```
leddar_one stop
```

<a id="leddar_one_usage"></a>

### 사용법

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

### 설명

Lightware SFxx 시리즈 LIDAR 거리 측정기용 I2C 버스 드라이버: SF10/a, SF10/b, SF10/c, SF11/c, SF/LW20.

Setup/usage information: https://docs.px4.io/main/en/sensor/sfxx_lidar.html

<a id="lightware_laser_i2c_usage"></a>

### 사용법

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

### 설명

LightWare SF02/F, SF10/a, SF10/b, SF10/c, SF11/c 레이저 거리 측정기용 직렬 버스 드라이버.

대부분의 보드는 SENS_SF0X_CFG 매개변수를 사용하여 지정된 UART에서 드라이버를 활성화/시작하도록 설정합니다.

Setup/usage information: https://docs.px4.io/main/en/sensor/sfxx_lidar.html

### 예

지정된 직렬 장치에서 드라이버를 시작하려고 합니다.

```
lightware_laser_serial start -d /dev/ttyS1
```

드라이버를 중지합니다.

```
lightware_laser_serial stop
```

<a id="lightware_laser_serial_usage"></a>

### 사용법

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

### 설명

Serial bus driver for the Lightware SF45/b Laser rangefinder.

### 예

지정된 직렬 장치에서 드라이버를 시작하려고 합니다.

```
lightware_sf45_serial start -d /dev/ttyS1
```

드라이버를 중지합니다.

```
lightware_sf45_serial stop
```

<a id="lightware_sf45_serial_usage"></a>

### 사용법

```
lightware_sf45_serial <command> [arguments...]
 Commands:
   start         Start driver
     -d <val>    Serial device

   stop          Stop driver
```

## ll40ls

Source: [drivers/distance_sensor/ll40ls](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/distance_sensor/ll40ls)

### 설명

LidarLite 거리 측정기를 위한 I2C 버스 드라이버입니다.

센서/드라이버는 매개변수 SENS_EN_LL40LS를 사용하여 활성화합니다.

Setup/usage information: https://docs.px4.io/main/en/sensor/lidar_lite.html

<a id="ll40ls_usage"></a>

### 사용법

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

### 사용법

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

### 사용법

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

### 설명

장치와의 통신을 처리하고, uORB를 통해 거리를 게시하는 초음파 거리 측정기 드라이버입니다.

### 구현

This driver is implemented as a NuttX task. 이 구현은 work_queue에서 지원되지 않는 UART를 통해 메시지에 대한 폴링이 필요하기 때문에 선택되었습니다. 이 드라이버는 실행되는 동안 지속적으로 범위 측정을 수행합니다. 잘못된 판독값을 감지하는 간단한 알고리즘은 게시중인 데이터의 품질을 개선하기 위하여 드라이버 수준에서 구현됩니다. 드라이버는 센서 데이터가 유효하지 않거나 불안정하다고 판단되는 경우에는, 데이터를 게시하지 않습니다.

<a id="pga460_usage"></a>

### 사용법

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

### 사용법

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

### 설명

HY-SRF05 / HC-SR05 및 HC-SR04 거리 측정기용 드라이버입니다.

센서/드라이버는 SENS_EN_HXSRX0X 매개변수를 사용하여 활성화합니다.

<a id="srf05_usage"></a>

### 사용법

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

### 설명

TeraRanger 거리 측정기를 위한 I2C 버스 드라이버입니다.

센서/드라이버는 SENS_EN_TRANGER 매개변수를 사용하여 활성화합니다.

Setup/usage information: https://docs.px4.io/main/en/sensor/rangefinders.html#teraranger-rangefinders

<a id="teraranger_usage"></a>

### 사용법

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

### 사용법

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

### 설명

Benewake TFmini LiDAR용 직렬 버스 드라이버입니다.

대부분의 보드는 SENS_TFMINI_CFG 매개변수를 사용하여 지정된 UART에서 드라이버를 활성화/시작하도록 설정합니다.

Setup/usage information: https://docs.px4.io/main/en/sensor/tfmini.html

### 예

지정된 직렬 장치에서 드라이버를 시작하려고 합니다.

```
tfmini start -d /dev/ttyS1
```

드라이버를 중지합니다.

```
tfmini stop
```

<a id="tfmini_usage"></a>

### 사용법

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

### 설명

Aerotenna uLanding 레이더용 직렬 버스 드라이버입니다.

Setup/usage information: https://docs.px4.io/main/en/sensor/ulanding_radar.html

### 예

지정된 직렬 장치에서 드라이버를 시작하려고 합니다.

```
ulanding_radar start -d /dev/ttyS1
```

드라이버를 중지합니다.

```
ulanding_radar stop
```

<a id="ulanding_radar_usage"></a>

### 사용법

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

### 사용법

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

### 사용법

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
