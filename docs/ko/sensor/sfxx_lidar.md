# LightWare Lidar (SF1X/SF02/LW20/SF45)

LightWare는 UAV에 장착에 적합한 경량의 범용 레이저 고도계( "라이다")를 개발합니다.
지형 추적, 정밀 호버링 (예 : 사진 촬영), 규제 높이 제한 경고, 충돌 방지 감지 등에 사용됩니다.

<img src="../../assets/hardware/sensors/lidar_lightware/sf11c_120_m.jpg" width="350px" alt="LightWare SF11/C Lidar"/>![LightWare SF45 rotating Lidar](../../assets/hardware/sensors/lidar_lightware/sf45.png)

## 지원 모델

아래의 모델들은 PX4에서 지원되며, I2C 또는 직렬 버스에 연결할 수 있습니다 (아래 표는 각 모델에 사용할 수 있는 버스를 나타냄).

| 모델                                                         | 범위 (m) | 버스           | 설명                                                                                                            |
| ---------------------------------------------------------- | ------------------------- | ------------ | ------------------------------------------------------------------------------------------------------------- |
| [SF11/C](https://lightwarelidar.com/products/sf11-c-100-m) | 100                       | 직렬 또는 I2C 버스 |                                                                                                               |
| [LW20/C](https://lightware.co.za/products/lw20-c-100-m)    | 100                       | I2C 버스       | 감지 및 회피 애플리케이션을 위한 서보가 있는 방수 (IP67)                                                        |
| [SF45/B](../sensor/sf45_rotating_lidar.md)                 | 50                        | 직렬           | Rotary Lidar (Used for [Collision Prevention](../computer_vision/collision_prevention.md)) |

:::details
Discontinued

The following models are supported by PX4 but are no longer available from the manufacturer.

| 모델                                                                                                 | 범위   | 버스           |                                                        |
| -------------------------------------------------------------------------------------------------- | ---- | ------------ | ------------------------------------------------------ |
| [SF02](http://documents.lightware.co.za/SF02%20-%20Laser%20Rangefinder%20Manual%20-%20Rev%208.pdf) | 50   | 직렬           |                                                        |
| [SF10/A](http://documents.lightware.co.za/SF10%20-%20Laser%20Altimeter%20Manual%20-%20Rev%206.pdf) | 25   | 직렬 또는 I2C 버스 |                                                        |
| [SF10/B](http://documents.lightware.co.za/SF10%20-%20Laser%20Altimeter%20Manual%20-%20Rev%206.pdf) | 50   | 직렬 또는 I2C 버스 |                                                        |
| SF10/C                                                                                             | 100m | 직렬 또는 I2C 버스 |                                                        |
| LW20/B                                                                                             | 50   | I2C 버스       | 감지 및 회피 애플리케이션을 위한 서보가 있는 방수 (IP67) |

:::

## I2C 설정

I2C 포트에 연결 가능한 모델을 위의 표를 참고하십시오.

### Lidar Configuration (SF11/C)

The SF11/C hardware (only) does not ship with Pixhawk I2C compatibility enabled by default.
To enable support, you have to download [LightWare Studio](https://lightwarelidar.com/pages/lightware-studio) and got to **Parameters > Communication** and tick mark **I2C compatibility mode (Pixhawk)**

![LightWare SF11/C Lidar-I2C Config](../../assets/hardware/sensors/lidar_lightware/lightware_studio_i2c_config.jpg)

This step is not required for the other supported Lightware rangefinders.

### 하드웨어

Connect the Lidar the autopilot I2C port as shown below (in this case, for the [Pixhawk 1](../flight_controller/mro_pixhawk.md)).

![SF1XX LIDAR to I2C connection](../../assets/hardware/sensors/lidar_lightware/sf1xx_i2c.jpg)

:::info
Some older revisions cannot be used with PX4.
Specifically they may be miss-configured to have an I2C address equal to `0x55`, which conflicts with `rgbled` module.
On Linux systems you may be able to determine the address using [i2cdetect](https://linux.die.net/man/8/i2cdetect).
If the I2C address is equal to `0x66` the sensor can be used with PX4.
:::

### Parameter Setup {#i2c_parameter_setup}

Set the [SENS_EN_SF1XX](../advanced_config/parameter_reference.md#SENS_EN_SF1XX) parameter to match the rangefinder model and then reboot.

VTOL vehicles may choose to also set [SF1XX_MODE](../advanced_config/parameter_reference.md#SF1XX_MODE) to `2: Disabled during VTOL fast forward flight`.

## Serial Setup {#serial_hardware_setup}

:::tip
[SF45/B](../sensor/sf45_rotating_lidar.md) setup is covered in the linked document.
:::

### 하드웨어

The lidar can be connected to any unused _serial port_ (UART), e.g.: TELEM2, TELEM3, GPS2 etc.

<!-- Would be good to show serial setup! -->

### Parameter Setup {#serial_parameter_setup}

[Configure the serial port](../peripherals/serial_configuration.md) on which the lidar will run using [SENS_SF0X_CFG](../advanced_config/parameter_reference.md#SENS_SF0X_CFG).
포트 전송속도는 드라이버에 의해 설정되므로, 추가로 설정하지 않아도 됩니다.

:::info
If the configuration parameter is not available in _QGroundControl_ then you may need to [add the driver to the firmware](../peripherals/serial_configuration.md#parameter_not_in_firmware).
:::

Then set the [SENS_EN_SF0X](../advanced_config/parameter_reference.md#SENS_EN_SF0X) parameter to match the rangefinder model and reboot.

VTOL vehicles may choose to also set [SF1XX_MODE](../advanced_config/parameter_reference.md#SF1XX_MODE) to `2: Disabled during VTOL fast forward flight`.

## 추가 정보

- [Modules Reference: Distance Sensor (Driver) : lightware_laser_i2c](../modules/modules_driver_distance_sensor.md#lightware-laser-i2c)
- [Modules Reference: Distance Sensor (Driver) : lightware_laser_serial](../modules/modules_driver_distance_sensor.md#lightware-laser-serial)
