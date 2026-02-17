# ARK DIST MR

ARK DIST MR is a mid range, open source [DroneCAN](index.md) [distance sensor](../sensor/rangefinders.md).

![ARK DIST MR](../../assets/hardware/sensors/optical_flow/ark_dist.jpg)

## Де купити

Замовте цей модуль з:

- [ARK Electronics](https://arkelectron.com/product/ark-dist-mr/) (US)

## Характеристики обладнання

- [Open Source Schematic and BOM](https://github.com/ARK-Electronics/ARK_DIST)
- Датчики
  - [Broadcom AFBR-S50LX85D Time-of-Flight Distance Sensor](https://www.broadcom.com/products/optical-sensors/time-of-flight-3d-sensors/afbr-s50lx85d)
    - Typical distance range up to 50m
    - Інтегроване джерело світла лазера 850 нм
    - Поле зору (FoV) 12,4° x 6,2° з 32 пікселями
    - Робота в умовах 200 тис. люксів світла навколишнього середовища
    - Reference Pixel for system health monitoring
    - Добре працює на всіх поверхнях
    - Трансмітер пучка 2° x 2° для підсвічування між 1 та 3 пікселями
- Два роз'єми стандарту CAN для Pixhawk (4 контакти JST GH)
- Pixhawk Standard UART Connector (6 Pin JST SH)
- Роз'єм для відлагодження стандарту Pixhawk (6 контактів JST SH)
- Малий форм-фактор
  - 2.0cm x 2.8cm x 1.4cm
  - 4g
- LED індикатори
- USA Built
- NDAA Compliant
- Вимоги до живлення
  - 5v
    - 78mA Average
    - 84mA Max

## Налаштування програмного забезпечення

### Підключення

The ARK DIST is connected to the CAN bus using a Pixhawk standard 4 pin JST GH cable.
For more information, refer to the [CAN Wiring](../can/index.md#wiring) instructions.

The ARK DIST can also be connected with UART and communicates over MAVLink sending the [DISTANCE_SENSOR](https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR) message.

## Налаштування прошивки

ARK DIST MR runs the [PX4 DroneCAN Firmware](px4_cannode_fw.md).
As such, it supports firmware update over the CAN bus and [dynamic node allocation](index.md#node-id-allocation).

## Конфігурація PX4

### DroneCAN

#### Увімкнути DroneCAN

Кроки наступні:

- In _QGroundControl_ set the parameter [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to `2` for dynamic node allocation (or `3` if using [DroneCAN ESCs](../dronecan/escs.md)) and reboot (see [Finding/Updating Parameters](../advanced_config/parameters.md)).
- Connect ARK DIST SR CAN to the Pixhawk CAN.

Після активації модуль буде виявлено при завантаженні.
Distance sensor data should arrive at 40Hz.

DroneCAN configuration in PX4 is explained in more detail in [DroneCAN > Enabling DroneCAN](../dronecan/index.md#enabling-dronecan).

#### CAN Configuration

First set the parameters to [Enable DroneCAN](#enable-dronecan) (as shown above).

Set the following parameters in _QGroundControl_:

- Enable [UAVCAN_ENABLE](../advanced_config/parameter_reference.md#UAVCAN_ENABLE) to 2 for dynamic node allocation.
- Enable [UAVCAN_SUB_RNG](../advanced_config/parameter_reference.md#UAVCAN_SUB_RNG).
- Set [EKF2_RNG_A_HMAX](../advanced_config/parameter_reference.md#EKF2_RNG_A_HMAX) to `50`.
- Set [EKF2_RNG_QLTY_T](../advanced_config/parameter_reference.md#EKF2_RNG_QLTY_T) to `0.2`.
- Set [UAVCAN_RNG_MIN](../advanced_config/parameter_reference.md#UAVCAN_RNG_MIN) to `0.08`.
- Set [UAVCAN_RNG_MAX](../advanced_config/parameter_reference.md#UAVCAN_RNG_MAX) to `50`.

See also [Distance Sensor/Range Finder in _DroneCAN > Subscriptions and Publications_](../dronecan/#distance-sensor-range-finder).

### UART/MAVLink Configuration

If connecting via a UART set the following parameters in _QGroundControl_:

- Set [MAV_X_CONFIG](../advanced_config/parameter_reference.md#MAV_0_CONFIG) to the port the sensor is connected to.
- Set [MAV_X_FORWARD](../advanced_config/parameter_reference.md#MAV_0_FORWARD) to `0` (off).
- Set [MAV_X_MODE](../advanced_config/parameter_reference.md#MAV_0_MODE) to `7` or `13` to (Minimal or Low Bandwidth) to reduce memory usage.
- Set `SER_XXX_BAUD` to `115200`, where `XXX` is specific to the port you are using (such as [SER_GPS2_BAUD](../advanced_config/parameter_reference.md#SER_GPS2_BAUD)).
- Set [EKF2_RNG_A_HMAX](../advanced_config/parameter_reference.md#EKF2_RNG_A_HMAX) to `50`.
- Set [EKF2_RNG_QLTY_T](../advanced_config/parameter_reference.md#EKF2_RNG_QLTY_T) to `0.2`.

## Дивіться також

- [ARK DIST MR](https://docs.arkelectron.com/sensor/ark-dist) (ARK Docs)
