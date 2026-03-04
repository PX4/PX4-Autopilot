# Розробка драйверів

PX4 device drivers are based on the [Device](https://github.com/PX4/PX4-Autopilot/tree/main/src/lib/drivers/device) framework.

## Створення драйвера

PX4 almost exclusively consumes data from [uORB](../middleware/uorb.md). Драйвери для поширених типів периферійних пристроїв повинні публікувати правильні повідомлення uORB (наприклад: гіроскоп, акселерометр, датчики тиску тощо).

The best approach for creating a new driver is to start with a similar driver as a template (see [src/drivers](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers)).

:::info
More detailed information about working with specific I/O buses and sensors may be available in [Sensor and Actuator Buses](../sensor_bus/index.md) section.
:::

:::info
Publishing the correct uORB topics is the only pattern that drivers _must_ follow.
:::

## Архітектура ядра

PX4 is a [reactive system](../concept/architecture.md) and uses [uORB](../middleware/uorb.md) publish/subscribe to transport messages. Файлові дескриптори не потрібні або не використовуються для основної роботи системи. Використовуються дві основні API:

- Система публікації/підписки, яка має файлову, мережеву або загальну пам'ять, залежно від системи, на якій працює PX4.
- Глобальний реєстр пристроїв, який можна використовувати для переліку пристроїв та отримання/налаштування їх конфігурації. Це може бути простий зв'язаний список або схема файлової системи.

## Ідентифікатори пристрою(ID)

PX4 використовує ідентифікатори пристроїв для постійної ідентифікації окремих сенсорів у всій системі. Ці ідентифікатори зберігаються в параметрах конфігурації та використовуються для відповідності значень калібрування сенсора, а також для визначення того, який сенсор зареєстрований в якому записі журналу.

The order of sensors (e.g. if there is a `/dev/mag0` and an alternate `/dev/mag1`) does not determine priority - the priority is instead stored as part of the published uORB topic.

### Приклад декодування

Для прикладу трьох магнітометрів у системі використовуйте журнал польотів (.px4log) для вивантаження параметрів. The three parameters encode the sensor IDs and `MAG_PRIME` identifies which magnetometer is selected as the primary sensor. Кожен MAGx_ID є 24-бітним числом і повинен бути доповнений зліва нулями для ручного декодування.

```
CAL_MAG0_ID = 73225.0
CAL_MAG1_ID = 66826.0
CAL_MAG2_ID = 263178.0
CAL_MAG_PRIME = 73225.0
```

This is the external HMC5983 connected via I2C, bus 1 at address `0x1E`: It will show up in the log file as `IMU.MagX`.

```
# device ID 73225 in 24-bit binary:
00000001  00011110  00001 001

# decodes to:
HMC5883   0x1E    bus 1 I2C
```

Це внутрішній HMC5983, підключений через SPI, шина 1, слот вибору slave 5. It will show up in the log file as `IMU1.MagX`.

```
# device ID 66826 in 24-bit binary:
00000001  00000101  00001 010

# decodes to:
HMC5883   dev 5   bus 1 SPI
```

І це внутрішній магнітометр MPU9250, підключений через SPI, шина 1, слот вибору slave 4. It will show up in the log file as `IMU2.MagX`.

```
# device ID 263178 in 24-bit binary:
00000100  00000100  00001 010

#decodes to:
MPU9250   dev 4   bus 1 SPI
```

### Кодування ID пристрою

Ідентифікатор пристрою - це 24-бітове число відповідно до цього формату. Зверніть увагу, що перші поля - це молодші біти у наведеному вище прикладі декодування.

```C
struct DeviceStructure {
  enum DeviceBusType bus_type : 3;
  uint8_t bus: 5;    // which instance of the bus type
  uint8_t address;   // address on the bus (eg. I2C address)
  uint8_t devtype;   // device class specific device type
};
```

The `bus_type` is decoded according to:

```C
enum DeviceBusType {
  DeviceBusType_UNKNOWN = 0,
  DeviceBusType_I2C     = 1,
  DeviceBusType_SPI     = 2,
  DeviceBusType_UAVCAN  = 3,
};
```

and `devtype` is decoded according to:

```C
#define DRV_MAG_DEVTYPE_HMC5883  0x01
#define DRV_MAG_DEVTYPE_LSM303D  0x02
#define DRV_MAG_DEVTYPE_ACCELSIM 0x03
#define DRV_MAG_DEVTYPE_MPU9250  0x04
#define DRV_ACC_DEVTYPE_LSM303D  0x11
#define DRV_ACC_DEVTYPE_BMA180   0x12
#define DRV_ACC_DEVTYPE_MPU6000  0x13
#define DRV_ACC_DEVTYPE_ACCELSIM 0x14
#define DRV_ACC_DEVTYPE_GYROSIM  0x15
#define DRV_ACC_DEVTYPE_MPU9250  0x16
#define DRV_GYR_DEVTYPE_MPU6000  0x21
#define DRV_GYR_DEVTYPE_L3GD20   0x22
#define DRV_GYR_DEVTYPE_GYROSIM  0x23
#define DRV_GYR_DEVTYPE_MPU9250  0x24
#define DRV_RNG_DEVTYPE_MB12XX   0x31
#define DRV_RNG_DEVTYPE_LL40LS   0x32
```

## Відлагодження

For general debugging topics see: [Debugging/Logging](../debug/index.md).

### Докладне ведення журналу

Drivers (and other modules) output minimally verbose logs strings by default (e.g. for `PX4_DEBUG`, `PX4_WARN`, `PX4_ERR`, etc.).

Log verbosity is defined at build time using the `RELEASE_BUILD` (default), `DEBUG_BUILD` (verbose) or `TRACE_BUILD` (extremely verbose) macros.

Change the logging level using `COMPILE_FLAGS` in the driver `px4_add_module` function (**CMakeLists.txt**).
У наведеному нижче фрагменті коду показано необхідну зміну, щоб увімкнути налагодження на рівні DEBUG_BUILD для окремого модуля або драйвера.

```
px4_add_module(
	MODULE templates__module
	MAIN module
```

```
	COMPILE_FLAGS
		-DDEBUG_BUILD
```

```
	SRCS
		module.cpp
	DEPENDS
		modules__uORB
	)
```

:::tip
Verbose logging can also be enabled on a per-file basis, by adding `#define DEBUG_BUILD` at the very top of a .cpp file (before any includes).
:::
