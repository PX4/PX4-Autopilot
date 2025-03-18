# 드라이버 개발

PX4 device drivers are based on the [Device](https://github.com/PX4/PX4-Autopilot/tree/main/src/lib/drivers/device) framework.

## 드라이버 생성

PX4 almost exclusively consumes data from [uORB](../middleware/uorb.md). 일반적인 주변 장치 유형에 대한 드라이버는 올바른 uORB 메시지(예: 자이로, 가속도계, 압력 센서 등)를 게시하여야 합니다.

The best approach for creating a new driver is to start with a similar driver as a template (see [src/drivers](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers)).

:::info
More detailed information about working with specific I/O buses and sensors may be available in [Sensor and Actuator Buses](../sensor_bus/index.md) section.
:::

:::info
Publishing the correct uORB topics is the only pattern that drivers _must_ follow.
:::

## 핵심 아키텍처

PX4 is a [reactive system](../concept/architecture.md) and uses [uORB](../middleware/uorb.md) publish/subscribe to transport messages. 파일 핸들은 시스템의 핵심 작업에 필요하지 않거나 사용되지 않습니다. 두 가지 주요 API가 사용됩니다.

- PX4가 실행되는 시스템에 따라 파일, 네트워크 또는 공유 메모리 백엔드가 있는 게시/구독 시스템.
- 장치를 열거하고 구성을 가져오거나 설정할 수 있는 전역 장치 레지스트리. 이것은 연결 목록이나 파일 시스템에 대한 매핑처럼 간단할 수 있습니다.

## 장치 ID

PX4는 장치 ID를 사용하여 시스템 전체에서 개별 센서를 일관되게 식별합니다. 이러한 ID는 구성 매개변수에 저장되며, 센서 보정 값을 일치시키고 어떤 센서가 어떤 로그 파일 항목에 기록되는 지 결정합니다.

The order of sensors (e.g. if there is a `/dev/mag0` and an alternate `/dev/mag1`) does not determine priority - the priority is instead stored as part of the published uORB topic.

### 디코딩 예제

시스템에 3개의 자력계가 있는 경우에는 비행 로그(.px4log)를 사용하여 매개변수를 덤프합니다. The three parameters encode the sensor IDs and `MAG_PRIME` identifies which magnetometer is selected as the primary sensor. 각 MAGx_ID는 24비트 숫자이며, 수동 디코딩을 하가 위하여 왼쪽에 0을 채워야 합니다.

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

이것은 SPI, 버스 1, 슬레이브 선택 슬롯 5를 통하여 연결된 내부 HMC5983입니다. It will show up in the log file as `IMU1.MagX`.

```
# device ID 66826 in 24-bit binary:
00000001  00000101  00001 010

# decodes to:
HMC5883   dev 5   bus 1 SPI
```

그리고 이것은 SPI, 버스 1, 슬레이브 선택 슬롯 4를 통하여 연결된 내부 MPU9250 자력계입니다. It will show up in the log file as `IMU2.MagX`.

```
# device ID 263178 in 24-bit binary:
00000100  00000100  00001 010

#decodes to:
MPU9250   dev 4   bus 1 SPI
```

### 장치 ID 인코딩

장치 ID는 이 형식에 따른 24비트 숫자입니다. 첫 번째 필드는 위의 디코딩 예에서 최하위 비트입니다.

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

## 디버깅

For general debugging topics see: [Debugging/Logging](../debug/index.md).

### 상세 로깅

Drivers (and other modules) output minimally verbose logs strings by default (e.g. for `PX4_DEBUG`, `PX4_WARN`, `PX4_ERR`, etc.).

Log verbosity is defined at build time using the `RELEASE_BUILD` (default), `DEBUG_BUILD` (verbose) or `TRACE_BUILD` (extremely verbose) macros.

Change the logging level using `COMPILE_FLAGS` in the driver `px4_add_module` function (**CMakeLists.txt**).
아래 코드 조각은 단일 모듈 또는 드라이버에 대해 DEBUG_BUILD 수준 디버깅을 활성화하에 필요한 변경 사항을 나타냅니다.

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
