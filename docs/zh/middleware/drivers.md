# 驱动开发

PX4 device drivers are based on the [Device](https://github.com/PX4/PX4-Autopilot/tree/main/src/lib/drivers/device) framework.

## 创建驱动程序

PX4 almost exclusively consumes data from [uORB](../middleware/uorb.md). 常见外设类型的驱动程序必须发布正确的 uORB 消息（例如: 陀螺仪、加速度计、压力传感器等）。

The best approach for creating a new driver is to start with a similar driver as a template (see [src/drivers](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers)).

:::info
More detailed information about working with specific I/O buses and sensors may be available in [Sensor and Actuator Buses](../sensor_bus/index.md) section.
:::

:::info
Publishing the correct uORB topics is the only pattern that drivers _must_ follow.
:::

## 核心架构

PX4 is a [reactive system](../concept/architecture.md) and uses [uORB](../middleware/uorb.md) publish/subscribe to transport messages. File handles are not required or used for the core operation of the system. Two main APIs are used:

- Publish / subscribe 系统具有文件、网络或共享内存后端，具体取决于系统 PX4 运行。
- The global device registry, which can be used to enumerate devices and get/set their configuration. This can be as simple as a linked list or map to the file system. 这可以像链接列表或映射到文件系统一样简单。

## 设备ID

For the example of three magnetometers on a system, use the flight log (.px4log) to dump the parameters. The three parameters encode the sensor IDs and <code>MAG_PRIME</code> identifies which magnetometer is selected as the primary sensor. Each MAGx_ID is a 24bit number and should be padded left with zeros for manual decoding. 这三个参数解码传感器的 ID， 并且 <code>MAG_PRIME</code> 区分那个磁力计作为主传感器。

The order of sensors (e.g. if there is a `/dev/mag0` and an alternate `/dev/mag1`) does not determine priority - the priority is instead stored as part of the published uORB topic.

### 解码示例

This is the internal HMC5983 connected via SPI, bus 1, slave select slot 5. It will show up in the log file as <code>IMU1.MagX</code>. The three parameters encode the sensor IDs and `MAG_PRIME` identifies which magnetometer is selected as the primary sensor. Each MAGx_ID is a 24bit number and should be padded left with zeros for manual decoding.

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

根据此格式，设备 ID 是一个24bit 数字。 It will show up in the log file as `IMU1.MagX`.

```
# device ID 66826 in 24-bit binary:
00000001  00000101  00001 010

# decodes to:
HMC5883   dev 5   bus 1 SPI
```

And this is the internal MPU9250 magnetometer connected via SPI, bus 1, slave select slot 4. It will show up in the log file as `IMU2.MagX`.

```
# device ID 263178 in 24-bit binary:
00000100  00000100  00001 010

#decodes to:
MPU9250   dev 4   bus 1 SPI
```

### 设备 ID 编码

The device ID is a 24bit number according to this format. Note that the first fields are the least significant bits in the decoding example above.

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

## 调试

For general debugging topics see: [Debugging/Logging](../debug/index.md).

### 使用操纵杆

Drivers (and other modules) output minimally verbose logs strings by default (e.g. for `PX4_DEBUG`, `PX4_WARN`, `PX4_ERR`, etc.).

Log verbosity is defined at build time using the `RELEASE_BUILD` (default), `DEBUG_BUILD` (verbose) or `TRACE_BUILD` (extremely verbose) macros.

Change the logging level using `COMPILE_FLAGS` in the driver `px4_add_module` function (**CMakeLists.txt**).
The code fragment below shows the required change to enable DEBUG_BUILD level debugging for a single module or driver.

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
