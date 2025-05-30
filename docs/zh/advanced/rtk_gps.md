# RTK GPS (PX4 集成)

[Real Time Kinematic](https://en.wikipedia.org/wiki/Real_Time_Kinematic) (RTK) provides centimeter-level GPS accuracy.
这一章节将介绍 RTK 是如何集成到 PX4 中的。

:::tip
Instructions for _using_ RTK GNSS are provided in [Hardware > RTK GPS](../gps_compass/rtk_gps.md).
:::

## 综述

RTK是使用导航信号的载波相位来进行测距的，而不是使用导航信号所搭载的信息。
它依靠一个单一的参考基站站实时校正，这种校正可以与多个流动站一起工作。

PX4 配置 RTK 需要两个 RTK GPS 模块和一个数传。
The fixed-position ground-based GPS unit is called the _Base_ and the in-air unit is called the _Rover_.
The Base unit connects to _QGroundControl_ (via USB) and uses the datalink to stream RTCM corrections to the vehicle (using the MAVLink [GPS_RTCM_DATA](https://mavlink.io/en/messages/common.html#GPS_RTCM_DATA) message).
在自驾仪上，MAVLink消息包被解包得到RTCM的修正信息，并把这些信息发送给移动站，移动站结合修正信息最终解算得到 RTK 解。

The datalink should typically be able to handle an uplink rate of 300 bytes per second (see the [Uplink Datarate](#uplink-datarate) section below for more information).

## 支持的 RTK GPS 模块

The list of devices that we have tested can be found [in the user guide](../gps_compass/rtk_gps.md#supported-devices).

:::info
Most devices come with two variants, a base and a rover.
确保选择正确的变体。
:::

## 自动配置

The PX4 GPS stack automatically sets up the GPS modules to send and receive the correct messages over the UART or USB, depending on where the module is connected (to _QGroundControl_ or the autopilot).

As soon as the autopilot receives `GPS_RTCM_DATA` MAVLink messages, it automatically forwards the RTCM data to the attached GPS module over existing data channels (a dedicated channel for correction data is not required).

:::info
The u-blox U-Center RTK module configuration tool is not needed/used!
:::

:::info
Both _QGroundControl_ and the autopilot firmware share the same [PX4 GPS driver stack](https://github.com/PX4/GpsDrivers).
实际上，这意味着只需要将新协议和/或消息添加到一个地方。
:::

### RTCM 报文

QGroundControl 配置RTK 基地站输出以下 RTCM3.2 帧, 每个帧均为 1 Hz, 除非另有说明：

- **1005** - Station coordinates XYZ for antenna reference point (Base position), 0.2 Hz.
- **1077** - Full GPS pseudo-ranges, carrier phases, Doppler and signal strength (high resolution).
- **1087** - Full GLONASS pseudo-ranges, carrier phases, Doppler and signal strength (high resolution).
- **1230** - GLONASS code-phase biases.
- **1097** - Full Galileo pseudo-ranges, carrier phases, Doppler and signal strength (high resolution)
- **1127** - Full BeiDou pseudo-ranges, carrier phases, Doppler and signal strength (high resolution)

## 上行数据速率

The raw RTCM messages from the base are packed into a MAVLink `GPS_RTCM_DATA` message and sent over the datalink.
MAVLink 消息的最大长度是182字节。 根据RTCM的信息类型，MAVLink信息是不会填满的。

RTCM 基础位置消息(1005)长度为 22 字节， 而其他卫星的长度则因可见卫星的数量和卫星信号的数量而异（M8P等L1单元只有一个）。
Since at a given time, the _maximum_ number of satellites visible from any single constellation is 12, under real-world conditions, theoretically an uplink rate of 300 B/s is sufficient.

If _MAVLink 1_ is used, a 182-byte `GPS_RTCM_DATA` message is sent for every RTCM message, irrespective of its length.
因此，大约每秒上行需求是700多个字节。
这可能导致低带宽半双轨遥测模块 (如3DR Telemetry Radios) 连接的饱和。

If _MAVLink 2_ is used then any empty space in the `GPS_RTCM_DATA message` is removed.
由此产生的上行链路需求与理论值 (约 300 字节/秒) 大致相同。

:::tip
PX4 automatically switches to MAVLink 2 if the GCS and telemetry modules support it.
:::

MAVLink 2 必须用于低带宽链接以保证 RTK 性能。 必须注意确保数传链在整个过程中使用 MAVLink 2。
You can verify the protocol version by using the `mavlink status` command on the system console:

```sh
nsh> mavlink status
instance #0:
        GCS heartbeat:  593486 us ago
        mavlink chan: #0
        type:           3DR RADIO
        rssi:           219
        remote rssi:    219
        txbuf:          94
        noise:          61
        remote noise:   58
        rx errors:      0
        fixed:          0
        flow control:   ON
        rates:
        tx: 1.285 kB/s
        txerr: 0.000 kB/s
        rx: 0.021 kB/s
        rate mult: 0.366
        accepting commands: YES
        MAVLink version: 2
        transport protocol: serial (/dev/ttyS1 @57600)
```
