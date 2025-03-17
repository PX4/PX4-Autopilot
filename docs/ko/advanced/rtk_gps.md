# RTK GPS (PX4 통합)

[Real Time Kinematic](https://en.wikipedia.org/wiki/Real_Time_Kinematic) (RTK) provides centimeter-level GPS accuracy.
RTK와 PX4 통합 방법을 설명합니다.

:::tip
Instructions for _using_ RTK GNSS are provided in [Hardware > RTK GPS](../gps_compass/rtk_gps.md).
:::

## 개요

RTK는 신호 보다는 신호의 반송파 위상을 사용합니다.
여러 모바일 스테이션에서 실시간 수정을 제공하기 위하여 단일 참조 스테이션에 의존합니다.

PX4로 RTK를 설정에는 2개의 RTK GPS 모듈과 데이터 링크가 필요합니다.
The fixed-position ground-based GPS unit is called the _Base_ and the in-air unit is called the _Rover_.
The Base unit connects to _QGroundControl_ (via USB) and uses the datalink to stream RTCM corrections to the vehicle (using the MAVLink [GPS_RTCM_DATA](https://mavlink.io/en/messages/common.html#GPS_RTCM_DATA) message).
자동조종장치에서 MAVLink 패킷의 압축을 풀고 로버에 전송하여, RTK 솔루션을 처리합니다.

The datalink should typically be able to handle an uplink rate of 300 bytes per second (see the [Uplink Datarate](#uplink-datarate) section below for more information).

## 지원 RTK GPS 모듈

The list of devices that we have tested can be found [in the user guide](../gps_compass/rtk_gps.md#supported-devices).

:::info
Most devices come with two variants, a base and a rover.
올바른 변형을 선택하여야 합니다.
:::

## 자동 설정

The PX4 GPS stack automatically sets up the GPS modules to send and receive the correct messages over the UART or USB, depending on where the module is connected (to _QGroundControl_ or the autopilot).

As soon as the autopilot receives `GPS_RTCM_DATA` MAVLink messages, it automatically forwards the RTCM data to the attached GPS module over existing data channels (a dedicated channel for correction data is not required).

:::info
The u-blox U-Center RTK module configuration tool is not needed/used!
:::

:::info
Both _QGroundControl_ and the autopilot firmware share the same [PX4 GPS driver stack](https://github.com/PX4/GpsDrivers).
실제로, 새 프로토콜 또는 메시지 지원시 한쪽에만 추가하면 됩니다.
:::

### RTCM 메시지

명시되어 있지 않은 경우에는 QGroundControl은 RTCM3.2 프레임을 1Hz로 출력하도록 RTK 베이스 스테이션을 구성합니다.

- **1005** - Station coordinates XYZ for antenna reference point (Base position), 0.2 Hz.
- **1077** - Full GPS pseudo-ranges, carrier phases, Doppler and signal strength (high resolution).
- **1087** - Full GLONASS pseudo-ranges, carrier phases, Doppler and signal strength (high resolution).
- **1230** - GLONASS code-phase biases.
- **1097** - Full Galileo pseudo-ranges, carrier phases, Doppler and signal strength (high resolution)
- **1127** - Full BeiDou pseudo-ranges, carrier phases, Doppler and signal strength (high resolution)

## 업링크 데이터 속도

The raw RTCM messages from the base are packed into a MAVLink `GPS_RTCM_DATA` message and sent over the datalink.
MAVLink 메시지의 최대 길이는 182바이트입니다. RTCM 메시지에 따라 MAVLink 메시지는 채워지지 않을 수도 있습니다.

RTCM Base Position 메시지(1005)의 길이는 22바이트이고, 나머지는 모두 가시 위성의 수와 위성의 신호 수에 따라 가변 길이입니다(M8P와 같은 L1 장치의 경우 1만).
Since at a given time, the _maximum_ number of satellites visible from any single constellation is 12, under real-world conditions, theoretically an uplink rate of 300 B/s is sufficient.

If _MAVLink 1_ is used, a 182-byte `GPS_RTCM_DATA` message is sent for every RTCM message, irrespective of its length.
결과적으로, 대략적인 업링크 요구 사항은 초당 약 700+바이트입니다.
이는 저대역폭 반이중 원격 측정 모듈(예: 3DR 원격 측정 라디오)에서 링크 포화로 이어질 수 있습니다.

If _MAVLink 2_ is used then any empty space in the `GPS_RTCM_DATA message` is removed.
결과적인 업링크 요구 사항은 이론적인 값(초당 ~300바이트)과 거의 같습니다.

:::tip
PX4 automatically switches to MAVLink 2 if the GCS and telemetry modules support it.
:::

MAVLink 2는 고성능 RTK을 위하여, 낮은 대역폭 링크를 사용합니다. 텔레메트리 체인에서 MAVLink 2를 사용하는지를 확인해야합니다.
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
