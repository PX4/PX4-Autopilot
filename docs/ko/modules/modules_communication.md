# 모듈 참조: 통신

## frsky_telemetry

Source: [drivers/telemetry/frsky_telemetry](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/telemetry/frsky_telemetry)

FrSky 텔레메트리를 지원합니다. D 또는 S.PORT 프로토콜을 자동으로 감지합니다. <a id="frsky_telemetry_usage"></a>

### 사용법

```
frsky_telemetry <command> [arguments...]
 Commands:
   start
     [-d <val>]  Select Serial Device
                 values: <file:dev>, default: /dev/ttyS6
     [-t <val>]  Scanning timeout [s] (default: no timeout)
                 default: 0
     [-m <val>]  Select protocol (default: auto-detect)
                 values: sport|sport_single|sport_single_invert|dtype, default:
                 auto

   stop

   status
```

## mavlink

Source: [modules/mavlink](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mavlink)

### 설명

이 모듈은 직렬 링크 또는 UDP 네트워크에서 사용할 수 있는 MAVLink 프로토콜을 구현합니다.
uORB로 시스템과 통신합니다. 일부 메시지는 모듈에서 직접 처리되고(예: 임무 프로토콜), 다른 메시지는 uORB를 통하여 게시됩니다(예: vehicle_command).

스트림은 차량 자세와 같은 특정 속도로 주기적 메시지를 전송합니다.
mavlink 인스턴스를 시작시에 활성화된 스트림 세트를 속도와 함께 정의하는 모드를 지정할 수 있습니다.
For a running instance, streams can be configured via `mavlink stream` command.

하나의 직렬 장치 또는 네트워크 포트에 각각 연결된 모듈의 여러 독립 인스턴스가 있을 수 있습니다.

### 구현

구현은 송신 및 수신 스레드의 2개 스레드를 사용합니다. The sender runs at a fixed rate and dynamically
reduces the rates of the streams if the combined bandwidth is higher than the configured rate (`-r`) or the
physical link becomes saturated. This can be checked with `mavlink status`, see if `rate mult` is less than 1.

**Careful**: some of the data is accessed and modified from both threads, so when changing code or extend the
functionality, this needs to be take into account, in order to avoid race conditions and corrupt data.

### 예

전송 속도가 921600이고, 최대 전송 속도가 80kB/s인 ttyS1 직렬 포트에서 mavlink를 시작합니다.

```
mavlink start -d /dev/ttyS1 -b 921600 -m onboard -r 80000
```

UDP 포트 14556에서 mavlink를 시작하고, 50Hz로 HIGHRES_IMU 메시지를 활성화합니다.

```
mavlink start -u 14556 -r 1000000
mavlink stream -u 14556 -s HIGHRES_IMU -r 50
```

<a id="mavlink_usage"></a>

### 사용법

```
mavlink <command> [arguments...]
 Commands:
   start         Start a new instance
     [-d <val>]  Select Serial Device
                 values: <file:dev>, default: /dev/ttyS1
     [-b <val>]  Baudrate (can also be p:<param_name>)
                 default: 57600
     [-r <val>]  Maximum sending data rate in B/s (if 0, use baudrate / 20)
                 default: 0
     [-p]        Enable Broadcast
     [-u <val>]  Select UDP Network Port (local)
                 default: 14556
     [-o <val>]  Select UDP Network Port (remote)
                 default: 14550
     [-t <val>]  Partner IP (broadcasting can be enabled via -p flag)
                 default: 127.0.0.1
     [-m <val>]  Mode: sets default streams and rates
                 values: custom|camera|onboard|osd|magic|config|iridium|minimal|
                 extvision|extvisionmin|gimbal|uavionix, default: normal
     [-n <val>]  wifi/ethernet interface name
                 values: <interface_name>
     [-c <val>]  Multicast address (multicasting can be enabled via
                 MAV_{i}_BROADCAST param)
                 values: Multicast address in the range
                 [239.0.0.0,239.255.255.255]
     [-F <val>]  Sets the transmission frequency for iridium mode
                 default: 0.0
     [-f]        Enable message forwarding to other Mavlink instances
     [-w]        Wait to send, until first message received
     [-x]        Enable FTP
     [-z]        Force hardware flow control always on
     [-Z]        Force hardware flow control always off

   stop-all      Stop all instances

   stop          Stop a running instance
     [-u <val>]  Select Mavlink instance via local Network Port
     [-d <val>]  Select Mavlink instance via Serial Device
                 values: <file:dev>

   status        Print status for all instances
     [streams]   Print all enabled streams

   stream        Configure the sending rate of a stream for a running instance
     [-u <val>]  Select Mavlink instance via local Network Port
     [-d <val>]  Select Mavlink instance via Serial Device
                 values: <file:dev>
     -s <val>    Mavlink stream to configure
     -r <val>    Rate in Hz (0 = turn off, -1 = set to default)

   boot_complete Enable sending of messages. (Must be) called as last step in
                 startup script.
```

## uorb

Source: [systemcmds/uorb](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/uorb)

### 설명

uORB는 모듈 간의 통신에 사용되는 내부 pub-sub 메시징 시스템입니다.

### 구현

구현은 비동기식이며 잠금이 없습니다. 게시자는 구독자를 기다리지 않으며, 그 반대도 마찬가지입니다.
이것은 발행자와 구독자 사이에 별도의 버퍼를 가짐으로써 달성됩니다.

코드는 메모리 공간과 메시지 교환 대기 시간을 최소화하도록 최적화되었습니다.

Messages are defined in the `/msg` directory. 빌드 타임에 C/C++ 코드로 변환됩니다.

ORB_USE_PUBLISHER_RULES로 컴파일하면, uORB 게시 규칙이 있는 파일을 사용하여, 어떤 모듈이 어떤 주제를 게시할 수 있는 지 설정할 수 있습니다. 이것은 시스템 전체 재생에 사용됩니다.

### 예

주제 게시 비율을 모니터링합니다. Besides `top`, this is an important command for general system inspection:

```
uorb top
```

<a id="uorb_usage"></a>

### 사용법

```
uorb <command> [arguments...]
 Commands:
   status        Print topic statistics

   top           Monitor topic publication rates
     [-a]        print all instead of only currently publishing topics with
                 subscribers
     [-1]        run only once, then exit
     [<filter1> [<filter2>]] topic(s) to match (implies -a)
```
