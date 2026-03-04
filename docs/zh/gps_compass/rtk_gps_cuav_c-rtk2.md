# CUAV C-RTK2 GNSS Module (RTK/PPK)

The [CUAV C-RTK2 receiver](https://www.cuav.net/en/c_rtk_9ps/) is a high-performance PPK/RTK positioning module created by CUAV for professional applications such as drone aerial surveying and mapping.
It has a high-precision IMU and positioning module, and can reduce the number of required [control points](https://www.youtube.com/watch?v=3k7v5aXyuKQ) by more than to 80%.
In addition to surveying/mapping, it is suitable for many other use-cases, including: agricultural plant protection and drone swarms.

![C-RTK2](../../assets/hardware/gps/cuav_rtk2/c-rtk2.png)

## Other characteristics

- High-performance H7 processor
- High precision industrial grade IMU
- Support RTK and save RAW raw data (PPK) at the same time
- Multi-satellite and multi-frequency receivers
- UAVCAN/Dronecan protocol
- Support hotshoe and shutter trigger
- HS_USB and U disk mode

## 购买渠道

- [CUAV Store](https://store.cuav.net/shop/c-rtk-2/)
- [CUAV aliexpress](https://pt.aliexpress.com/item/1005003754165772.html?spm=a2g0o.store_pc_groupList.8148356.13.2f893550i0NE4o)

# 总览

- RTK Receiver
  - ZED-F9P
- Receiving channel
  - 184
- Main FMU Processor
  - STM32H743VIH6(2M flash、1M RAM）
- 内置传感器：
  - Accelerometer/Gyroscope: ICM20689
- Magnetometer: RM3100
  - Barometer: ICP10111
- TF card expansion
  - 32G(MAX)
- PPK(post processed kinematic)
  - support
- RTK(Real - time kinematic)
  - support
- GNSS Bands
  - GPS:L1C/A,L2C
  - GLONASS:L1OF,L2OF
  - GALILEO: E1B/C E5b
  - Beidou:B1I B2I
- Enhanced system
  - QZSS:L1C/A,L2C,L1S
  - SBAS:L1C/A
- Number of concurrent GNSS
  - 4(GPS、GLONASS、GALILEO、Beidou）
- Nav. update rate
  - RTK Up to 20HZ
  - RAW Up to 25hz
  - default：5hz
- Convergence time
  - RTK < 10 sec
- Position accuracy（RMS)
  - RTK:0.01m+1ppm(level);0.02m+1ppm(vertical)
  - GPS:1.5m(level)
- Acquisition
  - Cold starts 24 s
  - Aided starts 2 s
  - Reacquisition 2 s
- Sensitivity
  - Tracking & Nav –167 dBm
  - Cold starts –148 dBm
  - Hot starts –157 dBm
  - Reacquisition –160 dBm
- Anti-spoofng
  - Advanced anti-spoofng algorithms
- Protocols
  - NMEA
  - UBX binary
  - RTCM version 3.x
- Time pulse
  - 0.25Hz~10Hz(Configurable)
- Anti-jamming
  - Active CW detection and removal Onboard band pass ﬂter
- Support flight control type
  - Compatible with flight controllers running PX4 firmware
- interface
  - 1 Hotshoe
  - 1 shutter in
  - 1 sutter out
  - 1 Type(HS_USB)
  - 1 F9P USB
  - 1 F9P UART
  - 1 Antenna(mmcx)
- Supply voltage
  - 4.5~6v
- Operating temperature
  - -20~85℃
- Size
  - 56x33x16.5mm
- 重量
  - 39g

## 配置

[CUAV Docs](https://doc.cuav.net/gps/c-rtk2/en/quick-start-c-rtk2.html)

## 针脚定义

![C-RTK2](../../assets/hardware/gps/cuav_rtk2/c-rtk2_pinouts1.jpg)

![C-RTK2](../../assets/hardware/gps/cuav_rtk2/c-rtk2_pinouts0.jpg)

![C-RTK2](../../assets/hardware/gps/cuav_rtk2/c-rtk2_pinouts2.jpg)

## More information

[CUAV Docs](https://doc.cuav.net/gps/c-rtk-series/en/c-rtk-9ps/)
