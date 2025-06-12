# Pixracer 배선 개요

:::warning
PX4 does not manufacture this (or any) autopilot.
Contact the [manufacturer](https://store.mrobotics.io/) for hardware support or compliance issues.
:::

:::warning
Under construction
:::

This quick start guide shows how to power the [Pixracer](../flight_controller/pixracer.md) flight controller and connect its most important peripherals.

<img src="../../assets/flight_controller/pixracer/pixracer_hero_grey.jpg" width="300px" title="pixracer + 8266 grey" />

## 무선 가이드/조립 방법

![Grau pixracer double](../../assets/flight_controller/pixracer/grau_pixracer_double.jpg)

### 기본 설정

![Grau setup pixracer top](../../assets/flight_controller/pixracer/grau_setup_pixracer_top.jpg)

![Grau setup pixracer bottom](../../assets/flight_controller/pixracer/grau_setup_pixracer_bottom.jpg)

### Radio/Remote Control

A remote control (RC) radio system is required if you want to _manually_ control your vehicle (PX4 does not require a radio system for autonomous flight modes).

You will need to [select a compatible transmitter/receiver](../getting_started/rc_transmitter_receiver.md) and then _bind_ them so that they communicate (read the instructions that come with your specific transmitter/receiver).

아래의 지침은 다양한 유형의 수신기를 연결하는 방법을 설명합니다:

- FrSky 수신기는 표시된 포트를 통해 연결되며 제공된 I/O 커넥터를 사용할 수 있습니다.

  ![Grau b Pixracer FrSkyS.Port Connection](../../assets/flight_controller/pixracer/grau_b_pixracer_frskys.port_connection.jpg)

  ![Pixracer FrSkyS.Port Connection](../../assets/flight_controller/pixracer/pixracer_FrSkyTelemetry.jpg)

- PPM-SUM and S.BUS receivers connect to the **RCIN** port.

  ![Radio Connection](../../assets/flight_controller/pixracer/grau_setup_pixracer_radio.jpg)

- PPM and PWM receivers that have an _individual wire for each channel_ must connect to the **RCIN** port _via a PPM encoder_ [like this one](http://www.getfpv.com/radios/radio-accessories/holybro-ppm-encoder-module.html) (PPM-Sum receivers use a single signal wire for all channels).

### 전원 모듈 (ACSP4)

![Grau ACSP4 2 roh](../../assets/flight_controller/pixracer/grau_acsp4_2_roh.jpg)

### 외부 텔레메트리

Pixracer has inbuilt WiFi, but also supports telemetry via external Wi-Fi or radio telemetry modules connected to the `TELEM1` or `TELEM2` ports.
무선 다이어그램은 아래의 그림과 같습니다.

![Pixracer external telemtry options](../../assets/flight_controller/pixracer/pixracer_top_telemetry.jpg)

:::info
The `TELEM2` port must be configured as a second MAVLink instance using the [MAV_2_CONFIG](../advanced_config/parameter_reference.md#MAV_2_CONFIG) parameter.
For more information see [MAVLink Peripherals > MAVLink Instances](../peripherals/mavlink_peripherals.md#mavlink-instances) (and [Serial Port Configuration](../peripherals/serial_configuration.md)).
:::
