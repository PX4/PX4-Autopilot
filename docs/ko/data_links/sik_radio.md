# SiK 라디오 통합

[SiK radio](https://github.com/LorenzMeier/SiK) is a collection of firmware and tools for telemetry radios.

Information about _using_ SiK Radio can be found in [Peripheral Hardware > Telemetry > SiK Radio](../telemetry/sik_radio.md)

아래의 "개발자" 정보는 소스에서 SiK 펌웨어를 빌드하고 AT 명령을 사용하여 구성하는 방법에 대하여 설명합니다.

## 지원 라디오 하드웨어

SiK 저장소에는 다음 텔레메트리(2020-02-25)들의 부트로더와 펌웨어를 제공합니다.

- HopeRF HM-TRP
- HopeRF RF50-DEMO
- RFD900
- RFD900a
- RFD900p
- RFD900pe
- RFD900u
- RFD900ue

:::info
The SiK repository does not currently firmware for RFD900x or RFD900ux telemetry radios.
이러한 라디오의 펌웨어를 업데이트하려면(예: MAVLink v2.0을 지원하기 위해) 다음 프로세스를 참고하십시오.

1. Download the appropriate firmware from the [RFDesign website](https://files.rfdesign.com.au/firmware/).
2. On a Windows PC, download and install [RFD Modem Tools](https://files.rfdesign.com.au/tools/).
3. Use the RFD Modem Tools GUI to upload the firmware to your RFD900x or RFD900ux telemetry radio.

:::

## 빌드 방법

You will need to install the required 8051 compiler, as this is not included in the default PX4 Build toolchain.

### Mac OS

Install the toolchain:

```sh
brew install sdcc
```

Build the image for the standard SiK Radio / 3DR Radio:

```sh
git clone https://github.com/LorenzMeier/SiK.git
cd SiK/Firmware
make install
```

Upload it to the radio \(**change the serial port name**\):

```
라디오에 업로드 \(<0>직렬 포트 이름 변경</0>\):
```

## 설정 방법

The radio supports AT commands for configuration.

```sh
screen /dev/tty.usbserial-CHANGETHIS 57600 8N1
```

Then start command mode:

:::info
DO NOT TYPE ANYTHING ONE SECOND BEFORE AND AFTER
:::

```sh
+++
```

List the current settings:

```sh
ATI5
```

Then set the net ID, write settings and reboot radio:

```sh
ATS3=55
AT&W
ATZ
```

:::info
You might have to power-cycle the radio to connect it to the 2nd radio.
:::
