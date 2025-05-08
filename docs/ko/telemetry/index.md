# 텔레메트리 라디오/모뎀

Telemetry Radios can (optionally) be used to provide a wireless MAVLink connection between a ground control station like _QGroundControl_ and a vehicle running PX4. 비행중인 기체의 매개변수 변경, 실시간 텔레메트로 통신, 임무 변경 등의 작업을 수행할 수 있습니다.

PX4는 다양한 텔레메트리 라디오 타입을 지원합니다:

- [SiK Radio](../telemetry/sik_radio.md) based firmware (more generally, any radio with a UART interface should work).
  - [RFD900 Telemetry Radio](../telemetry/rfd900_telemetry.md)
  - [HolyBro SiK Telemetry Radio](../telemetry/holybro_sik_radio.md)
  - <del>_HKPilot Telemetry Radio_</del> (Discontinued)
  - <del>_3DR Telemetry Radio_</del> (Discontinued)
- [Telemetry Wifi](../telemetry/telemetry_wifi.md)
- [J.Fi Wireless Telemetry Module](../telemetry/jfi_telemetry.md)
- [Microhard Serial Telemetry Radio](../telemetry/microhard_serial.md)
  - [ARK Electron Microhard Serial Telemetry Radio](../telemetry/ark_microhard_serial.md)
  - [Holybro Microhard P900 Telemetry Radio](../telemetry/holybro_microhard_p900_radio.md)
- CUAV Serial Telemetry Radio
  - [CUAV P8 Telemetry Radio](../telemetry/cuav_p8_radio.md)
- XBee Serial Telemetry Radio
  - <del>[HolyBro XBP9X Telemetry Radio](../telemetry/holybro_xbp9x_radio.md)</del> (Discontinued)

PX4 is protocol compatible with [SiK Radio](../telemetry/sik_radio.md) and will generally work out of the box (though you may need to change/use an appropriate connector).

WiFi 원격 측정은 단거리에서 데이터 속도가 빠르며, FPV/비디오 피드를 보다 쉽게 지원할 수 있습니다.
WiFi의 이점은 차량용 라디오 장치 하나만 구입하면되는 점입니다 (지상국에 이미 WiFi가 있다고 가정).

:::info
PX4 does not support connecting an LTE USB module to the flight controller (and sending MAVLink traffic via the Internet).
보조 컴퓨터로 LTE 모듈을 연결하여 보조 컴퓨터에서 비행 제어 장치로 들어가는 MAVLink 데이터 흐름을 통제 가능합니다.
For more information see: [Companion Computer Peripherals > Data Telephony](../companion_computer/companion_computer_peripherals.md#data-telephony-lte).
:::

## Allowed Frequency Bands

Radio bands allowed for use with drones differ between continents, regions, countries, and even states.
You should select a telemetry radio that uses a frequency range that is allowed in the areas where you plan on using the drone.

Low power [SiK radios](../telemetry/sik_radio.md), such as the [Holybro Telemetry Radio](../telemetry/holybro_sik_radio.md), are often available in 915 MHz and 433 MHz variants.
While you should check applicable laws in your country/state, broadly speaking 915 MHz can be used in the US, while 433 MHz can be used in EU, Africa, Oceania, and most of Asia.
