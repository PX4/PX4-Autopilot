# WiFi 텔레메트리 라디오

WiFi telemetry enables MAVLink communication between a WiFi radio on a vehicle and a GCS.\
WiFi typically offers shorter range than a normal telemetry radio, but supports higher data rates, and makes it easier to support FPV/video feeds.
일반적으로 차량용 라디오 장치 하나만 필요합니다 (지상국에 이미 WiFi가 있다고 가정).

PX4는 UDP와 WiFi를 통한 텔레메트리를 지원합니다. 지상국에서 첫 번째 heartbeat를 수신 할 때까지 하트 비트를 255.255.255.255의 포트 14550으로 heartbeat를 브로드캐스팅합니다. 이 시점에서는 데이터는 지상국에만 전송됩니다.

호환 가능한 WiFi 텔레메트리 모듈은 아래와 같습니다.

- [ESP8266 WiFi Module](../telemetry/esp8266_wifi_module.md)
- [ESP32 WiFi Module](../telemetry/esp32_wifi_module.md)
- [3DR Telemetry Wifi](../telemetry/3dr_telemetry_wifi.md) (Discontinued)
