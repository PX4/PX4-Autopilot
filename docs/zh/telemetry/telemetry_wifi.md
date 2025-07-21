# WiFi 数传电台

WiFi telemetry enables MAVLink communication between a WiFi radio on a vehicle and a GCS.\
WiFi typically offers shorter range than a normal telemetry radio, but supports higher data rates, and makes it easier to support FPV/video feeds.
Usually only a single radio unit for the vehicle is needed (assuming the ground station already has WiFi).

PX4 supports telemetry via UDP and Wifi. It broadcasts a heartbeat to port 14550 on 255.255.255.255 until it receives the first heartbeat from a ground control station, at which point it will only send data to this ground control station.

兼容的 WiFi 数传模块有：

- [ESP8266 WiFi Module](../telemetry/esp8266_wifi_module.md)
- [ESP32 WiFi Module](../telemetry/esp32_wifi_module.md)
- [3DR Telemetry Wifi](../telemetry/3dr_telemetry_wifi.md) (Discontinued)
