# WiFi Телеметрійне радіо

WiFi telemetry enables MAVLink communication between a WiFi radio on a vehicle and a GCS.\
WiFi typically offers shorter range than a normal telemetry radio, but supports higher data rates, and makes it easier to support FPV/video feeds.
Зазвичай для транспортного засобу потрібен лише один радіоприймач (з умовою, що наземна станція вже має WiFi).

PX4 підтримує телеметрію через UDP та Wifi. Він транслює сигнал серцебиття на порт 14550 за адресою 255.255.255.255, доки не отримає перший сигнал серцебиття від земної станції керування, після чого він буде надсилати дані лише на цю земну станцію керування.

Сумісні модулі телеметрії WiFi включають в себе:

- [ESP8266 WiFi Module](../telemetry/esp8266_wifi_module.md)
- [ESP32 WiFi Module](../telemetry/esp32_wifi_module.md)
- [3DR Telemetry Wifi](../telemetry/3dr_telemetry_wifi.md) (Discontinued)
