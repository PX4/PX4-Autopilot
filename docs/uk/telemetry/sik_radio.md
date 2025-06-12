# SiK Radio

[SiK radio](https://github.com/LorenzMeier/SiK) is a collection of firmware and tools for telemetry radios.

PX4 is protocol-compatible with radios that use _SiK_.
SiK Radios often come with appropriate connectors/cables allowing them to be directly connected to [Pixhawk Series](../flight_controller/pixhawk_series.md) controllers
(in some cases you may need to obtain an appropriate cable/connector).
Зазвичай вам знадобиться пара пристроїв: один для транспортного засобу та один для наземної станції.

Апаратне забезпечення для радіо SiK можна придбати у різних виробників/магазинів у варіантах, які підтримують різні діапазони та форм-фактори.

![SiK Radio](../../assets/hardware/telemetry/holybro_sik_radio.jpg)

## Постачальники

- [RFD900 Telemetry Radio](../telemetry/rfd900_telemetry.md)
- [Holybro Telemetry Radio](../telemetry/holybro_sik_radio.md)
- <del>_HKPilot Телеметрійне радіо_</del> (Припинено)
- <del>_3DR телеметрія радіо_</del> (припинено)

## Налаштування/Конфігурація

Підземне радіо, орієнтоване на станції підключено через USB (по суті plug-n-play).

The vehicle-based radio is connected to the flight-controller's `TELEM1` port, and typically requires no further configuration.

## Оновлення прошивки

Hardware sourced from most [vendors](#vendors) should come pre-configured with the latest firmware.
Можливо, вам знадобиться оновити старе обладнання новим мікропрограмним забезпеченням, наприклад, щоб отримати підтримку MAVLink 2.

You can update the radio firmware using _QGroundControl_: [QGroundControl User Guide > Loading Firmware](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/firmware.html).

## Розширене налаштування/конфігурація

The Development section has [additional information](../data_links/sik_radio.md) about building firmware and AT-command based configuration.
Це не повинно вимагатися особам, які не є розробниками.
