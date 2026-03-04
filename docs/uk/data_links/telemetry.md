# Інтеграція радіо/модему телеметрії

Telemetry Radios can (optionally) be used to provide a wireless MAVLink connection between a ground control station like _QGroundControl_ and a vehicle running PX4.
Цей розділ містить теми про розширене використання підтримуваних радіомодулів і інтеграцію нових телеметрічних систем в PX4.

:::tip
[Peripheral Hardware > Telemetry Radios](../telemetry/index.md) contains information about telemetry radio systems already supported by PX4.
This includes radios that use the _SiK Radio_ firmware and _3DR WiFi Telemetry Radios_.
:::

## Інтеграція телеметричних систем

PX4 дозволяє використовувати телеметрію на основі протоколу MAVLink через телеметрійний порт контролера польоту на основі Pixhawk.
При умові, що телеметрійне радіо підтримує MAVLink та надає інтерфейс UART з сумісними рівнями напруги/роз'ємом, подальша інтеграція не потрібна.

Телеметрійні системи, що використовують інший протокол, потребують більш глибокої інтеграції, можливо, як програмного (наприклад, драйвери пристрою), так і апаратного (роз'єми тощо).
While this has been done for specific cases (e.g. [FrSky Telemetry](../peripherals/frsky_telemetry.md) enables sending vehicle status to an RC controller via an FrSky receiver) providing general advice is difficult.
We recommend you start by [discussing with the development team](../contribute/support.md).
