# Silicon Errata

This page lists known issues with silicon (hardware) errata of 3rd-party parts (micro controller, sensors, etc.) used on the Pixhawk board series. Залежно від типу помилки кремнію, їх неможливо виправити програмним забезпеченням і можуть накладати певні обмеження.

## FMUv2 / Pixhawk Silicon Errata

### STM32F427VIT6 (errata)

Flash Bank 2 та ексклюзивний пристрій USB з повною швидкістю.

Ревізії Silicon до версії 2 (ревізія 3 є першою, яка не зазнає впливу) можуть спричинити помилки/пошкодження даних під час доступу до 2-го банку флеш-пам’яті, коли є активність на PA12, який є однією з ліній даних USB. Не існує обхідного шляху / програмного виправлення для цього, крім як не використовувати флеш-банк #2.
Since USB is needed to program the device, Pixhawk revisions built with silicon revisions < rev 3 can only use up to 1MB of the 2MB flash of the microprocessor.

:::tip
The errata is fixed in later versions, but this may not be detected if you are using an older bootloader.
See [Firmware > FMUv2 Bootloader Update](../config/firmware.md#bootloader) for more information.
:::

## FMUv1 / Pixhawk Silicon Errata

Немає відомих проблем.
