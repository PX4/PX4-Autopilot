# Телеметричні радіо/модеми

Telemetry Radios can (optionally) be used to provide a wireless MAVLink connection between a ground control station like _QGroundControl_ and a vehicle running PX4. Це дозволяє налаштовувати параметри, коли транспортний засіб в польоті, перевіряти телеметрію в режимі реального часу, змінювати політне завдання на льоту тощо.

PX4 підтримує ряд типів телеметрійних радіозон:

- [SiK Radio](../telemetry/sik_radio.md) based firmware (more generally, any radio with a UART interface should work).
  - [RFD900 Telemetry Radio](../telemetry/rfd900_telemetry.md)
  - [HolyBro SiK Telemetry Radio](../telemetry/holybro_sik_radio.md)
  - <del>_HKPilot Телеметрійне радіо_</del> (Припинено)
  - <del>_3DR телеметрія радіо_</del> (припинено)
- [Telemetry Wifi](../telemetry/telemetry_wifi.md)
- [J.Fi Wireless Telemetry Module](../telemetry/jfi_telemetry.md)
- [Microhard Serial Telemetry Radio](../telemetry/microhard_serial.md)
  - [ARK Electron Microhard Serial Telemetry Radio](../telemetry/ark_microhard_serial.md)
  - [Holybro Microhard P900 Telemetry Radio](../telemetry/holybro_microhard_p900_radio.md)
- CUAV Серійне телеметрійне радіо
  - [CUAV P8 Telemetry Radio](../telemetry/cuav_p8_radio.md)
- XBee Серійне Телеметрійне Радіо
  - <del>[HolyBro XBP9X Telemetry Radio](../telemetry/holybro_xbp9x_radio.md)</del> (Discontinued)

PX4 is protocol compatible with [SiK Radio](../telemetry/sik_radio.md) and will generally work out of the box (though you may need to change/use an appropriate connector).

Телеметрія по WiFi зазвичай має менший радіус дії, вищі швидкості передачі даних і спрощує підтримку FPV/відеопотоків.
Однією з переваг WiFi-радіо є те, що вам потрібно придбати лише один радіоприймач для вашого транспортного засобу (за умови, що земна станція вже має WiFi).

:::info
PX4 does not support connecting an LTE USB module to the flight controller (and sending MAVLink traffic via the Internet).
Однак ви можете підключити модуль LTE до компаньйонного комп'ютера і використовувати його для маршрутизації трафіку MAVLink від контролера польоту.
For more information see: [Companion Computer Peripherals > Data Telephony](../companion_computer/companion_computer_peripherals.md#data-telephony-lte).
:::

## Дозволені частоти

Діапазони радіочастот, дозволені для використання з дронами, відрізняються між континентами, регіонами, країнами, а навіть штатами.Вам слід вибрати телеметричне радіо, яке використовує діапазон частот, дозволений у тих областях, де ви плануєте використовувати дрона.

Low power [SiK radios](../telemetry/sik_radio.md), such as the [Holybro Telemetry Radio](../telemetry/holybro_sik_radio.md), are often available in 915 MHz and 433 MHz variants.Хоча вам слід перевірити діючі закони у своїй країні/штаті, загалом кажучи, 915 МГц можна використовувати у США, тоді як 433 МГц можна використовувати в ЄС, Африці, Океанії та більшості Азії.
