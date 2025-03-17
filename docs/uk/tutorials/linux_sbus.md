# Підключення RC-приймача до автопілота на базі Linux PX4

This topic shows how to setup a PX4 Linux-based autopilot to connect and use a [supported RC receiver](../getting_started/rc_transmitter_receiver.md) on any serial port.

Для типів RC, крім S.Bus, ви можете просто під'єднати приймач безпосередньо до серійних портів або до USB через USB до TTY серійного кабелю (наприклад, PL2302 USB в Serial TTL)

:::info
For an S.Bus receiver (or encoder - e.g. from Futaba, RadioLink, etc.) you will usually need to connect the receiver and device via a [signal inverter circuit](#signal_inverter_circuit), but otherwise the setup is the same.
:::

Then [Start the PX4 RC Driver](#start_driver) on the device, as shown below.

<a id="start_driver"></a>

## Запуск драйвера

To start the RC driver on a particular UART (e.g. in this case `/dev/ttyS2`):

```sh
rc_input start -d /dev/ttyS2
```

For other driver usage information see: [rc_input](../modules/modules_driver.md#rc-input).

<a id="signal_inverter_circuit"></a>

## Схема інвертування сигналу (лише для S.Bus)

S.Bus is an _inverted_ UART communication signal.

Хоча деякі серійні порти / контролери польоту можуть читати інвертований сигнал UART, більшість вимагає схеми інвертування сигналу між приймачем та серійним портом для деінвертації сигналу.

:::tip
This circuit is also required to read S.Bus remote control signals through the serial port or USB-to-TTY serial converter.
:::

У цьому розділі показано, як створити відповідну схему.

### Необхідні компоненти

- 1x NPN транзистор (наприклад, NPN S9014 TO92)
- 1x 10K резистор
- 1x 1K резистор

:::info
Any type/model of transistor can be used because the current drain is very low.
:::

### Схема Діаграми/Підключення

Підключіть компоненти, як описано нижче (і показано на схемі):

- S.Bus сигнал &rarr; 1K резистор &rarr; база NPN транзистора
- NPN транзистора емітер &rarr; GND
- 3.3VCC &rarr; резистор 10K &rarr; колектор NPN транзистора &rarr; rxd USB-to-TTY
- 5.0VCC &rarr; S.Bus VCC
- GND &rarr; S.Bus GND

![Signal inverter circuit diagram](../../assets/sbus/driver_sbus_signal_inverter_circuit_diagram.png)

На зображенні нижче показано підключення на дошці для макету.

![Signal inverter breadboard](../../assets/sbus/driver_sbus_signal_inverter_breadboard.png)
