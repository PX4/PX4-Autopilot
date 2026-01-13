# CAN (DroneCAN & Cyphal)

[Controller Area Network (CAN)](https://en.wikipedia.org/wiki/CAN_bus) is a robust wired network that allows drone components such as flight controller, ESCs, sensors, and other peripherals, to communicate with each other.

It is particularly recommended on larger vehicles.

## Загальний огляд

CAN it is designed to be democratic and uses differential signaling.
For this reason it is very robust even over longer cable lengths (on large vehicles), and avoids a single point of failure.
CAN також дозволяє отримання зворотного зв'язку від периферійних пристроїв та зручне оновлення прошивки через шину.

PX4 підтримує два програмні протоколи для взаємодії з пристроями CAN:

- [DroneCAN](../dronecan/README.md): PX4 рекомендує це для більшості типових налаштувань.
  Воно має гарну підтримку в PX4, є вже досить зрілим продуктом з широкою підтримкою периферійних пристроїв та багаторічними тестами.
- [Cyphal](https://opencyphal.org): PX4 support is a "work in progress".
  Cyphal - набагато новіший протокол, який надає більше гнучкості та конфігураційних можливостей, особливо на більших і складніших транспортних засобах.
  Він ще не отримав значного впровадження.

:::info
Обидва DroneCAN і Cyphal походять з раніше проекту під назвою UAVCAN.
У 2022 році проект розділився на дві частини: оригінальна версія UAVCAN (UAVCAN v0) була перейменована в DroneCAN, а нова версія UAVCAN v1 отримала назву Cyphal.
Відмінності між цими двома протоколами описані в [Cyphal vs. DroneCAN](https://forum.opencyphal.org/t/cyphal-vs-dronecan/1814).
:::

У PX4 немає підтримки інших програмних протоколів CAN для безпілотних літальних апаратів, таких як KDECAN (на момент написання).

## Підключення

Проводка для мереж CAN однакова як для DroneCAN, так і для Cyphal/CAN (фактично, для всіх мереж CAN).

Devices within a network are connected in a _daisy-chain_ in any order (this differs from UARTs peripherals, where you attach just one component per port).

:::warning
Don't connect each CAN peripheral to a separate CAN port!
Unlike UARTs, CAN peripherals are designed to be daisy chained, with additional ports such as `CAN2` used for [redundancy](redundancy).
:::

На обох кінцях ланцюга між двома лініями передачі даних слід під’єднати термінальний резистор 120 Ом.
Польотні контролери та деякі модулі GNSS мають вбудовані резистори завершення для зручності, тому їх слід розміщувати на протилежних кінцях ланцюга.
В іншому випадку, ви можете використовувати резистор завершення, наприклад, [цей від Zubax Robotics](https://shop.zubax.com/products/uavcan-micro-termination-plug?variant=6007985111069), або припаяти його самостійно, якщо у вас є доступ до затискача JST-GH.

Наступна діаграма показує приклад шини CAN, що з'єднує автопілот з 4 контролерами ESC CAN та GNSS.
It includes a redundant bus connected to `CAN 2`.

![CAN Wiring](../../assets/can/uavcan_wiring.svg)

На схемі не показано електропроводку.
Для підтвердження, чи компоненти потребують окремого живлення, чи можуть бути живлені від самої шини CAN, звертайтеся до інструкцій виробника.

:::info
For more information, see [Cyphal/CAN device interconnection](https://wiki.zubax.com/public/cyphal/CyphalCAN-device-interconnection?pageId=2195476) (kb.zubax.com).
Хоча стаття написана з урахуванням протоколу Cyphal, вона однаково стосується апаратного забезпечення DroneCAN і будь-яких інших налаштувань CAN.
Для більш складних сценаріїв зверніться до розділу [Про топологію та термінацію шини CAN](https://forum.opencyphal.org/t/on-can-bus-topology-and-termination/1685).
:::

### З’єднання

Пристрої CAN, сумісні зі стандартом Pixhawk, використовують 4-контактні роз’єми JST-GH для CAN.
Для підключення в ланцюг використовуються два роз'єми: для введення і виведення (крім контролерів польоту та деяких пристроїв GNSS з вбудованим завершенням, які мають лише один роз'єм JST-GH).

Інші пристрої (які несумісні з Pixhawk) можуть використовувати інші роз'єми.
Однак, якщо прошивка пристрою підтримує DroneCAN або Cyphal, його можна використовувати.

### Резервування

DroneCAN та Cyphal/CAN підтримують використання другого (резервного) інтерфейсу CAN.
Це абсолютно необов'язково, але збільшує надійність підключення.

Pixhawk flight controllers come with 2 CAN interfaces; if your peripherals support 2 CAN interfaces as well, it is recommended to wire both up for increased safety.

### Flight Controllers with Multiple CAN Ports

[Flight Controllers](../flight_controller/index.md) may have up to three independent CAN ports, such as `CAN1`, `CAN2`, `CAN3` (neither DroneCAN or Cyphal support more than three).
Note that you can't have both DroneCAN and Cyphal running on PX4 at the same time.

:::tip
You only _need_ one CAN port to support an arbitrary number of CAN devices using a particular CAN protocol.
Don't connect each CAN peripheral to a separate CAN port!
:::

Generally you'll daisy all CAN peripherals off a single port, and if there is more than one CAN port, use the second one for [redundancy](redundancy).
If three are three ports, you might use the remaining network for devices that support another CAN protocol.

The documentation for your flight controller should indicate which ports are supported/enabled.
At runtime you can check what DroneCAN ports are enabled and their status using the following command on the [MAVLink Shell](../debug/mavlink_shell.md) (or some other console):

```sh
uavcan status
```

Note that you can also check the number of supported CAN interfaces for a board by searching for `CONFIG_BOARD_UAVCAN_INTERFACES` in its [default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v6xrt/default.px4board#) configuration file.

## Прошивка

Периферійні пристрої CAN можуть працювати на власній пропрієтарній або відкритій прошивці (перевірте посібники виробника, щоб підтвердити потрібну настройку).

PX4 може бути зібраний для запуску як прошивка DroneCAN з відкритим вихідним кодом на підтримуваному апаратному забезпеченні CAN.
Для отримання додаткової інформації див
[PX4 DroneCAN Firmware](../dronecan/px4_cannode_fw.md).

## Підтримка та конфігурація

[Налаштування та налаштування DroneCAN](../dronecan/index.md)

[PX4 DroneCAN Firmware](../dronecan/px4_cannode_fw.md)

## Відео

### DroneCAN

Вступ до DroneCAN (UAVCANv0) та практичні приклади з установкою в QGroundControl:

<lite-youtube videoid="IZMTq9fTiOM" title="Intro to DroneCAN (UAVCANv0) and practical example with setup in QGroundControl"/>

### Cyphal

UAVCAN v1 for drones (Cyphal) — PX4 Developer Summit Virtual 2020

<lite-youtube videoid="6Bvtn_g8liU" title="UAVCAN v1 for drones — PX4 Developer Summit Virtual 2020"/>

---

Getting started using UAVCAN v1 with PX4 on the NXP UAVCAN Board — PX4 Developer Summit Virtual 2020

<lite-youtube videoid="MwdHwjaXYKs" title="Getting started using UAVCAN v1 with PX4 on the NXP UAVCAN Board"/>

---

UAVCAN: дуже надійний протокол публікації-підписки для внутрішньоавтомобільних мереж у реальному часі — PX4 Developer Summit Virtual 2019

<lite-youtube videoid="MBtROivYPik" title="UAVCAN: a highly dependable publish-subscribe protocol for hard ..."/>
