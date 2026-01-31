# Радіоінтеграція SiK

[SiK radio](https://github.com/LorenzMeier/SiK) is a collection of firmware and tools for telemetry radios.

Information about _using_ SiK Radio can be found in [Peripheral Hardware > Telemetry > SiK Radio](../telemetry/sik_radio.md)

Наведена нижче інформація ("розробник") пояснює, як створити мікропрограму SiK із джерела та налаштувати її за допомогою AT-команд.

## Підтримуване радіообладнання

Репозиторій SiK містить завантажувачі та мікропрограми для наступних радіостанцій телеметрії (2020-02-25):

- HopeRF HM-TRP
- HopeRF RF50-DEMO
- RFD900
- RFD900a
- RFD900p
- RFD900pe
- RFD900u
- RFD900ue

:::info
The SiK repository does not currently firmware for RFD900x or RFD900ux telemetry radios.
Щоб оновити мікропрограму на цих радіостанціях (наприклад, щоб підтримувати MAVLink v2.0), пропонується такий процес:

1. Download the appropriate firmware from the [RFDesign website](https://files.rfdesign.com.au/firmware/).
2. On a Windows PC, download and install [RFD Modem Tools](https://files.rfdesign.com.au/tools/).
3. Використовуйте графічний інтерфейс RFD Modem Tools, щоб завантажити мікропрограму на телеметричну радіостанцію RFD900x або RFD900ux.

:::

## Інструкції з установки

Вам потрібно буде інсталювати потрібний компілятор 8051, оскільки він не включений у стандартний інструментарій збірки PX4.

### Mac OS

Встановити ланцюжок інструментів:

```sh
brew install sdcc
```

Створіть образ для стандартного SiK Radio / 3DR Radio:

```sh
git clone https://github.com/LorenzMeier/SiK.git
cd SiK/Firmware
make install
```

Upload it to the radio \(**change the serial port name**\):

```
tools/uploader.py --port /dev/tty.usbserial-CHANGETHIS dst/radio~hm_trp.ihx
```

## Інструкції з налаштування

Радіо підтримує AT-команди для налаштування.

```sh
screen /dev/tty.usbserial-CHANGETHIS 57600 8N1
```

Після цього почніть командний режим:

:::info
DO NOT TYPE ANYTHING ONE SECOND BEFORE AND AFTER
:::

```sh
+++
```

Перелічіть поточні налаштування:

```sh
ATI5
```

Потім встановіть net ID, запишіть налаштування та перезавантажте радіо:

```sh
ATS3=55
AT&W
ATZ
```

:::info
You might have to power-cycle the radio to connect it to the 2nd radio.
:::
