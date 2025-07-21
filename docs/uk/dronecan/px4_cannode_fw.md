# Прошивка PX4 DroneCAN

PX4 може працювати як прошивка на багатьох периферійних пристроях DroneCAN. Є кілька переваг цього:

- PX4 has built-in drivers for a [wide range](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers) of sensors and peripherals components.
- PX4 має міцну реалізацію драйвера DroneCAN, яка пройшла кілька років полевих випробувань.
- PX4 постійно розвивається. Ви регулярно отримуєте доступ до останніх покращень.
- Код оцінки та керування PX4 дозволяє легко створювати "розумні" канноди, такі як інтегровані модулі AHRS.
- Прошивка повністю відкритого коду (PX4 має ліцензію BSD).

## Збірка прошивки

Follow the [PX4 building docs](../dev_setup/building_px4.md) just as you would to build firmware for a flight controller. Device build configurations are stored [here](https://github.com/PX4/PX4-Autopilot/tree/main/boards). After installing the [PX4 toolchain](../dev_setup/dev_env.md), clone the sources and build. For example, to build for the [Ark Flow](ark_flow.md) target:

```sh
git clone --recursive https://github.com/PX4/PX4-Autopilot
cd PX4-Autopilot
make ark_can-flow_default
```

This will create an output in **build/ark_can-flow_default** named **XX-X.X.XXXXXXXX.uavcan.bin**. Follow the instructions at [DroneCAN firmware update](index.md#firmware-update) to flash the firmware.

## Інформація для розробників

Цей розділ містить інформацію, яка є актуальною для розробників, які хочуть додати підтримку нового апаратного забезпечення DroneCAN до автопілота PX4.

### Встановлення завантажувача DroneCAN

:::warning
DroneCAN devices typically ship with a bootloader pre-installed.
Не слід дотримуватися інструкцій у цьому розділі, якщо ви не розробляєте пристрої DroneCAN,
або (випадково) пошкодили / видалили свій завантажувач.
:::

Проект PX4 включає стандартний завантажувач DroneCAN для пристроїв STM32.

Завантажувач займає перших 8-16 КБ флеш-пам'яті і є першим кодом, що виконується при увімкненні живлення.
Typically the bootloader performs low-level device initialization, automatically determines the CAN
bus baud rate, acts as a [DroneCAN dynamic node ID client](index.md#node-id-allocation) to obtain a unique node ID, and waits for confirmation from the flight controller before proceeding with application boot.

Цей процес забезпечує можливість відновлення пристрою DroneCAN від недійсного або пошкодженого програмного забезпечення додатка без втручання користувача, а також дозволяє автоматичне оновлення програмного забезпечення.

Build the bootloader firmware by specifying the same peripheral target with the `canbootloader` build configuration instead of the `default` configuration.

For example, to build for the [Ark Flow](ark_flow.md) target:

```sh
git clone --recursive https://github.com/PX4/PX4-Autopilot
cd PX4-Autopilot
make ark_can-flow_canbootloader
```

The binary can then be flashed to the microcontroller using your favorite SWD/JTAG debugger, such as the [Black Magic Probe](https://black-magic.org/index.html), [ST-Link](https://www.st.com/en/development-tools/st-link-v2.html), or [Segger JLink](https://www.segger.com/products/debug-probes/j-link/).

### Налаштування прошивки

Переважно, вбудоване програмне забезпечення периферійних пристроїв працює так само, як і версії програмного забезпечення контролера польоту.
However, most modules are disabled - only the sensor drivers, DroneCAN driver, and internal infrastructure (uORB, etc.) are enabled.

DroneCAN communication is handled by the [uavcannode](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/uavcannode) module.
Цей драйвер відповідає за комунікацію з боку виробника - він отримує дані сенсорів/виконавчих пристроїв з uORB, серіалізує їх за допомогою бібліотек DroneCAN та публікує їх по CAN.
In the future, this will likely be merged with the [uavcan](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/uavcan) module which handles flight controller side (consumer side) drivers, which receive/deserialize data from the CAN bus and publish them over uORB.

The build system also produces firmware binaries designed to be flashed through a DroneCAN bootloader via [PX4's DroneCAN flashing support] or the DroneCAN GUI, in addition to the standard raw binary, ELF, and `.px4` firmware files.
