# DShot ESCs

DShot is an alternative ESC protocol that has several advantages over [PWM](../peripherals/pwm_escs_and_servo.md) or [OneShot](../peripherals/oneshot.md):

- Зменшений час затримки.
- Підвищена надійність за допомогою контрольної суми.
- Не потрібна калібрування ESC, оскільки протокол використовує цифрове кодування.
- Зворотний зв'язок телеметрії доступний/підтримується на деяких ESC.
- Може змінювати напрямок обертання двигуна за допомогою команд при потребі (замість фізичного переміщення проводів / перепайки).
- Інші корисні команди підтримуються.

Ця тема показує, як підключити та налаштувати DShot ESC.

## Wiring/Connections {#wiring}

DShot ESC are wired the same way as [PWM ESCs](pwm_escs_and_servo.md).
Єдина відмінність полягає в тому, що їх можна підключити лише до FMU, і зазвичай лише до певного підмножини контактів.

:::info
You may want to check the actuator configuration screen to see what pins are available for DShot on your controller before wiring up!
:::

Pixhawk controllers with both an FMU and an IO board usually label them as `AUX` (FMU) and `MAIN` (IO), respectively.
These match the `PWM AUX` and `PWM MAIN` output tabs on the actuator configuration screen.
For these controllers connect the DShot ESC to the `AUX` port.

Controllers that don't have an IO board usually label the (single) output port as `MAIN`, and this is where you will connect your DShot ESC.
If the controller without IO has its own firmware, the actuator assignment will be to the matching `PWM MAIN` outputs.
However if the same firmware is used for hardware with/without the IO board, such as for the Pixhawk 4 and Pixhawk 4 Mini, then actuator assignment tab used is the same in both cases: `PWM AUX` (i.e. not matching the port label `MAIN` in the "mini" case).

## Налаштування

:::warning
Remove propellers before changing ESC configuration parameters!
:::

Enable DShot for your required outputs in the [Actuator Configuration](../config/actuators.md).

DShot comes with different speed options: _DShot150_, _DShot300_, _DShot600_ and _DShot1200_, where the number indicates the speed in kilo-bits/second.
Ви повинні встановити параметр на найвищу швидкість, підтримувану вашим ESC (згідно з його технічним описом).

Підключіть батарею та озбройте транспортний засіб.
РЕГБ повинні ініціалізуватися, а мотори повинні обертатися в правильних напрямках.

- If the motors do not spin in the correct direction (for the [selected airframe](../airframes/airframe_reference.md)) you can reverse them in the UI using the **Set Spin Direction** option (this option appears after you select DShot and assign motors).
  You can also reverse motors by sending an [ESC Command](#commands).

## ESC Commands {#commands}

Commands can be sent to the ESC via the [MAVLink shell](../debug/mavlink_shell.md).
See [here](../modules/modules_driver.md#dshot) for a full reference of the supported commands.

Найважливіші з них:

- Make a motor connected to to FMU output pin 1 beep (helps with identifying motors)

  ```sh
  dshot beep1 -m 1
  ```

- Retrieve ESC information (requires telemetry, see below):

  ```sh
  nsh> dshot esc_info -m 2
  INFO  [dshot] ESC Type: #TEKKO32_4in1#
  INFO  [dshot] MCU Serial Number: xxxxxx-xxxxxx-xxxxxx-xxxxxx
  INFO  [dshot] Firmware version: 32.60
  INFO  [dshot] Rotation Direction: normal
  INFO  [dshot] 3D Mode: off
  INFO  [dshot] Low voltage Limit: off
  INFO  [dshot] Current Limit: off
  INFO  [dshot] LED 0: unsupported
  INFO  [dshot] LED 1: unsupported
  INFO  [dshot] LED 2: unsupported
  INFO  [dshot] LED 3: unsupported
  ```

- Permanently set the spin direction of a motor connected to FMU output pin 1 (while motors are _not_ spinning):

  - Set spin direction to `reversed`:

    ```sh
    dshot reverse -m 1
    dshot save -m 1
    ```

    Retrieving ESC information will then show:

    ```sh
    Rotation Direction: reversed
    ```

  - Set spin direction to `normal`:

    ```sh
    dshot normal -m 1
    dshot save -m 1
    ```

    Retrieving ESC information will then show:

    ```sh
    Rotation Direction: normal
    ```

  :::info

  - The commands will have no effect if the motors are spinning, or if the ESC is already set to the corresponding direction.
  - The ESC will revert to its last saved direction (normal or reversed) on reboot if `save` is not called after changing the direction.


:::

## Телеметрія

Деякі ESC можуть надсилати телеметрію до контролера польоту, включаючи:

- температура
- напруга
- струм
- накопичене поточне споживання
- Значення RPM

Ці DShot ESCs матимуть додатковий телеметрійний дріт.

Щоб увімкнути цю функцію (на ESC, які її підтримують):

1. Об'єднайте всі дроти телеметрії з усіх ESC разом, а потім підключіть їх до одного з контактів RX на не використаному порту послідовного зв'язку контролера польоту.
2. Enable telemetry on that serial port using [DSHOT_TEL_CFG](../advanced_config/parameter_reference.md#DSHOT_TEL_CFG).

Після перезавантаження ви можете перевірити, чи працює телеметрія (переконайтеся, що акумулятор підключений), використовуючи:

```sh
dshot esc_info -m 1
```

:::tip
You may have to configure [MOT_POLE_COUNT](../advanced_config/parameter_reference.md#MOT_POLE_COUNT) to get the correct RPM values.
:::

:::tip
Not all DSHOT-capable ESCs support `[esc_info]`(e.g. APD 80F3x), even when telemetry is supported and enabled.
Отримана помилка:

```sh
ERROR [dshot] No data received. If telemetry is setup correctly, try again.
```

Перевірте документацію виробника для підтвердження/подробиць.
:::
