# E-flite Convergence Tiltrotor VTOL (Pixfalcon)

The [E-Flite Convergence](https://youtu.be/HNedXQ_jhYo) can easily be converted to a fully autonomous VTOL with PX4.
There is not much space but it's enough for a [Pixfalcon](../flight_controller/pixfalcon.md) flight controller with GPS and telemetry.

:::info
The original Horizon Hobby _E-Flite Convergence_ frame and [Pixfalcon](../flight_controller/pixfalcon.md) have been discontinued.
Alternatives are provided in the [Purchase](#where-to-buy) section.
:::

<lite-youtube videoid="E61P2f2WPNU" title="E-flite Convergence Autonomous Mission Flight"/>

## Де купити

Опції рами транспортного засобу:

- **WL Tech XK X450** - [AliExpress](https://www.aliexpress.com/item/1005001946025611.html)
- **JJRC M02** - [Banggood (AU)](https://au.banggood.com/JJRC-M02-2_4G-6CH-450mm-Wingspan-EPO-Brushless-6-axis-Gyro-Aerobatic-RC-Airplane-RTF-3D-or-6G-Mode-Aircraft-p-1588201.html), [AliExpress](https://www.aliexpress.com/item/4001031497018.html)

Вибір контролера польоту ():

- [Pixhawk 4 Mini](../flight_controller/pixhawk4_mini.md)
- [Holybro Pixhawk Mini](../flight_controller/pixhawk_mini.md).
- Будь-який інший сумісний контролер польоту з достатньо малим форм-фактором.

## Налаштування програмного забезпечення

Транспортний засіб потребує 7 сигналів ШШД для моторів та поверхонь управління:

- Мотор (лівий/правий/задній)
- Сервоприводи нахилу (право/ліво)
- Elevons (вліво/вправо)

Ці можна підключити до виходів регулятора польоту більш-менш у будь-якому порядку (хоча виходи для двигунів повинні бути груповані разом і так далі).

The outputs are configured in the [Actuators Configuration](../config/actuators.md) by following the instructions for VTOL tiltrotor geometry and output configuration.
Note that you will need to start from the [Generic Tiltrotor VTOL](../airframes/airframe_reference.md#vtol_vtol_tiltrotor_generic_tiltrotor_vtol) frame.

Зверніть увагу, що ліворуч і праворуч на екрані конфігурації та посилання на раму визначені з точки зору людини-пілота всередині реального літака (або дивлячись зверху, як показано нижче):

<img src="../../assets/airframes/types/VTOLTiltRotor_eflite_convergence.svg" width="300px" />

### Політний контролер

Контролер польоту можна встановити там же, де був оригінальний автопілот.

![Mount Pixfalcon](../../assets/airframes/vtol/eflite_convergence_pixfalcon/eflight_convergence_pixfalcon_mounting.jpg)

### Телеметрійне радіо

Модуль телеметрії вставляється у відсік, призначений для розміщення механізму передачі FPV.

![Mount telemetry module](../../assets/airframes/vtol/eflite_convergence_pixfalcon/eflight_convergence_telemetry_module.jpg)

### GPS

Для GPS ми вирізали частину піни всередині «кабіни».
Таким чином GPS може бути поміщений всередину корпусу і ретельно зберігається без порушення зовнішнього вигляду транспортного засобу.

![Mount GPS](../../assets/airframes/vtol/eflite_convergence_pixfalcon/eflight_convergence_gps_mounting.jpg)

## Конфігурація PX4

Follow the [Standard Configuration](../config/index.md) in _QGroundControl_ (radio, sensors, flight modes, etc.).

Особливі налаштування, які є важливими для цього транспортного засобу:

- [Airframe](../config/airframe.md)
  - Select the airframe configuration **E-flite Convergence** under **VTOL Tiltrotor** and restart _QGroundControl_.
    ![QGroundControl Vehicle Setting - Airframe selection E-Flight](../../assets/airframes/vtol/eflite_convergence_pixfalcon/qgc_setup_airframe.jpg)
- [Flight Modes/Switches](../config/flight_mode.md)
  - As this is a VTOL vehicle, you must [assign an RC controller switch](../config/flight_mode.md#what-flight-modes-and-switches-should-i-set) for transitioning between multicopter and fixed-wing modes.
