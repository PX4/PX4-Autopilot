# QGroundControl Flight-Readiness Status

PX4 performs a number of preflight sensor quality and estimator checks to determine if, for example, there is a good enough position estimate to fly the vehicle in the current mode, and will block arming if the vehicle is not ready.

QGroundControl can be used to determine whether the vehicle is ready to fly, and more importantly what checks are failing.

:::tip
You can also get readiness notifications from the [vehicle status LEDs](../getting_started/led_meanings.md) and [warning tunes](../getting_started/tunes.md).
However QGC is the only way to determine the precise reasons why PX4 will not arm.
:::

## Flight Readiness Status

The overall "readiness to fly" is displayed in QGroundControl in the top left corner near the **Q** menu icon, as shown below:

![QGC flight readiness indicators from top left corner](../../assets/flying/qgc_flight_readiness.png)

The three states are:

- "Ready to Fly" (Green background): The vehicle is ready to fly in all modes, and can be armed.
- "Ready to Fly" (Amber background): The vehicle is ready to fly in the current mode and can be armed, but some check is failing that means it will not be able to switch to some other mode.
- "Not Ready" (Amber background): The vehicle is not ready to fly in the current mode, and cannot be armed.

## QGC Arming Check Report

<Badge type="tip" text="PX4 v1.14" /> <Badge type="tip" text="QGC v4.2.0" />

You can find out what prearming checks are failing using the QGroundControl [Arming Check Report](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/fly_view/fly_view.html#arm) in _Fly View_.
To access this UI select the [Flight Readiness Status](#flight-readiness-status) indicator in the top left corner of QGroundControl's Fly View.

![QGC Arming Check Report](../../assets/flying/qgc_arming_checks_ui.png)

The Arming Check Report will then pop up and list all current warnings, with a toggle on the right of each warning that expands each entry with additional information and possible solutions.

Once each issue is resolved it will disappear from the UI. When all issues blocking arming have been removed you can use the arm button to display the arming confirmation slider, and arm the vehicle (or you can just take off).

:::tip
The QGC Arming Checks UI is available in the QGC Daily Build (QGC v4.2.0 and later), and works with PX4 v1.14 and later.
:::

## Журнали польотів

Preflight errors are also reported in _QGroundControl_ as `PREFLIGHT FAIL` messages.
The `estimator_status.gps_check_fail_flags` message [in the logs](../getting_started/flight_reporting.md) shows which GPS quality checks are failing.

Note that the [Arming Check Report](#qgc-arming-check-report) is a much easier way to determine reasons for failure, but the logs may be useful in versions prior to PX4 v1.14.

## Передпольотні перевірки/помилки EKF

This sections lists errors, with associated checks and parameters, that are reported by [EKF2](../advanced_config/tuning_the_ecl_ekf.md) (and propagate to _QGroundControl_).
These are provided for information only (the QGC Arming Checks UI is the best way to get error and solution information).

#### PREFLIGHT FAIL: EKF HIGH IMU ACCEL BIAS

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/commander/Arming/PreFlightCheck/checks/ekf2Check.cpp#L267 -->

<!-- Useful primer on biases: https://www.vectornav.com/resources/inertial-navigation-primer/specifications--and--error-budgets/specs-imuspecs -->

<!-- Mathieu Bresciani is expert -->

Зміщення прискорення EKF IMU – це різниця між виміряним прискоренням, яке повідомляє датчик IMU, і очікуваним прискоренням, яке повідомляє оцінювач EKF2 (який об’єднує дані про позицію та/або швидкість з ряду джерел, включаючи IMU, GNSS, датчики потоку тощо).
This bias may change when the sensor is turned on ("turn-on bias") and over time due to noise and temperature differences ("in-run bias").
Число, як правило, має бути дуже малим (близько нуля), що вказує на те, що всі вимірювання з різних джерел збігаються щодо прискорення.

Попередження вказує на те, що зсув перевищує певний довільний поріг (апарату не буде дозволено злетіти).
Швидше за все, це ознака того, що потрібне калібрування акселерометру або термокалібрування:

- If you _sometimes_ get the warning: [re-calibrate the accelerometer](../config/accelerometer.md).
- If you get _regularly_ get the warning: Perform a [thermal calibration](../advanced_config/sensor_thermal_calibration.md).
- Якщо ви все ще отримуєте попередження після термального калібрування (або ви не можете виконати термальне калібрування):
  - Переконайтеся, що проблеми не виникають через апаратне забезпечення датчика чи автопілота:
    - Найпростіший спосіб зробити це — перевірити те саме шасі/сенсори з іншим автопілотом.
    - Alternatively, [log and compare](../dev_log/logging.md#configuration) all accelerometers across a number of bench test runs with `6: Sensor comparison` enabled in [SDLOG_PROFILE](../advanced_config/parameter_reference.md#SDLOG_PROFILE).
  - Спробуйте змінити параметри налаштування навчання зміщенню акселерометра.

Збільшення параметрів зменшить вірогідність виявлення аномалії автопілотом і може змінити стабільність оцінювача.
Однак це може знадобитися, якщо є проблеми з датчиком, які неможливо вирішити іншими засобами (тобто ви можете налаштувати EKF для кращої продуктивності, але неможливо «краще» відкалібрувати акселерометр).

:::warning
Tuning these parameters is a last resort.
Це слід робити, лише якщо у вас є дані, які показують, що це покращить продуктивність оцінювача.
:::

| Параметр                                                                                                                                                                   | Опис                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
| -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="EKF2_ABL_LIM"></a>[EKF2_ABL_LIM](../advanced_config/parameter_reference.md#EKF2_ABL_LIM)                                  | Максимальне значення зміщення, яке дозволено оцінити EKF (вище цього значення зміщення буде обрізано, і EKF спробує скинути себе, можливо, навіть переключившись на більш здоровий EKF із робочим IMU у системі з кількома EKF). The autopilot will report a "high accel bias" if the estimated bias exceeds 75% of this parameter during a preflight check and prevent takeoff. Поточне значення 0.4m/s2 вже досить високе, і його збільшення призведе до того, що автопілот виявлятиме проблеми з меншою ймовірністю. |
| <a id="EKF2_ABIAS_INIT"></a>[EKF2_ABIAS_INIT](../advanced_config/parameter_reference.md#EKF2_ABIAS_INIT)                         | Initial bias uncertainty (if perfectly calibrated, this is related to the "turn-on bias" of the sensor). Деякі користувачі можуть захотіти зменшити це значення, якщо вони знають, що датчик добре відкалібрований і що зміщення увімкнення невелике.                                                                                                                                                                                                                                                                                                   |
| <a id="EKF2_ACC_B_NOISE"></a>[EKF2_ACC_B_NOISE](../advanced_config/parameter_reference.md#EKF2_ACC_B_NOISE) | The expected "in-run bias" of the accelerometer or "how fast do we expect the bias to change per second". За замовчуванням це значення достатньо велике, щоб включити дрейф через зміну температури. Якщо IMU відкалібровано за температурою, користувач може захотіти зменшити цей параметр.                                                                                                                                                                                                                                                              |
| <a id="EKF2_ABL_ACCLIM"></a>[EKF2_ABL_ACCLIM](../advanced_config/parameter_reference.md#EKF2_ABL_ACCLIM)                         | Максимальне прискорення, при якому оцінювач намагатиметься вивчати зміщення прискорення. Це необхідно для запобігання тому, щоб оцінювач вчився зміщенню через помилки нелінійності та коефіцієнта масштабу. (Майже жоден користувач не має потреби змінювати цей параметр, за винятком випадків, коли він справді знає, що робить).                                                                                                                                                                                                    |

#### PREFLIGHT FAIL: EKF HIGH IMU GYRO BIAS

- Ця помилка виникає, коли зсув гіроскопа IMU, оцінений EKF, є надмірним.
- У цьому випадку надмірне означає, що оцінка зміщення перевищує 10 градусів/с (половина налаштованого обмеження, яке встановлено 20 градусів/с).

#### PREFLIGHT FAIL: ACCEL SENSORS INCONSISTENT - CHECK CALIBRATION

- Це повідомлення про помилку виникає, коли вимірювання прискорення від різних IMU суперечать.
- Ця перевірка стосується лише плат із кількома IMU.
- The check is controlled by the [COM_ARM_IMU_ACC](../advanced_config/parameter_reference.md#COM_ARM_IMU_ACC) parameter.

#### PREFLIGHT FAIL: GYRO SENSORS INCONSISTENT - CHECK CALIBRATION

- Це повідомлення про помилку виникає, коли вимірювання кутової швидкості від різних IMU суперечать.
- Ця перевірка стосується лише плат із кількома IMU.
- The check is controlled by the [COM_ARM_IMU_GYR](../advanced_config/parameter_reference.md#COM_ARM_IMU_GYR) parameter.

#### PREFLIGHT FAIL: COMPASS SENSORS INCONSISTENT - CHECK CALIBRATION

- Це повідомлення про помилку виникає, коли різниця в вимірюваннях від різних датчиків компаса надто велика.
- Це вказує на погане калібрування, орієнтацію або магнітні перешкоди.
- Ця перевірка застосовується, лише коли підключені більше одного компаса/магнітометра.
- The check is controlled by the [COM_ARM_MAG_ANG](../advanced_config/parameter_reference.md#COM_ARM_MAG_ANG) parameter.

#### PREFLIGHT FAIL: EKF INTERNAL CHECKS

- Це повідомлення про помилку виникає, якщо величини нововведень горизонтальної швидкості GPS, магнітного повороту, вертикальної швидкості GPS або датчика вертикального положення (за замовчуванням барометр, але може бути далекомір або GPS, якщо використовуються нестандартні параметри) є надмірними. Нововведення - це різниця між значенням, передбаченим інерційним навігаційним розрахунком, і виміряним датчиком.
- Користувачам слід перевірити рівні нововведень у файлі журналу, щоб визначити причину. These can be found under the `ekf2_innovations` message.
  Поширені проблеми/рішення включають:
  - Дрейф IMU під час прогріву. Можна вирішити, перезапустивши автопілот. Може знадобитися калібрування IMU акселерометра та гіроскопа.
  - Суміжні магнітні перешкоди в поєднанні з рухом апарату. Можна вирішити, перемістивши апарат і зачекавши або перезавантаживши його.
  - Погане калібрування магнітометра в поєднанні з рухом апарату. Вирішується шляхом повторного калібрування.
  - Початковий поштовх або швидкий рух під час запуску, що спричинено поганим інерційним навігаційним рішенням. Можна вирішити, перезапустивши апарат і зведенням руху до мінімуму протягом перших 5 секунд.

## Інші параметри

Наступні параметри також впливають на передпольотну перевірку.

#### COM_ARM_WO_GPS

The [COM_ARM_WO_GPS](../advanced_config/parameter_reference.md#COM_ARM_WO_GPS) parameter controls whether or not arming is allowed without a global position estimate.

- `1` (default): Arming _is_ allowed without a position estimate for flight modes that do not require position information (only).
- `0`: Arming is allowed only if EKF is providing a global position estimate and EFK GPS quality checks are passing
