# Generic Standard VTOL (QuadPlane) Configuration & Tuning

This is the configuration documentation for a [Generic Standard VTOL](../airframes/airframe_reference.md#vtol_standard_vtol_generic_standard_vtol), also known as a "QuadPlane VTOL".
Це в основному літальний апарат з фіксованими крилами з додаванням квадрокоптерних двигунів.

For airframe specific documentation and build instructions see [VTOL Framebuilds](../frames_vtol/index.md).

## Firmware & Basic Settings

1. Run _QGroundControl_
2. Flash the firmware for your current release or master (PX4 `main` branch build).
3. In the [Frame setup](../config/airframe.md) section select the appropriate VTOL airframe.

  If your airframe is not listed select the [Generic Standard VTOL](../airframes/airframe_reference.md#vtol_standard_vtol_generic_standard_vtol) frame.

### Перемикач режимів польоту / переходу

Вам слід призначити перемикач на вашому пульті управління радіокерованої моделі для перемикання між режимами багатокоптера і фіксованим крилом.

:::info
While PX4 allows flight without an RC controller, you must have one when tuning/configuring up a new airframe.
:::

This is done in [Flight Mode](../config/flight_mode.md) configuration, where you [assign flight modes and other functions](../config/flight_mode.md#what-flight-modes-and-switches-should-i-set) to switches on your RC controller.
The switch can also be assigned using the parameter [RC_MAP_TRANS_SW](../advanced_config/parameter_reference.md#RC_MAP_TRANS_SW).

Перемикач у вимкненому положенні означає, що ви польотаєте в режимі багатокоптера.

### Настройка мультироторів / фіксованих крил

Перш ніж ви спробуєте перший перехід до польоту з фіксованим крилом, вам слід абсолютно впевнитися, що ваш VTOL налаштований належним чином у режимі багатокоптера.
Одна з причин цього полягає в тому, що це режим, до якого ви повернетеся, якщо щось піде не так з переходом, і можливо, він буде рухатися досить швидко.
Якщо він не налаштований належним чином, можуть трапитися погані речі.

Якщо у вас є доступна злітно-посадкова смуга і загальна вага не занадто велика, вам також слід налаштувати польот з фіксованим крилом.
Якщо ні, то ви будете це намагатися зробити, коли він перейде в режим фіксованого крила.
Якщо щось піде не так, вам потрібно буде бути готовим (і здатним) повернутися до режиму багатокоптера.

Дотримуйтесь відповідних посібників з налаштування та мультироторів, і фіксованих крил.

### Налаштування переходу

Хоча може здатися, що ви працюєте з транспортним засобом, який може літати у двох режимах (багатокоптер для вертикальних злітів та посадок та фіксоване крило для прямолінійного польоту), також існує додатковий стан, який вам також потрібно налаштувати: перехід.

Важливо належним чином налаштувати ваш перехід для отримання безпечного входу в режим фіксованого крила, наприклад, якщо ваша швидкість повітря буде занадто низькою при переході, вона може загальмуватися.

#### Параметр переходу газу

Parameter: [VT_F_TRANS_THR](../advanced_config/parameter_reference.md#VT_F_TRANS_THR)

Перехідний газ спереду визначає цільовий газ для тягового мотора (підтягуючий / тяговий) під час переднього переходу.

Це повинно бути встановлено достатньо високим, щоб забезпечити досягнення швидкості переходу.
Якщо ваше повітряне судно обладнане датчиком швидкості повітря, ви можете збільшити цей параметр, щоб зробити передній перехід швидшим.
Для вашого першого переходу краще встановити значення вище, ніж нижче.

#### Швидкість наростання тягового мотора вперед / назад

Parameter: [VT_PSHER_SLEW](../advanced_config/parameter_reference.md#VT_PSHER_SLEW)

Передній перехід відноситься до переходу з режиму багатокоптера в режим фіксованого крила.
The forward transition pusher/puller slew rate is the amount of time in seconds that should be spent ramping up the throttle to the target value (defined by `VT_F_TRANS_THR`).

Значення 0 призведе до того, що команда встановлення газу переходу буде негайно встановлена.
За замовчуванням швидкість наростання встановлена на 0,33, що означає, що для досягнення 100% газу знадобиться 3 с.
Якщо ви хочете зробити збільшення гладкішим, ви можете зменшити це значення.

Зауважте, що після закінчення періоду збільшення газу газ буде встановлено на цільове значення і залишиться там, доки (сподіваємося) не буде досягнута швидкість переходу.
Змішування швидкості повітря.

#### Змішування повітряної швидкості

Parameter: [VT_ARSP_BLEND](../advanced_config/parameter_reference.md#VT_ARSP_BLEND)

За замовчуванням, коли швидкість повітря наближається до швидкості переходу, керування орієнтацією багатокоптера буде зменшуватися, а керування фіксованим крилом почне безперервно збільшуватися до того моменту, поки не відбудеться перехід.

Вимкніть змішування, встановивши цей параметр на 0, що збереже повне керування багатокоптером і нульове керування фіксованим крилом до моменту переходу.

#### Швидкість переходу

Parameter: [VT_ARSP_TRANS](../advanced_config/parameter_reference.md#VT_ARSP_TRANS)

Це швидкість повітря, яка, коли досягнута, спричинить перехід з режиму багатокоптера в режим фіксованого крила.
Дуже важливо правильно калібрувати ваш датчик швидкості повітря.
It is also important that you pick an airspeed that is comfortably above your airframes stall speed (check `FW_AIRSPD_MIN`) as this is currently not checked.

#### Openloop Transition Time

Parameter: [VT_F_TR_OL_TM](../advanced_config/parameter_reference.md#VT_F_TR_OL_TM)

This specifies the duration of the front transition in seconds when no airspeed feedback is available (e.g. no airspeed sensor present).
It should be set to a value which ensures that the vehicle reaches a high enough airspeed to complete the transition, e.g. airspeed should exceed [VT_ARSP_TRANS](../advanced_config/parameter_reference.md#VT_ARSP_TRANS).

#### Transition Timeout

[VT_TRANS_TIMEOUT](../advanced_config/parameter_reference.md#VT_TRANS_TIMEOUT)

This specifies the upper limit for the duration of the front transition. If the vehicle has not reached the transition airspeed after this time, then the transition will be aborted and a [Quadchute](../config/safety.md#quad-chute-failsafe) event will be triggered.
:::note
Additionally, if an airspeed sensor is present, the transition will also be aborted if the airspeed has not reached [VT_ARSP_BLEND](../advanced_config/parameter_reference.md#VT_ARSP_BLEND) after the openloop transition time [VT_F_TR_OL_TM](../advanced_config/parameter_reference.md#VT_F_TR_OL_TM) has elapsed. This checks is used to avoid a scenario where the vehicle gains excessive speed when the airspeed sensor is faulty.
:::

### Поради щодо переходу

Як вже зазначалося, переконайтеся, що у вас добре налаштований режим багатокоптера.
Якщо під час переходу виникне проблема, ви перейдете назад до цього режиму, і це повинно бути досить плавно.

Перед польотом майте план, що ви будете робити в кожній з трьох фаз (багатокоптер, перехід, фіксоване крило), коли ви будете в будь-якому з них, і виникне проблема.

Рівень заряду батареї: залиште достатньо місця для переходу багатокоптера для посадки в кінці польоту.
Не робіть батарею занадто розрядною, оскільки вам знадобиться більше потужності в режимі багатокоптера, щоб сісти.
Будьте консервативними.

#### Перехід: підготовка

Переконайтеся, що ви перебуваєте принаймні на висоті 20 метрів над землею і маєте достатньо місця для завершення переходу.
Можливо, вашому багатокоптеру VTOL доведеться втратити висоту, коли він перейде в режим фіксованого крила, особливо якщо швидкість повітря недостатня.

Перехід до вітру, коли це можливо, інакше він буде рухатися далі від вас, перш ніж перейти.

Перед початком переходу переконайтеся, що VTOL знаходиться в стабільному висінні.

#### Перехід: багатороторний на нерухоме крило (передній перехід)

Розпочніть перехід.
Він повинен відбуватися на відстані від 50 до 100 метрів.
Якщо цього не відбувається, або він не летить стабільно, перервіть перехід (див. нижче) і сідайте або повертайтеся у вихідне положення та сідайте.
Try increasing the [transition throttle](#transition-throttle) (`VT_F_TRANS_THR`) value.
Also consider reducing the transition duration (`VT_F_TRANS_DUR`) if you are not using an airspeed sensor.
Якщо ви використовуєте датчик швидкості повітря, розгляньте зниження швидкості переходу, але залиште її значно вище мінімальної швидкості стрибка.

Як тільки ви помітите, що відбувається перехід, будьте готові взяти під контроль втрату висоти, яка може включати швидке збільшення обертів.

:::warning
The following feature has been discussed but not implemented yet:

- Once the transition happens the multirotor motors will stop and the pusher/puller throttle will remain at the `VT_F_TRANS_THR` level until you move the throttle stick, assuming you are in manual mode.

:::

#### Перехід: Фіксовані крила на багатороторний вертоліт (Зворотний перехід)

Під час переходу назад до режиму багатороторного вертольота приведіть свій літак на прямий рівний захід та зменште його швидкість, перекиньте перемикач переходу і двигуни багатороторного вертольота почнуть роботу, а тяговий пропелер відразу зупиниться, що повинно призвести до досить плавного перехідного глайдування.

Пам'ятайте, що значення газу, яке ви маєте при переході, вказуватиме кількість тяги вашого багатороторного вертольота у момент перемикання. Оскільки крило все ще буде у повітрі, ви відчуєте, що у вас є достатньо часу, щоб відрегулювати газ для досягнення/утримання стійкого витримання.
Для розширеного налаштування зворотного переходу дивіться Посібник з налаштування зворотного переходу.

For advanced tuning of the back-transition please refer to the [Back-transition Tuning Guide](vtol_back_transition_tuning.md)

#### Скасування переходу

It’s important to know what to expect when you revert a transition command _during_ a transition.

When transitioning from **multirotor to fixed-wing** (transition switch is on/fixed-wing) then reverting the switch back (off/multirotor position) _before_ the transition happens it will immediately return to multirotor mode.

When transitioning from **fixed-wing to multirotor** for this type of VTOL the switch is immediate so there isn’t really a backing out option here, unlike for tilt rotor VTOLs.
Якщо ви хочете, щоб він повернувся у режим фіксованих крил, вам потрібно буде пройти повний перехід.
Якщо він все ще рухається швидко, це має відбутися швидко.

### Підтримка

If you have any questions regarding your VTOL conversion or configuration please see [discuss.px4.io/c/px4/vtol](https://discuss.px4.io/c/px4/vtol).
