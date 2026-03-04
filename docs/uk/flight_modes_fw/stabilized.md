# Стабілізований режим (фіксоване крило)

<img src="../../assets/site/difficulty_medium.png" title="Medium difficulty to fly" width="30px" />&nbsp;<img src="../../assets/site/remote_control.svg" title="Manual/Remote control required" width="30px" />

_Stabilized mode_ is a manual mode were centering the sticks levels the vehicle attitude (roll and pitch) and maintains the horizontal posture.

:::info
_Stabilized mode_ is similar to [Altitude mode](../flight_modes_fw/altitude.md) in that releasing the sticks levels the vehicle, but unlike altitude mode it does not maintain altitude or heading.
It is much easier to fly than [Manual mode](../flight_modes_fw/manual.md) because you can't roll or flip it, and if needed it is easy to level the vehicle (by centering the control sticks).
:::

The vehicle climbs/descends based on pitch and throttle input and performs a [coordinated turn](https://en.wikipedia.org/wiki/Coordinated_flight) if the roll stick is non-zero.
Курс і крен контролюються кутом (ви не можете перекотитися чи зробити петлю).

Транспортний засіб буде летіти інертно, якщо рівень газу знижується до 0% (двигун зупиняється).
Для виконання повороту команда повинна бути утримана протягом маневру, оскільки якщо випускати рульове управління, літак зупинить поворот і врівноважить себе (так само відносно тангажу).

Стік повороту може бути використана для збільшення/зменшення кута приводу автомобіля на поворотах.
Якщо контролер фіксований у центрі, то він самостійно здійснює координацію повороту, що означає, що він застосовує необхідну швидкість розвороту для поточного кута крену, щоб виконати плавний поворот.

The diagram below shows the mode behaviour visually (for a [mode 2 transmitter](../getting_started/rc_transmitter_receiver.md#transmitter_modes)).

![FW Manual Flight](../../assets/flight_modes/stabilized_fw.png)

## Технічний опис

Ручний режим, де центровані стіки крену/тангажурівняють положення транспортного засобу.
Курс і висота транспортного засобу не підтримуються, і він може дрейфувати через вітер.

- Центровані стіки крену/тангажу/рискання (в межах мертвої зони) встановлюють повітряний апарат на прямолінійний та горизонтальний польот.
  Курс і висота транспортного засобу не підтримуються, і він може дрейфувати через вітер.
- Стік керування використовує кут крена.
  Автопілот буде підтримувати <a href="https://en.wikipedia.org/wiki/Coordinated_flight">координований польот</a>.
- Pitch stick controls pitch angle around the defined offset [FW_PSP_OFF](../advanced_config/parameter_reference.md#FW_PSP_OFF)
- Ручка дроселя керує дроселем безпосередньо.
- Стік крену додає додатковий значення швидкості рискання (додається до розрахованого автопілотом для підтримки координованого польоту).
  Може бути використаний для ручної зміни кута рискання безпілотного засобу.
- Потрібен ручний ввід управління (наприклад, за допомогою пульта дистанційного керування, джойстика).

## Параметри

Режим впливає на наступні параметри:

| Параметр                                                                                                                                                          | Опис                                                                                                                                                                        |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="FW_MAN_P_MAX"></a>[FW_MAN_P_MAX](../advanced_config/parameter_reference.md#FW_MAN_P_MAX)    | Максимальний крен для керування вручну в режимі стабілізації кута нахилу. За замовчуванням: 45 градусів.                    |
| <a id="FW_MAN_R_MAX"></a>[FW_MAN_R_MAX](../advanced_config/parameter_reference.md#FW_MAN_R_MAX)    | Максимальне значення крена для керування в ручному режимі в режимі стабілізації кута нахилу. За замовчуванням: 45 градусів. |
| <a id="FW_MAN_YR_MAX"></a>[FW_MAN_YR_MAX](../advanced_config/parameter_reference.md#FW_MAN_YR_MAX) | Максимальна вручну додана швидкість крену. За замовчуванням: 30 градусів на секунду.                                        |
| <a id="FW_PSP_OFF"></a>[FW_PSP_OFF](../advanced_config/parameter_reference.md#FW_PSP_OFF)                               | Зсув вихідної установки крена (крен на рівному польоті). За замовчуванням: 0 градусів.                   |

<!-- this document needs to be extended -->
