# Функція VTOL Флюгер

The _weather vane_ feature automatically turns a VTOL vehicle to face its nose into the relative wind during hover flight.
Це поліпшує стабільність (зменшуючи ймовірність того, що вітер з боку підніме крило, що звернуто до вітру, і перекине транспортний засіб).

The feature is [enabled by default](#configuration) on VTOL hybrid vehicles flying in multicopter mode.

:::info
Weather vane functionality is not supported on pure multirotors.
:::

## Поведінка в ручному режимі

The weather vane feature will only take effect in [Position mode](../flight_modes_mc/position.md) (not other manual MC modes).

Користувач все ще може використовувати ручку повороту, щоб запитати швидкість повороту, навіть коли контролер флюгера намагається повернути ніс транспортного засобу по вітру.
Цільова швидкість повороту — це сума швидкості повороту флюгера та заданої користувачем швидкості повороту.

## Поведінка в режимі місії

In [Mission mode](../flight_modes_vtol/mission.md) the weather vane feature will always be active when the parameter is enabled.
Будь-який кут повороту, заданий у місії, ігноруватиметься.

<a id="configuration"></a>

## Налаштування

This functionality is configured using the [WV\_\* parameters](../advanced_config/parameter_reference.md#WV_EN).

| Параметр                                                                                                         | Опис                                                                                                           |
| ---------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| [WV_EN](../advanced_config/parameter_reference.md#WV_EN)                                    | Увімкнути флюгер.                                                                              |
| [WV_ROLL_MIN](../advanced_config/parameter_reference.md#WV_ROLL_MIN)   | Мінімальне задане значення кута крену для контролера флюгера, щоб вимагати швидкості повороту. |
| [WV_YRATE_MAX](../advanced_config/parameter_reference.md#WV_YRATE_MAX) | Максимальна швидкість, яку дозволено вимагати контролеру флюгера.                              |

## Як це працює?

Під час польоту на місці транспортний засіб повинен подолати опір, що справляється на нього вітром, щоб утримати своє положення.
Єдиний спосіб досягнути цього - нахилити вектор тяги в напрямку відносинного вітру (буквально "нахиляється" проти вітру).
Відстежуючи вектор тяги, можна оцінити напрям вітру.
Контролер поперечної стійкості використовується для керування швидкістю обертання, яка повертає нос транспортного засобу в оцінений напрям вітру.
