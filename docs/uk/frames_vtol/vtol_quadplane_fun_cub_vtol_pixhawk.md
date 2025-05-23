# FunCub QuadPlane (Pixhawk)

QuadPlane VTOL Fun Cub є стандартним повітряним судном з хвостовим оперенням (Multiplex FunCub), яке було модернізовано з системою QuadCopter.

Основна Інформація:

- **Frame:** Multiplex FunCub
- **Flight controller:** Pixhawk

![Fun Cub VTOL](../../assets/airframes/vtol/funcub_pixhawk/fun_cub_vtol_complete.jpg)

Без змін, Fun Cub - це відносно доступний літак і відносно легкий у польоті.
Після конвертації літак стає значно важчим і менш аеродинамічним.
Він все ще досить добре літає, але потребує близько 75% газу у польоті вперед.

## Специфікація матеріалів

Справжній літак приблизно виглядає так, як показано на зображенні вище (інші схожі моделі також підійдуть добре - це Multiplex Fun Cub).

Мінімально необхідне обладнання:

- Multiplex FunCub (або подібний)
- Pixhawk або сумісний
- Цифровий датчик швидкості польоту
- 900 кВ двигуни (наприклад, комплект пропульсії Iris - двигуни та регулятори швидкості)
- 10" пропелери для квадрокоптерів (10х45 або 10х47)
- 10" гвинт для мотора фіксованого крила (10×7)
- Модуль GPS
- Батарея 4S
- Алюмінієва рама для кріплення двигунів квадрокоптера (квадратна труба 10х10 мм, стінка 1 мм)
- TOW важить ~2.3кг з батареєю 4S на 4200mAh

## Структура

Структура виготовлена з алюмінієвих стрижнів, як показано нижче.

![quad_frame](../../assets/airframes/vtol/funcub_pixhawk/fun_cub_aluminium_frame_for_vtol.jpg)
![Fun Cub -frame for vtol mounted](../../assets/airframes/vtol/funcub_pixhawk/fun_cub_aluminium_frame_for_vtol_mounted.jpg)

## Підключення

Motor and servo wiring is nearly entirely up to you, but should match the [Generic Standard VTOL](../airframes/airframe_reference.md#vtol_standard_vtol_generic_standard_vtol) configuration, as shown in the airframe reference.
The geometry and output assignment can be configured in the [Actuators Configuration](../config/actuators.md#actuator-outputs)

Наприклад, ви можете з’єднати його так, як у цьому прикладі (орієнтація як у "сидячи в літаку"):

| Порт   | Підключення                                    |
| ------ | ---------------------------------------------- |
| MAIN 1 | Передній правий мотор (CCW) |
| MAIN 2 | Задній лівий мотор (CCW)    |
| MAIN 3 | Передній лівий мотор (CW)   |
| MAIN 4 | Правий задній мотор (CW)    |
| AUX 1  | Лівий елерон TODO                              |
| AUX 2  | Правий елерон                                  |
| AUX 3  | Elevator                                       |
| AUX 4  | Rudder                                         |
| AUX 5  | Тяга                                           |

For further instructions on wiring and configurations please see:
[Standard VTOL Wiring and Configuration](../config_vtol/vtol_quad_configuration.md). <!-- replace with Pixhawk Wiring Quickstart -->

## Конфігурація планера

1. For [Airframe](../config/airframe.md) select the vehicle group/type as _Standard VTOL_ and the specific vehicle as [Generic Standard VTOL](../airframes/airframe_reference.md#vtol_standard_vtol_generic_standard_vtol) as shown below (do not forget to click **Apply and Restart** in the top).

   ![QCG - Select Generic Standard VTOL](../../assets/qgc/setup/airframe/px4_frame_generic_standard_vtol.png)

2. Configure the outputs and geometry following the instructions in [Actuators Configuration](../config/actuators.md)

3. За замовчуванням параметри часто достатні для стабільного польоту. For more detailed tuning information see [Standard VTOL Wiring and Configuration](../config_vtol/vtol_quad_configuration.md).

Після завершення калібрування, VTOL готовий до польоту.

## Відео

<lite-youtube videoid="4K8yaa6A0ks" title="Fun Cub PX4 VTOL Maiden"/>

## Підтримка

If you have any questions regarding your VTOL conversion or configuration please visit <https://discuss.px4.io/c/px4/vtol>.

