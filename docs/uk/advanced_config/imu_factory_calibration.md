# Заводське калібрування IMU/компаса

Виробники PX4 OEM можуть виконувати IMU і компасувати завод для збереження значень для акселерометра, калібрування гіроскопа та магнітометра у постійне сховище (зазвичай ЕЗПМ).
Це забезпечує можливість завжди скидати конфігурації та налаштування транспортного засобу до безпечного стану для польоту.

This procedure will write the following parameters to `/fs/mtd_caldata`: [CAL_ACC\*](../advanced_config/parameter_reference.md#CAL_ACC0_ID), [CAL_GYRO\*](../advanced_config/parameter_reference.md#CAL_GYRO0_ID), [CAL_MAG\*](../advanced_config/parameter_reference.md#CAL_MAG0_ID).
Ці дані будуть використані, коли параметри будуть встановлені (або скинуті) до їхніх значень за замовчуванням.

:::warning
This feature relies on the FMU having a dedicated EEPROM chip or an accompanying IMU PCBA that has sufficient space for the data.
PX4 will store the data to `/fs/mtd_caldata`, creating the file if necessary.
:::

:::info
Ці значення не можуть бути збережені в [конфігурації кадрів](../dev_airframes/adding_a_new_frame.md) оскільки вони відрізняються від пристрою в пристрої (конфігурація рамки визначає набір параметрів, які застосовуються на всіх автомобілях того ж типу, такі, як увімкнені датчики, [обертання автопілота](../config/flight_controller_orientation.md) і настроювання PID).
:::

## Виконання заводського калібрування

1. Встановіть параметр [SYS_FAC_CAL_MODE](../advanced_config/parameter_reference.md#SYS_FAC_CAL_MODE) на 1.
2. Виконайте всі калібрування IMU: [акселерометр](../config/accelerometer.md#performing-the-calibration), [гіроскоп](../config/gyroscope.md#performing-the-calibration) і [магнітометр](../config/compass.md#performing-the-calibration).
3. Перезапустіть транспортний засіб.
   This will write all `CAL_ACC*`, `CAL_GYRO*` and `CAL_MAG*` parameters into `/fs/mtd_caldata`.
4. Знову встановіть параметр `SYS_FAC_CAL_MODE` на 0 (за замовчуванням).

:::info
Якщо ви хочете провести заводську калібрування лише акселерометра та гіроскопа, ви можете встановити [SYS_FAC_CAL_MODE](../advanced_config/parameter_reference.md#SYS_FAC_CAL_MODE) на 2, у цьому випадку магнітометр виключається.
:::

Подальші корекції користувача будуть враховані як зазвичай (дані заводського калібрування використовуються лише для значень за замовчуванням параметрів).

## Подальша інформація

- [Посібник користувача з QGroundControl > Датчики](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/sensors_px4.html)
