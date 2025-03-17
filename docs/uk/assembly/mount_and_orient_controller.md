# Кріплення польотного контролера

The flight controller should be placed on the frame as close as possible to the centre-of-gravity (CoG), top-side up, and oriented so that the _heading mark arrow_ points towards the front of the vehicle.
[Vibration isolation](#vibration-isolation) is often needed, and you should follow the manufacturer recommendations.
Якщо змонтовано таким чином, подальша конфігурація PX4 не потрібна.

## Орієнтація

Almost all Flight Controllers have a _heading mark arrow_ (shown below).
Контролер повинен бути розміщений на верхній стороні рами, орієнтований таким чином, щоб стрілка вказувала вперед транспортного засобу (на всіх рамах повітряних суден - літак, багатокоптер, VTOL, наземні транспортні засоби тощо).

![FC Heading Mark](../../assets/qgc/setup/sensor/fc_heading_mark_1.png)

![FC Orientation](../../assets/qgc/setup/sensor/fc_orientation_1.png)

:::info
If the controller cannot be mounted in the recommended/default orientation due to physical constraints, you will need to configure the autopilot software with the orientation that you actually used: [Flight Controller Orientation](../config/flight_controller_orientation.md).
:::

## Положення

Контролер польоту повинен бути розміщений на рамці якнайближче до центру ваги.

If you can't mount the controller in this position, then you should [configure](../advanced_config/parameters.md) the following parameters to set offset relative to the CoG: [EKF2_IMU_POS_X](../advanced_config/parameter_reference.md#EKF2_IMU_POS_X), [EKF2_IMU_POS_Y](../advanced_config/parameter_reference.md#EKF2_IMU_POS_Y), [EKF2_IMU_POS_Z](../advanced_config/parameter_reference.md#EKF2_IMU_POS_Z) (for the default EKF2 estimator).

Зверніть увагу, що якщо ви не встановите ці зміщення, то оцінки положення / швидкості EKF2 будуть відображатися на місці його розташування, а не в ЦМ.
Це може призвести до небажаних коливань, залежно від того, наскільки далеко знаходиться IMU від CoG.

:::details
Explanation
To understand the impact of not setting these offsets, consider the case when the flight controller (IMU) is in front of the CoG, you're flying in position mode, and there is a forward pitching motion around the CoG.
Оцінка висоти зменшиться, оскільки IMU фактично рухалася вниз.
Як реакцію, контролер висоти надасть більше тяги для компенсації.
Амплітуда залежить від того, наскільки далеко знаходиться IMU від CoG.
Можливо, це може бути незначним, але це все ще деякий непотрібний зусилля по контролю, яке постійно застосовується.
Якщо вказані зміщення, чистий рух висоти не створить жодних змін у приблизній оцінці, тому буде менше паразитних коригувань.
:::

## Ізоляція Вібрацій

Плати управління польотом з вбудованими акселерометрами або гіроскопами чутливі до вібрацій.
Some boards include in-built vibration-isolation, while others come with _mounting foam_ that you can use to isolate the controller from the vehicle.

![Pixhawk Mounting foam](../../assets/hardware/mounting/3dr_anti_vibration_mounting_foam.png)
_Vibration damping foam_

Ви повинні використовувати стратегію монтажу, рекомендовану у документації вашого контролера польоту.

:::tip
[Log Analysis using Flight Review > Vibration](../log/flight_review.md#vibration) explains how to test whether vibration levels are acceptable, and [Vibration Isolation](../assembly/vibration_isolation.md) suggests a number of possible solutions if there is a problem.
:::
