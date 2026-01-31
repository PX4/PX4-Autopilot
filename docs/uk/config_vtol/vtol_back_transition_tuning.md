# Налаштування зворотного переходу VTOL

Коли VTOL виконує зворотний перехід (перехід з режиму фіксованих крил на багатороторний вертоліт), він повинен сповільнити рух перед тим, як багатороторний вертоліт може прийняти належний контроль.
To help with braking, the controller will pitch up the vehicle if the current deceleration is below what is set in expected deceleration ([VT_B_DEC_MSS](../advanced_config/parameter_reference.md#VT_B_DEC_MSS)).
The response of this deceleration controller can be tuned through a `I` gain: [VT_B_DEC_I](../advanced_config/parameter_reference.md#VT_B_DEC_I).
Increasing the `I` will result in more aggressive pitch-up to achieve the configured deceleration setting.

The vehicle will consider the back-transition complete when the horizontal speed has reached multicopter cruise speed ([MPC_XY_CRUISE](../advanced_config/parameter_reference.md#MPC_XY_CRUISE)) or when the back-transition duration ([VT_B_TRANS_DUR](../advanced_config/parameter_reference.md#VT_B_TRANS_DUR)) has passed (whichever comes first).

## Встановлення очікуваного уповільнення

When flying missions that make use of a [VTOL_LAND](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_VTOL_LAND) waypoint, the autopilot will attempt to calculate the proper distance at which to initiate the back-transition. Він робить це, розглядаючи поточну швидкість (порівнювальну зі швидкістю на землі) та очікуване сповільнення.
To get the vehicle to come out of back-transition very close to its landing point you can tune the expected deceleration ([VT_B_DEC_MSS](../advanced_config/parameter_reference.md#VT_B_DEC_MSS)) parameter.
Переконайтеся, що ви встановили достатньо великий час зворотного переходу, щоб дозволити транспортному засобу досягнути своєї заданої позиції, перш ніж цей таймаут спрацює.

## Тривалість зворотного переходу

Setting a high back-transition time ([VT_B_TRANS_DUR](../advanced_config/parameter_reference.md#VT_B_TRANS_DUR)) will give the vehicle more time to slow down.
Протягом цього періоду VTOL вимкне свій двигун фіксованих крил і повільно підніме свої MC двигуни під час глайдування.
Чим довше цей час, тим довше транспортний засіб буде глайдувати в спробі сповільнити рух. Недолік цього поведінки полягає в тому, що транспортний засіб буде контролювати лише висоту, а не позицію, під час цього періоду, тому може відбуватися деяке зміщення.
