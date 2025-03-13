# Тест MC_05 - Політ у приміщенні (ручні режими)

## Коли використовувати цю тестову картку

- Перший політ нової конструкції
- Коли необхідно повторити проблему в обмеженій області
- Експериментальні збірки, які можуть мати проблеми стабільності
- Тестування обладнання, яке було замінено та/або модифіковано

## Озброєння та зліт

❏ Встановіть режим польоту на стабілізацію та поставте під охорону

❏ Зліт, піднявши дросель

## Політ

❏ Стабілізований

&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response 1:1

&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response 1:1

❏ Висота

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vertical position should hold current value with stick centered

&nbsp;&nbsp;&nbsp;&nbsp;❏ Pitch/Roll/Yaw response 1:1

&nbsp;&nbsp;&nbsp;&nbsp;❏ Throttle response set to Climbs/Descend rate

## Посадка

❏ Посадка в режимі Стабілізації або на висоті з дроселем нижче 40%

❏ Upon touching ground, copter should disarm automatically within 2 seconds (disarm time set by parameter: [COM_DISARM_LAND](../advanced_config/parameter_reference.md#COM_DISARM_LAND))

## Очікувані результати

- Зліт повинен бути плавним, коли газ піднято
- Немає коливання в жодному з перерахованих режимів польоту
- Після посадки, коптер не повинен підскакувати на землі
