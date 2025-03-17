# Гелікоптери

<LinkedBadge type="warning" text="Experimental" url="../airframes/#experimental-vehicles"/>

:::warning
Support for helicopters is [experimental](../airframes/index.md#experimental-vehicles).
Maintainer volunteers, [contribution](../contribute/index.md) of new features, new frame configurations, or other improvements would all be very welcome!

Проблеми включають:

- Обмежена підтримка різних типів гелікоптерів.
  Наприклад, PX4 не підтримує гелікоптери з коаксіальним або подвійним ротором, а також функції, такі як автоматичне регулювання обертів і авторотація.

:::

<!-- image here please of PX4 helicopter -->

## Типи гелікоптерів

PX4 підтримує гелікоптери з одним головним ротором з рингом рушія, керованим рухомим майданчиком за допомогою до 4 рушійних сервоприводів рухомого майданчика, і:

- механічно розділене хвостове вістря, приводиме в дію електронним регулятором, або
- механічно з'єднане хвостове вістря, кероване сервоприводом на хвостовому двигуні.

The allowed flight operations and [flight modes](../flight_modes_mc/index.md) are the same as for multicopter.
Однак слід зауважити, що (на момент написання) 3D-польот з використанням негативного тяги не підтримується в автономних/керованих режимах.

## Збірка

Збирання основних компонентів автопілота схоже для всіх каркасів.
This is covered in [Basic Assembly](../assembly/index.md).

Специфічне для гелікоптерів збирання в основному полягає в підключенні і живленні двигунів та сервоприводів рухомого майданчика.

:::info
Зауважте, що контролер польоту не може живити двигуни та сервоприводи (лише модуль GPS, приймач RC і модулі телеметрії малої потужності можуть живитися від контролерів польоту Pixhawk).
Як правило, для живлення двигунів використовується плата розподілу електроенергії (PDB), а для живлення кожного з сервоприводів використовується окрема (або інтегрована) схема відключення батарей (BEC).
:::

## Налаштування

Конфігурацію та налаштування гелікооптера описано в:

- [Helicopter configuration](../config_heli/index.md): Vehicle frame selection, actuator configuration and testing, and tuning.
- [Standard Configuration](../config/index.md): The common configuration and calibration steps for most frames.
  Це включає конфігурацію/калібрування основних компонентів, таких як компас і гіроскоп, налаштування режимів польоту на пульті дистанційного керування та налаштування безпеки.

## Каркасні конструкції

Недоступно.
