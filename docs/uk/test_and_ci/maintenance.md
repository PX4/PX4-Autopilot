# Вказівки щодо підтримки

Тут зібрані та описані деякі інструменти, які допомагають аналізувати стан кодової бази та підтримувати її роботу.

## Аналіз змін

Кількість змін, зроблених у файлі, може бути індикатором того, які файли/частини можуть потребувати рефакторингу.

To find churn metrics a tool such as [Churn](https://github.com/danmayer/churn) can be used:

```sh
gem install churn
```

An example output as of `v1.6.0-rc2` would be:

```sh
cd src/PX4-Autopilot
churn --start_date "6 months ago"
**********************************************************************
* Revision Changes
**********************************************************************
Files
+------------------------------------------+
| file                                     |
+------------------------------------------+
| src/modules/navigator/mission.cpp        |
| src/modules/navigator/navigator_main.cpp |
| src/modules/navigator/rtl.cpp            |
+------------------------------------------+



**********************************************************************
* Project Churn
**********************************************************************

Files
+---------------------------------------------------------------------------+---------------+
| file_path                                                                 | times_changed |
+---------------------------------------------------------------------------+---------------+
| src/modules/mc_pos_control/mc_pos_control_main.cpp                        | 107           |
| src/modules/commander/commander.cpp                                       | 67            |
| ROMFS/px4fmu_common/init.d/rcS                                            | 52            |
| Makefile                                                                  | 49            |
| src/drivers/px4fmu/fmu.cpp                                                | 47            |
| ROMFS/px4fmu_common/init.d/rc.sensors                                     | 40            |
| src/drivers/boards/aerofc-v1/board_config.h                               | 31            |
| src/modules/logger/logger.cpp                                             | 29            |
| src/modules/navigator/navigator_main.cpp                                  | 28            |
```
