# Відтворення усієї системи

Можливо записувати та відтворювати довільні частини системи на основі повідомлень ORB.

Перегравання корисне для тестування ефекту різних значень параметрів на основі реальних даних, порівняння різних оцінювачів тощо.

## Вимоги

Перший крок - визначити модуль або модулі, які слід відтворити.
Потім визначте всі вхідні дані для цих модулів, тобто підписані теми ORB.
Для системного відтворення це включає в себе всі апаратні вхідні дані: сенсори, вхід RC, команди MAVLink та файлову систему.

All identified topics need to be logged at full rate (see [logging](../dev_log/logging.md)).
For `ekf2` this is already the case with the default set of logged topics.

It is important that all replayed topics contain only a single absolute timestamp, which is the automatically generated field `timestamp`.
Якщо потрібно додати більше відміток часу, вони повинні бути відносно основної відмітки.
For an example, see [SensorCombined.msg](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorCombined.msg).
Причини цього наведено нижче.

## Використання

- Спочатку виберіть файл для відтворення та побудуйте ціль (з каталогу PX4-Autopilot):

  ```sh
  export replay=<absolute_path_to_log_file.ulg>
  make px4_sitl_default
  ```

  This will create the build/make output in a separate build directory `build/px4_sitl_default_replay` (so that the parameters don't interfere with normal builds).
  It's possible to choose any posix SITL build target for replay, since the build system knows through the `replay` environment variable that it's in replay mode.

- Add ORB publisher rules in the file `build/px4_sitl_default_replay/rootfs/orb_publisher.rules`.
  Цей файл визначає модулі, які мають право публікувати певні повідомлення.
  Має наступний формат:

  ```sh
  restrict_topics: <topic1>, <topic2>, ..., <topicN>
  module: <module>
  ignore_others: <true/false>
  ```

  This means that the given list of topics should only be published by `<module>` (which is the command name).
  Публікації з будь-якої з цих тем з іншого модуля мовчки ігноруються.
  If `ignore_others` is `true`, publications to other topics from `<module>` are ignored.

  For replay, we only want the `replay` module to be able to publish the previously identified list of topics.
  So, for replaying `ekf2`, the rules file should look like this:

  ```sh
  restrict_topics: sensor_combined, vehicle_gps_position, vehicle_land_detected
  module: replay
  ignore_others: true
  ```

  З цим модулями, які зазвичай публікують ці теми, не потрібно вимикати для повторення.

- _(Optional)_ Setup parameter overrides (see [instructions below](#overriding-parameters-in-the-original-log)).

- _(Optional)_ Copy a `dataman` mission file from the SD card to the build directory.
  Це потрібно лише у випадку, якщо місію слід переграти.

- Почніть відтворення:

  ```sh
  make px4_sitl_default jmavsim
  ```

  Це автоматично відкриє файл журналу, застосує параметри та почне відтворення.
  Після завершення буде повідомляти про результат і вихід.
  Новостворений файл журналу можна потім проаналізувати. It can be found in `rootfs/fs/microsd/log`, in subdirectories organised by date.
  Replayed log file names will have the `_replayed` suffix.

  Зверніть увагу, що вищезазначена команда також покаже симулятор, але - в залежності від того, що відтворюється - вона не покаже, що насправді відбувається.
  Ще завжди можна підключитися через QGC та, наприклад, переглянути зміну ставлення під час відтворення.

- Нарешті, скасуйте змінну середовища, щоб знову використовувалися звичайні цілі збирання:

  ```sh
  unset replay
  ```

### Перевизначення параметрів у вихідному журналі

За замовчуванням, під час відтворення застосовуються всі параметри зі стартового файлу журналу.
Якщо параметр змінюється під час запису, він буде змінений у відповідний час під час відтворення.

Parameters can be overridden during a replay in two ways: _fixed_ and _dynamic_.
Коли параметри перевизначаються, відповідні зміни параметрів в журналі не застосовуються під час повторного відтворення.

- **Fixed parameter overrides** will override parameters from the start of the replay.
  They are defined in the file `build/px4_sitl_default_replay/rootfs/replay_params.txt`, where each line should have the format `<param_name> <value>`.
  Наприклад:

  ```sh
  EKF2_RNG_NOISE 0.1
  ```

- **Dynamic parameter overrides** will update parameter values at specified times.
  Ці параметри все ще будуть ініціалізовані до значень у журналі або в зафіксованих замінах.
  Parameter update events should be defined in `build/px4_sitl_default_replay/rootfs/replay_params_dynamic.txt`, where each line has the format `<param_name> <value> <timestamp>`.
  Відмітка часу - це час у секундах з моменту початку журналу. Наприклад:

  ```sh
  EKF2_RNG_NOISE 0.15 23.4
  EKF2_RNG_NOISE 0.05 56.7
  EKF2_RNG_DELAY 4.5 30.0
  ```

### Важливе зауваження

- Під час відтворення всі відключення в журналі звітуються.
  Вони мають негативний вплив на повторення, тому слід уникати втрат під час запису.
- Наразі можливо лише відтворення в 'реальному часі': так швидко, як було зроблено запис.
  Це планується розширити у майбутньому.
- Повідомлення з міткою часу 0 буде вважатися недійсним і не буде відтворено.

## EKF2 Повтор

Це спеціалізація системного повторення для швидкого відтворення EKF2.

:::info
The recording and replay of flight logs with [multiple EKF2 instances](../advanced_config/tuning_the_ecl_ekf.md#running-multiple-ekf-instances) is not supported.
To enable recording for EKF replay you must set the parameters to enable a [single EKF2 instance](../advanced_config/tuning_the_ecl_ekf.md#running-a-single-ekf-instance).
:::

У режимі EKF2 відтворення автоматично створить правила публікації ORB, описані вище.

Для виконання повторення EKF2:

- Запишіть оригінальний журнал.
  Optionally set `SDLOG_MODE` to `1` to log from boot.

- In addition to the `replay` environment variable, set `replay_mode` to `ekf2`:

  ```sh
  export replay_mode=ekf2
  export replay=<absolute_path_to_log.ulg>
  ```

- Run the replay with the `none` target:

  ```sh
  make px4_sitl none
  ```

- Once finished, unset both `replay` and `replay_mode`.

  ```sh
  unset replay; unset replay_mode
  ```

### Налаштування конкретних параметрів EKF2 для відтворення

First install `pyulog`:

```sh
pip install --user pyulog
```

Extract the original log's parameters to `replay_params.txt`:

```sh
ulog_params -i "$replay" -d ' ' | grep -e '^EKF2' > build/px4_sitl_default_replay/rootfs/replay_params.txt
```

Adjust these as desired, and add dynamic parameter overrides in `replay_params_dynamic.txt` if necessary.

## Позаду кадру

Перегравання розділено на 3 компоненти:

- A replay module
  These have a negative effect on replay, so care should be taken to avoid dropouts during recording.
- Наразі можливо лише відтворення в 'реальному часі': так швидко, як було зроблено запис.

Модуль відтворення читає журнал та публікує повідомлення з тією самою швидкістю, з якою вони були записані.
До часового позначення кожного повідомлення додається постійний зсув, щоб відповідати поточному часу системи (це причина, чому всі інші часові позначення повинні бути відносними).
The command `replay tryapplyparams` is executed before all other modules are loaded and applies the parameters from the log and user-set parameters.
Then as the last command, `replay trystart` will again apply the parameters and start the actual replay.
Both commands do nothing if the environment variable `replay` is not set.

Правила видавця ORB дозволяють вибрати, яка частина системи повторюється, як описано вище. Вони компілюються лише для цільових posix SITL.

The **time handling** is still an **open point**, and needs to be implemented.
