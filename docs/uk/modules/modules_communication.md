# Modules Reference: Communication

## frsky_telemetry

Source: [drivers/telemetry/frsky_telemetry](https://github.com/PX4/PX4-Autopilot/tree/main/src/drivers/telemetry/frsky_telemetry)

Підтримка FrSky Telemetry. Автоматичне визначення протоколу D або S.PORT. <a id="frsky_telemetry_usage"></a>

### Використання

```
frsky_telemetry <command> [arguments...]
 Commands:
   start
     [-d <val>]  Select Serial Device
                 values: <file:dev>, default: /dev/ttyS6
     [-t <val>]  Scanning timeout [s] (default: no timeout)
                 default: 0
     [-m <val>]  Select protocol (default: auto-detect)
                 values: sport|sport_single|sport_single_invert|dtype, default:
                 auto

   stop

   status
```

## mavlink

Source: [modules/mavlink](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mavlink)

### Опис

Цей модуль реалізує протокол MAVLink, який можна використовувати на послідовному каналі або мережевому з'єднанні UDP.
Він взаємодіє з системою через uORB: деякі повідомлення обробляються безпосередньо в модулі (наприклад, протокол місії), інші публікуються через uORB (наприклад, vehicle_command).

Потоки використовуються для надсилання періодичних повідомлень з певною частотою, наприклад, про положення транспортного засобу.
При запуску екземпляра mavlink можна вказати режим, який визначає набір увімкнених потоків з їхніми швидкостями.
For a running instance, streams can be configured via `mavlink stream` command.

Може бути декілька незалежних екземплярів модуля, кожен з яких підключений до одного послідовного пристрою або мережевого порту.

### Імплементація

Реалізація використовує 2 потоки, потік відправлення та потік отримання. The sender runs at a fixed rate and dynamically
reduces the rates of the streams if the combined bandwidth is higher than the configured rate (`-r`) or the
physical link becomes saturated. This can be checked with `mavlink status`, see if `rate mult` is less than 1.

**Careful**: some of the data is accessed and modified from both threads, so when changing code or extend the
functionality, this needs to be take into account, in order to avoid race conditions and corrupt data.

### Приклади

Запустіть mavlink на послідовному каналі ttyS1 з швидкістю передачі даних 921600 і максимальною швидкістю надсилання 80 кБ/с:

```
mavlink start -d /dev/ttyS1 -b 921600 -m onboard -r 80000
```

Запустіть mavlink на UDP-порт 14556 і увімкніть повідомлення HIGHRES_IMU з частотою 50 Гц:

```
mavlink start -u 14556 -r 1000000
mavlink stream -u 14556 -s HIGHRES_IMU -r 50
```

<a id="mavlink_usage"></a>

### Використання

```
mavlink <command> [arguments...]
 Commands:
   start         Start a new instance
     [-d <val>]  Select Serial Device
                 values: <file:dev>, default: /dev/ttyS1
     [-b <val>]  Baudrate (can also be p:<param_name>)
                 default: 57600
     [-r <val>]  Maximum sending data rate in B/s (if 0, use baudrate / 20)
                 default: 0
     [-p]        Enable Broadcast
     [-u <val>]  Select UDP Network Port (local)
                 default: 14556
     [-o <val>]  Select UDP Network Port (remote)
                 default: 14550
     [-t <val>]  Partner IP (broadcasting can be enabled via -p flag)
                 default: 127.0.0.1
     [-m <val>]  Mode: sets default streams and rates
                 values: custom|camera|onboard|osd|magic|config|iridium|minimal|
                 extvision|extvisionmin|gimbal|uavionix, default: normal
     [-n <val>]  wifi/ethernet interface name
                 values: <interface_name>
     [-c <val>]  Multicast address (multicasting can be enabled via
                 MAV_{i}_BROADCAST param)
                 values: Multicast address in the range
                 [239.0.0.0,239.255.255.255]
     [-F <val>]  Sets the transmission frequency for iridium mode
                 default: 0.0
     [-f]        Enable message forwarding to other Mavlink instances
     [-w]        Wait to send, until first message received
     [-x]        Enable FTP
     [-z]        Force hardware flow control always on
     [-Z]        Force hardware flow control always off

   stop-all      Stop all instances

   stop          Stop a running instance
     [-u <val>]  Select Mavlink instance via local Network Port
     [-d <val>]  Select Mavlink instance via Serial Device
                 values: <file:dev>

   status        Print status for all instances
     [streams]   Print all enabled streams

   stream        Configure the sending rate of a stream for a running instance
     [-u <val>]  Select Mavlink instance via local Network Port
     [-d <val>]  Select Mavlink instance via Serial Device
                 values: <file:dev>
     -s <val>    Mavlink stream to configure
     -r <val>    Rate in Hz (0 = turn off, -1 = set to default)

   boot_complete Enable sending of messages. (Must be) called as last step in
                 startup script.
```

## uorb

Source: [systemcmds/uorb](https://github.com/PX4/PX4-Autopilot/tree/main/src/systemcmds/uorb)

### Опис

uORB - це внутрішня система обміну повідомленнями pub-sub, яка використовується для комунікації між модулями.

### Імплементація

Реалізація є асинхронною та безблоковою, тобто видавець не чекає на підписника і навпаки.
Це досягається завдяки наявності окремого буфера між публікатором і підписником.

Код оптимізовано для мінімізації використання пам'яті та затримок при обміні повідомленнями.

Messages are defined in the `/msg` directory. Вони перетворюються в код C/C++ під час збірки.

Якщо ви компілюєте з ORB_USE_PUBLISHER_RULES, файл з правилами публікації uORB можна використовувати для налаштування того, яким
модулям дозволено публікувати які теми. Це використовується для загальносистемного відтворення.

### Приклади

Відстежуйте показники публікацій тем. Besides `top`, this is an important command for general system inspection:

```
uorb top
```

<a id="uorb_usage"></a>

### Використання

```
uorb <command> [arguments...]
 Commands:
   status        Print topic statistics

   top           Monitor topic publication rates
     [-a]        print all instead of only currently publishing topics with
                 subscribers
     [-1]        run only once, then exit
     [<filter1> [<filter2>]] topic(s) to match (implies -a)
```
