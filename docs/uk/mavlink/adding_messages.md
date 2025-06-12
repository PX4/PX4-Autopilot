# Adding Standard MAVLink Definitions (Messages/Commands)

This topic explains how to add new MAVLink messages and commands that are expected to be _part of_ the normal PX4 build.

## Standard MAVLink Messages

The PX4/PX4-Autopilot source code uses only messages that have been standardized by MAVLink.
That is to say, the standard definitions in [common.xml](https://mavlink.io/en/messages/common.html) in releases, and [development.xml](https://mavlink.io/en/messages/development.html) during development.
These messages are present in at least one significant flight stack, and members of other flight stacks have accepted them as a reasonable design that would likely be adopted if the same functionality was required.

:::tip
A [Custom MAVLink Message](../mavlink/custom_messages.md) is one that isn't part of the standard.
These are defined in your own XML as part of your own fork of PX4.
If you use [custom MAVLink messages](../mavlink/custom_messages.md) you will need maintain the definitions in PX4, your ground station, and any other SDKs that communicate with it.
Загалом, щоб зменшити тягар обслуговування, слід використовувати (або доповнювати) стандартні визначення, якщо це можливо.
:::

New standard definitions are added first to `development.xml`, and then moved to `common.xml` following review and prototyping, and acceptance by the MAVLink team.

If you intend your message to become part of the default PX4 build you will need to propose it to the MAVLink community by submitting a pull request (PR) to [development.xml](https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/development.xml).
The [MAVLink Developer guide](https://mavlink.io/en/getting_started/) explains how to define new messages in [How to Define MAVLink Messages & Enums](https://mavlink.io/en/guide/define_xml_element.html).

## Generating Message Headers

During development you can add your definitions to `PX4-Autopilot/src/modules/mavlink/mavlink/message_definitions/v1.0/development.xml` (or pull them from MAVLink).

When you build PX4, header files for these message definitions are generated in the build directory (`/build/<build target>/mavlink/`).
If headers are not build for your messages, they may be incorrectly formatted, or use clashing ids.
Перевірте журнал збірки для отримання інформації.

## Implementing Message Senders/Receivers

Once the message headers for your definitions are generated in the PX4 build, you can use them in your code to send and receive the messages:

- [Streaming MAVLink Messages](../mavlink/streaming_messages.md)
- [Receiving MAVLink Messages](../mavlink/receiving_messages.md)

## Тестування

The first step in debugging is to confirm that any messages you've created are being sent/received as you expect.

You should should first use the `uorb top [<message_name>]` command to verify in real-time that your message is published and the rate (see [uORB Messaging](../middleware/uorb.md#uorb-top-command)).
This approach can also be used to test incoming messages that publish a uORB topic (for other messages you might use `printf` in your code and test in SITL).

Існує кілька підходів для перегляду трафіку MAVLink:

- Create a [Wireshark MAVLink plugin](https://mavlink.io/en/guide/wireshark.html) for your dialect.
  This allows you to inspect MAVLink traffic on an IP interface - for example between _QGroundControl_ or MAVSDK and your real or simulated version of PX4.

  :::tip
  It is much easier to generate a wireshark plugin and inspect traffic in Wireshark, than to rebuild QGroundControl with your dialect and use MAVLink Inspector.

:::

- [Log uORB topics](../dev_log/logging.md) associate with your MAVLink message.

- View received messages in the QGroundControl [MAVLink Inspector](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/mavlink_inspector.html).
  You will need to [rebuild QGroundControl with the new message definitions](#updating-ground-stations).

### Встановити швидкість передачі за допомогою оболонки

Для тестування іноді корисно збільшити швидкість передачі окремих тем під час виконання (наприклад, для перевірки в QGC).
This can be achieved using by calling the [mavlink](../modules/modules_communication.md#mavlink) module through the [QGC MAVLink console](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/analyze_view/mavlink_console.html) (or some other shell):

```sh
mavlink stream -u <port number> -s <mavlink topic name> -r <rate>
```

You can get the port number with `mavlink status` which will output (amongst others) `transport protocol: UDP (<port number>)`.
Прикладом може бути:

```sh
mavlink stream -u 14556 -s CA_TRAJECTORY -r 300
```

## Оновлення наземних станцій

Зрештою, ви захочете використовувати ваш новий інтерфейс MAVLink, надавши відповідну наземну станцію або реалізацію MAVSDK.

Важливо пам'ятати, що MAVLink вимагає, щоб ви використовували версію бібліотеки, яка побудована за тим самим визначенням (XML-файл).
Отже, якщо ви створили власне повідомлення у PX4, ви не зможете його використати, доки не зберете QGC або MAVSDK з тим самим визначенням.

### Оновлення QGroundControl

You will need to [Build QGroundControl](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/getting_started/index.html) including a pre-built C library that contains your custom messages.

QGC uses a pre-built C library that must be located at [/qgroundcontrol/libs/mavlink/include/mavlink](https://github.com/mavlink/qgroundcontrol/tree/master/libs/mavlink/include/mavlink) in the QGC source.

By default this is pre-included as a submodule from <https://github.com/mavlink/c_library_v2> but you can [generate your own MAVLink Libraries](https://mavlink.io/en/getting_started/generate_libraries.html).

QGC uses the **all.xml** dialect by default, which includes **common.xml**.
You can include your messages in either file.

Note that if you use your own _custom dialect_ then it should include **ArduPilotMega.xml** (or it will miss all the existing messages), and you will need to change the dialect used by setting it in [`MAVLINK_CONF`](https://github.com/mavlink/qgroundcontrol/blob/master/QGCExternalLibs.pri#L52) when running _qmake_.

### Оновлення MAVSDK

See the MAVSDK docs for information about how to work with [MAVLink headers and dialects](https://mavsdk.mavlink.io/main/en/cpp/guide/build.html).
