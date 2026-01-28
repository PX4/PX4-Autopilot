# Спеціальні повідомлення MAVLink

A custom [MAVLink message](../middleware/mavlink.md) is one that isn't in the standard MAVLink definitions that are included into PX4 by default.

:::info
If you use a custom definition you will fork and maintain PX4, your ground station, and any other SDKs that communicate with it.
Загалом, щоб зменшити тягар обслуговування, слід використовувати (або доповнювати) стандартні визначення, якщо це можливо.
:::

## Adding Custom XML

Custom definitions can be added in a new dialect file in the same directory as [when using the standard XML definitions](../mavlink/adding_messages.md).
For example, create `PX4-Autopilot/src/modules/mavlink/mavlink/message_definitions/v1.0/custom_messages.xml`, and set `CONFIG_MAVLINK_DIALECT` to build the new file for SITL.
This dialect file should include `development.xml` so that all the standard definitions are also included.

For initial prototyping, or if you intend your message to be "standard", you can also add your messages to `common.xml` (or `development.xml`).
Це спрощує збірку, оскільки вам не потрібно модифікувати вже зібраний діалект.

The MAVLink developer guide explains how to define new messages in [How to Define MAVLink Messages & Enums](https://mavlink.io/en/guide/define_xml_element.html).

You can check that your new messages are built by inspecting the headers generated in the build directory (`/build/<build target>/mavlink/`).
Якщо ваші повідомлення не збираються, вони можуть бути неправильно відформатовані або використовувати конфліктуючі ідентифікатори.
Перевірте журнал збірки для отримання інформації.

Після того, як повідомлення створено, ви можете передавати, отримувати або використовувати його в інший спосіб, як описано в наступних розділах.

:::info
The [MAVLink Developer guide](https://mavlink.io/en/getting_started/) has more information about using the MAVLink toolchain.
:::

## Альтернатива створення користувацьких повідомлень MAVLink

Іноді існує потреба в довільному повідомленні MAVLink з вмістом, який не повністю визначений.

Наприклад, при використанні MAVLink для інтерфейсу PX4 з вбудованим пристроєм, повідомлення, якими обмінюються автопілот і пристрій, можуть пройти кілька ітерацій, перш ніж вони будуть стабілізовані.
У цьому випадку відновлення заголовків MAVLink може зайняти багато часу і призвести до помилок, а також переконатися, що обидва пристрої використовують одну і ту ж версію протоколу.

Альтернативним - і тимчасовим - рішенням є перепризначення налагоджувальних повідомлень.
Instead of creating a custom MAVLink message `CA_TRAJECTORY`, you can send a message `DEBUG_VECT` with the string key `CA_TRAJ` and data in the `x`, `y` and `z` fields.
See [this tutorial](../debug/debug_values.md) for an example usage of debug messages.

:::info
This solution is not efficient as it sends character string over the network and involves comparison of strings.
Це повинно використовуватися лише для розробки!
:::

## Testing & Updating Ground Stations

Testing the code and updating ground stations is done in the same way as when [Adding New Standard MAVLink Definitions ](../mavlink/adding_messages.md).
