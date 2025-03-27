# Повідомлення MAVLink

[MAVLink](https://mavlink.io/en/) is a very lightweight messaging protocol that has been designed for the drone ecosystem.

PX4 uses _MAVLink_ to communicate with ground stations and MAVLink SDKs, such as _QGroundControl_ and [MAVSDK](https://mavsdk.mavlink.io/), and as the integration mechanism for connecting to drone components outside of the flight controller: companion computers, MAVLink enabled cameras, and so on.

Ця тема надає короткий огляд основних концепцій MAVLink, таких як повідомлення, команди та мікросервіси.
Він також надає інструкції посібника про те, як ви можете додати підтримку PX4 для:

- Потокових повідомлень MAVLink
- Обробка вхідних повідомлень MAVLink та запис до теми uORB.

:::info
The topic does not cover _command_ handling and sending, or how to implement your own microservices.
:::

## Огляд MAVLink

MAVLink - це легкий протокол, який був розроблений для ефективної відправки повідомлень по ненадійним радіоканалах з низькою пропускною здатністю.

_Messages_ are simplest and most "fundamental" definition in MAVLink, consisting of a name (e.g. [ATTITUDE](https://mavlink.io/en/messages/common.html#ATTITUDE)), id, and fields containing relevant data.
Вони навмисно легкі, мають обмежений розмір і не мають семантики для повторного надсилання та підтвердження.
Окремі повідомлення зазвичай використовуються для потокової передачі телеметрії або інформації про стан, а також для надсилання команд, які не потребують підтвердження - наприклад, команд уставки, що надсилаються з високою швидкістю.

The [Command Protocol](https://mavlink.io/en/services/command.html) is a higher level protocol for sending commands that may need acknowledgement.
Specific commands are defined as values of the [MAV_CMD](https://mavlink.io/en/messages/common.html#mav_commands) enumeration, such as the takeoff command [MAV_CMD_NAV_TAKEOFF](https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_TAKEOFF), and include up to 7 numeric "param" values.
The protocol sends a command by packaging the parameter values in a `COMMAND_INT` or `COMMAND_LONG` message, and waits for an acknowledgement with a result in a `COMMAND_ACK`.
Якщо команда не буде отримана, вона буде повторно надіслана автоматично.
Note that [MAV_CMD](https://mavlink.io/en/messages/common.html#mav_commands) definitions are also used to define mission actions, and that not all definitions are supported for use in commands/missions on PX4.

[Microservices](https://mavlink.io/en/services/) are other higher level protocols built on top of MAVLink messages.
Вони використовуються для передачі інформації, яку неможливо надіслати одним повідомленням, а також для забезпечення таких функцій, як надійний зв'язок.
Описаний вище командний протокол є одним з таких сервісів.
Others include the [File Transfer Protocol](https://mavlink.io/en/services/ftp.html), [Camera Protocol](https://mavlink.io/en/services/camera.html) and [Mission Protocol](https://mavlink.io/en/services/mission.html).

MAVLink messages, commands and enumerations are defined in [XML definition files](https://mavlink.io/en/guide/define_xml_element.html).
Інструментарій MAVLink включає в себе генератори коду, які створюють з цих визначень специфічні для мови програмування бібліотеки для надсилання та отримання повідомлень.
Зверніть увагу, що більшість згенерованих бібліотек не створюють код для реалізації мікросервісів.

The MAVLink project standardizes a number of messages, commands, enumerations, and microservices, for exchanging data using the following definition files (note that higher level files _include_ the definitions of the files below them):

- [development.xml](https://mavlink.io/en/messages/development.html) — Definitions that are proposed to be part of the standard.
 The definitions move to `common.xml` if accepted following testing.
- [common.xml](https://mavlink.io/en/messages/common.html) — A "library" of definitions meeting many common UAV use cases.
 Вони підтримуються багатьма польотними стеками, наземними станціями та периферійними пристроями MAVLink.
 Польотні стеки, які використовують ці визначення, з більшою ймовірністю будуть взаємодіяти.
- [standard.xml](https://mavlink.io/en/messages/standard.html) — Definitions that are actually standard.
 Вони присутні на переважній більшості польотних стеків і реалізовані однаково.
- [minimal.xml](https://mavlink.io/en/messages/minimal.html) — Definitions required by a minimal MAVLink implementation.

The project also hosts [dialect XML definitions](https://mavlink.io/en/messages/#dialects), which contain MAVLink definitions that are specific to a flight stack or other stakeholder.

Протокол покладається на те, що кожна сторона комунікації має спільне визначення того, які повідомлення надсилаються.
Це означає, що для того, щоб взаємодіяти, обидва кінці комунікації повинні використовувати бібліотеки, створені на основі одного і того ж визначення XML.

<!--
The messages are sent over-the-wire in the "payload" of a [MAVLink packet](https://mavlink.io/en/guide/serialization.html#mavlink2_packet_format).
In order to reduce the amount of information that must be sent, the packet does not include the message metadata, such as what fields are in the message and so on.
Instead, the fields are serialized in a predefined order based on data size and XML definition order, and MAVLink relies on each end of the communication having a shared definition of what messages are being sent.
The shared identity of the message is conveyed by the message id, along with a CRC ("`CRC_EXTRA`") that uniquely identifies the message based on its name and id, and the field names and types.
The receiving end of the communication will discard any packet for which the message id and the `CRC_EXTRA` do not match.
-->

## PX4 та MAVLink

PX4 releases build `common.xml` MAVLink definitions by default, for the greatest compatibility with MAVLink ground stations, libraries, and external components such as MAVLink cameras.
In the `main` branch, these are included from `development.xml` on SITL, and `common.xml` for other boards.

:::info
To be part of a PX4 release, any MAVLink definitions that you use must be in `common.xml` (or included files such as `standard.xml` and `minimal.xml`).
During development you can use definitions in `development.xml`.
You will need to work with the [MAVLink team](https://mavlink.io/en/contributing/contributing.html) to define and contribute these definitions.
:::

PX4 includes the [mavlink/mavlink](https://github.com/mavlink/mavlink) repo as a submodule under [/src/modules/mavlink](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mavlink).
This contains XML definition files in [/mavlink/messages/1.0/](https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/).

Інструментарій збірки генерує заголовні файли MAVLink 2 C під час збірки.
The XML file for which headers files are generated may be defined in the [PX4 kconfig board configuration](../hardware/porting_guide_config.md#px4-board-configuration-kconfig) on a per-board basis, using the variable `CONFIG_MAVLINK_DIALECT`:

- For SITL `CONFIG_MAVLINK_DIALECT` is set to `development` in [boards/px4/sitl/default.px4board](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/sitl/default.px4board#L36).
 You can change this to any other definition file, but the file must include `common.xml`.
- For other boards `CONFIG_MAVLINK_DIALECT` is not set by default, and PX4 builds the definitions in `common.xml` (these are build into the [mavlink module](../modules/modules_communication.md#mavlink) by default — search for `menuconfig MAVLINK_DIALECT` in [src/modules/mavlink/Kconfig](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/Kconfig#L10)).

The files are generated into the build directory: `/build/<build target>/mavlink/`.

## Спеціальні повідомлення MAVLink

Користувацьке повідомлення MAVLink - це повідомлення, якого немає у визначеннях за замовчуванням, включених до PX4.

:::info
If you use a custom definition you will need maintain the definition in PX4, your ground station, and any other SDKs that communicate with it.
Загалом, щоб зменшити тягар обслуговування, слід використовувати (або доповнювати) стандартні визначення, якщо це можливо.
:::

Користувацькі визначення можна додати до нового файлу діалекту у тому самому каталозі, що й стандартні визначення XML.
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

## Потокові повідомлення MAVLink

MAVLink messages are streamed using a streaming class, derived from `MavlinkStream`, that has been added to the PX4 stream list.
Клас має фреймворкові методи, які ви реалізуєте, щоб PX4 міг отримати потрібну йому інформацію зі згенерованого визначення повідомлення MAVLink.
It also has a `send()` method that is called each time the message needs to be sent — you override this to copy information from a uORB subscription to the MAVLink message object that is to be sent.

Цей посібник демонструє, як транслювати повідомлення uORB як повідомлення MAVLink, і застосовується як до стандартних, так і до користувацьких повідомлень.

### Передумови

Generally you will already have a [uORB](../middleware/uorb.md) message that contains information you'd like to stream and a definition of a MAVLink message that you'd like to stream it with.

For this example we're going to assume that you want to stream the (existing) [BatteryStatus](../msg_docs/BatteryStatus.md) uORB message to a new MAVLink battery status message, which we will name `BATTERY_STATUS_DEMO`.

Copy this `BATTERY_STATUS_DEMO` message into the message section of `development.xml` in your PX4 source code, which will be located at: `\src\modules\mavlink\mavlink\message_definitions\v1.0\development.xml`.

```xml
    <message id="11514" name="BATTERY_STATUS_DEMO">
      <description>Simple demo battery.</description>
      <field type="uint8_t" name="id" instance="true">Battery ID</field>
      <field type="int16_t" name="temperature" units="cdegC" invalid="INT16_MAX">Temperature of the whole battery pack (not internal electronics). INT16_MAX field not provided.</field>
      <field type="uint8_t" name="percent_remaining" units="%" invalid="UINT8_MAX">Remaining battery energy. Values: [0-100], UINT8_MAX: field not provided.</field>
    </message>
```

:::info
Note that this is a cut-down version of the not-yet-implemented [BATTERY_STATUS_V2](https://mavlink.io/en/messages/development.html#BATTERY_STATUS_V2) message with randomly chosen unused id of `11514`.
Here we've put the message in `development.xml`, which is fine for testing and if the message is intended to eventually be part of the standard message set, but you might also put a [custom message](#custom-mavlink-messages) in its own dialect file.
:::

Build PX4 for SITL and confirm that the associated message is generated in `/build/px4_sitl_default/mavlink/development/mavlink_msg_battery_status_demo.h`.

Because `BatteryStatus` already exists you will not need to do anything to create or build it.

### Об'явлення класу потокового відтворення

First create a file named `BATTERY_STATUS_DEMO.hpp` for your streaming class (named after the message to stream) inside the [/src/modules/mavlink/streams](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mavlink/streams) directory.

Додайте заголовки для повідомлень uORB у верхню частину файлу (необхідні заголовки MAVLink вже мають бути доступними):

```cpp
#include <uORB/topics/battery_status.h>
```

:::info
The uORB topic's snake-case header file is generated from the CamelCase uORB filename at build time.
:::

Потім скопіюйте визначення класу трансляції нижче у файл:

```cpp
class MavlinkStreamBatteryStatusDemo : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new MavlinkStreamBatteryStatusDemo(mavlink);
    }
    const char *get_name() const
    {
        return MavlinkStreamBatteryStatusDemo::get_name_static();
    }
    static const char *get_name_static()
    {
        return "BATTERY_STATUS_DEMO";
    }
    static uint16_t get_id_static()
    {
        return MAVLINK_MSG_ID_BATTERY_STATUS_DEMO;
    }
    uint16_t get_id()
    {
        return get_id_static();
    }
    unsigned get_size()
    {
        return MAVLINK_MSG_ID_BATTERY_STATUS_DEMO_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

private:
    //Subscription to array of uORB battery status instances
    uORB::SubscriptionMultiArray<battery_status_s, battery_status_s::MAX_INSTANCES> _battery_status_subs{ORB_ID::battery_status};
    // SubscriptionMultiArray subscription is needed because battery has multiple instances.
    // uORB::Subscription is used to subscribe to a single-instance topic

    /* do not allow top copying this class */
    MavlinkStreamBatteryStatusDemo(MavlinkStreamBatteryStatusDemo &);
    MavlinkStreamBatteryStatusDemo& operator = (const MavlinkStreamBatteryStatusDemo &);

protected:
    explicit MavlinkStreamBatteryStatusDemo(Mavlink *mavlink) : MavlinkStream(mavlink)
    {}

	bool send() override
	{
		bool updated = false;

		// Loop through _battery_status_subs (subscription to array of BatteryStatus instances)
		for (auto &battery_sub : _battery_status_subs) {
            // battery_status_s is a struct that can hold the battery object topic
			battery_status_s battery_status;

			// Update battery_status and publish only if the status has changed
			if (battery_sub.update(&battery_status)) {
                // mavlink_battery_status_demo_t is the MAVLink message object
				mavlink_battery_status_demo_t bat_msg{};

				bat_msg.id = battery_status.id - 1;
				bat_msg.percent_remaining = (battery_status.connected) ? roundf(battery_status.remaining * 100.f) : -1;

				// check if temperature valid
				if (battery_status.connected && PX4_ISFINITE(battery_status.temperature)) {
					bat_msg.temperature = battery_status.temperature * 100.f;
				} else {
					bat_msg.temperature = INT16_MAX;
				}

                //Send the message
				mavlink_msg_battery_status_demo_send_struct(_mavlink->get_channel(), &bat_msg);
				updated = true;
			}
		}

		return updated;
	}

};
```

Most streaming classes are very similar (see examples in [/src/modules/mavlink/streams](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mavlink/streams)):

- The streaming class derives from [`MavlinkStream`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_stream.h) and is named using the pattern `MavlinkStream<CamelCaseMessageName>`.

- The `public` definitions are "near-boilerplate", allowing PX4 to get an instance of the class (`new_instance()`), and then to use it to fetch the name, id, and size of the message from the MAVLink headers (`get_name()`, `get_name_static()`, `get_id_static()`, `get_id()`, `get_size()`).
 Для ваших власних потокових класів їх можна просто скопіювати і змінити, щоб вони відповідали значенням для вашого повідомлення MAVLink.

- The `private` definitions subscribe to the uORB topics that need to be published.
 У цьому випадку тема uORB має кілька екземплярів: по одному для кожної батареї.
 We use `uORB::SubscriptionMultiArray` to get an array of battery status subscriptions.

 Тут ми також визначаємо конструктори, щоб уникнути копіювання визначення.

- The `protected` section is where the important work takes place!

 Here we override the `send()` method, copying values from the subscribed uORB topic(s) into appropriate fields in the MAVLink message, and then send the message.

 In this particular example we have an array of uORB instances `_battery_status_subs` (because we have multiple batteries).
 We iterate the array and use `update()` on each subscription to check if the associated battery instance has changed (and update a structure with the current data).
 This allows us to send the MAVLink message _only_ if the associated battery uORB topic has changed:

 ```cpp
 // Struct to hold current topic data.
 battery_status_s battery_status;

 // update() populates battery_status and returns true if the status has changed
 if (battery_sub.update(&battery_status)) {
    // Use battery_status to populate message and send
 }
 ```

 If wanted to send a MAVLink message whether or not the data changed, we could instead use `copy()` as shown:

 ```cpp
 battery_status_s battery_status;
 battery_sub.copy(&battery_status);
 ```

 ::: info
 For a single-instance topic like [VehicleStatus](../msg_docs/VehicleStatus.md) we would subscribe like this:

 ```cpp
 // Create subscription _vehicle_status_sub
 uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
 ```

 І ми можемо використовувати отриману підписку так само, з оновленням або копіюванням.

 ```cpp
 vehicle_status_s vehicle_status{}; // vehicle_status_s is the definition of the uORB topic
 if (_vehicle_status_sub.update(&vehicle_status)) {
   // Use the vehicle_status as it has been updated.
 }
 ```


:::

Next we include our new class in [mavlink_messages.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_messages.cpp#L2193).
Додайте рядок нижче до частини файлу, де включені всі інші потоки:

```cpp
#include "streams/BATTERY_STATUS_DEMO.hpp"
```

Finally append the stream class to the `streams_list` at the bottom of
[mavlink_messages.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_messages.cpp)

```C
StreamListItem *streams_list[] = {
...
#if defined(BATTERY_STATUS_DEMO_HPP)
    create_stream_list_item<MavlinkStreamBatteryStatusDemo>(),
#endif // BATTERY_STATUS_DEMO_HPP
...
}
```

Клас тепер доступний для потокової передачі, але за замовчуванням не буде транслюватися.
Ми розглянемо це в наступних розділах.

### Трансляція за замовчуванням

The easiest way to stream your messages by default (as part of a build) is to add them to [mavlink_main.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_main.cpp) in the appropriate message group.

Якщо ви виконаєте пошук у файлі, то знайдете групи повідомлень, визначені в інструкції switch:

- `MAVLINK_MODE_NORMAL`: Streamed to a GCS.
- `MAVLINK_MODE_ONBOARD`: Streamed to a companion computer on a fast link, such as Ethernet
- `MAVLINK_MODE_ONBOARD_LOW_BANDWIDTH`: Streamed to a companion computer for re-routing to a reduced-traffic link, such as a GCS.
- `MAVLINK_MODE_GIMBAL`: Streamed to a gimbal
- `MAVLINK_MODE_EXTVISION`: Streamed to an external vision system
- `MAVLINK_MODE_EXTVISIONMIN`: Streamed to an external vision system on a slower link
- `MAVLINK_MODE_OSD`: Streamed to an OSD, such as an FPV headset.
- `MAVLINK_MODE_CUSTOM`: Stream nothing by default. Використовується при налаштуванні потокового передавання за допомогою MAVLink.
- `MAVLINK_MODE_MAGIC`: Same as `MAVLINK_MODE_CUSTOM`
- `MAVLINK_MODE_CONFIG`: Streaming over USB with higher rates than `MAVLINK_MODE_NORMAL`.
- `MAVLINK_MODE_MINIMAL`: Stream a minimal set of messages. Зазвичай використовується для поганого зв'язку телеметрії.
- `MAVLINK_MODE_IRIDIUM`: Streamed to an iridium satellite phone

Normally you'll be testing on a GCS, so you could just add the message to the `MAVLINK_MODE_NORMAL` case using the `configure_stream_local()` method.
Наприклад, для трансляції CA_TRAJECTORY з частотою 5 Гц:

```cpp
	case MAVLINK_MODE_CONFIG: // USB
		// Note: streams requiring low latency come first
		...
		configure_stream_local("BATTERY_STATUS_DEMO", 5.0f);
        ...
```

It is also possible to add a stream by calling the [mavlink](../modules/modules_communication.md#mavlink) module with the `stream` argument in a [startup script](../concept/system_startup.md).
For example, you might add the following line to [/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink) in order to stream `BATTERY_STATUS_DEMO` at 50Hz on UDP port `14556` (`-r` configures the streaming rate and `-u` identifies the MAVLink channel on UDP port 14556).

```sh
mavlink stream -r 50 -s BATTERY_STATUS_DEMO -u 14556
```

### Транслювання за запитом

Деякі повідомлення потрібні лише один раз, при підключенні певного обладнання або за інших обставин.
Щоб уникнути перевантаження каналів зв'язку непотрібними повідомленнями, ви можете не передавати всі повідомлення за замовчуванням, навіть з низькою швидкістю.

If you needed, a GCS or other MAVLink API can request that particular messages are streamed at a particular rate using [MAV_CMD_SET_MESSAGE_INTERVAL](https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL).
A particular message can be requested just once using [MAV_CMD_REQUEST_MESSAGE](https://mavlink.io/en/messages/common.html#MAV_CMD_REQUEST_MESSAGE).

## Отримання повідомлень MAVLink

Цей розділ пояснює, як отримати повідомлення через MAVLink та опублікувати його в uORB.

It assumes that we are receiving the `BATTERY_STATUS_DEMO` message and we want to update the (existing) [BatteryStatus uORB message](../msg_docs/BatteryStatus.md) with the contained information.
Це той тип реалізації, який ви надаєте для підтримки інтеграції батареї MAVLink з PX4.

Add the headers for the uORB topic to publish to in [mavlink_receiver.h](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_receiver.h#L77):

```cpp
#include <uORB/topics/battery_status.h>
```

Add a function signature for a function that handles the incoming MAVLink message in the `MavlinkReceiver` class in
[mavlink_receiver.h](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_receiver.h#L126)

```cpp
void handle_message_battery_status_demo(mavlink_message_t *msg);
```

Normally you would add a uORB publisher for the uORB topic to publish in the `MavlinkReceiver` class in
[mavlink_receiver.h](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_receiver.h#L296).
In this case the [BatteryStatus](../msg_docs/BatteryStatus.md) uORB topic already exists:

```cpp
uORB::Publication<battery_status_s> _battery_pub{ORB_ID(battery_status)};
```

This creates a publication to a single uORB topic instance, which by default will be the _first_ instance.

:::info
This implementation won't work on multi-battery systems, because several batteries might be publishing data to the first instance of the topic, and there is no way to differentiate them.
To support multiple batteries we'd need to use `PublicationMulti` and map the MAVLink message instance IDs to specific uORB topic instances.
:::

Implement the `handle_message_battery_status_demo` function in [mavlink_receiver.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_receiver.cpp).

```cpp
void
MavlinkReceiver::handle_message_battery_status_demo(mavlink_message_t *msg)
{
	if ((msg->sysid != mavlink_system.sysid) || (msg->compid == mavlink_system.compid)) {
		// ignore battery status coming from other systems or from the autopilot itself
		return;
	}

	// external battery measurements
	mavlink_battery_status_t battery_mavlink;
	mavlink_msg_battery_status_decode(msg, &battery_mavlink);

	battery_status_s battery_status{};
	battery_status.timestamp = hrt_absolute_time();

	battery_status.remaining = (float)battery_mavlink.battery_remaining / 100.0f;
	battery_status.temperature = (float)battery_mavlink.temperature;
	battery_status.connected = true;

	_battery_pub.publish(battery_status);
}
```

:::info
Above we only write to the battery fields that are defined in the topic.
На практиці ви оновлювали б всі поля або з дійсними, або з недійсними значеннями: це було скорочено для стислості.
:::

and finally make sure it is called in [MavlinkReceiver::handle_message()](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_receiver.cpp#L228)

```cpp
MavlinkReceiver::handle_message(mavlink_message_t *msg)
 {
    switch (msg->msgid) {
        ...
    case MAVLINK_MSG_ID_BATTERY_STATUS_DEMO:
        handle_message_battery_status_demo(msg);
        break;
        ...
    }
 }
```

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

## Тестування

Як перший крок і під час відлагодження, зазвичай ви просто хочете переконатися, що всі створені вами повідомлення надсилаються/отримуються так, як ви очікуєте.

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
 You will need to rebuild QGroundControl with the custom message definitions, [as described below](#updating-qgroundcontrol)

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

QGC uses the all.xml dialect by default, which includes **common.xml**.
Ви можете додавати свої повідомлення як у файлі, так і у власному діалекті.
However if you use your own dialect then it should include ArduPilotMega.xml (or it will miss all the existing messages), and you will need to change the dialect used by setting it in [`MAVLINK_CONF`](https://github.com/mavlink/qgroundcontrol/blob/master/QGCExternalLibs.pri#L52) when running _qmake_.

### Оновлення MAVSDK

See the MAVSDK docs for information about how to work with [MAVLink headers and dialects](https://mavsdk.mavlink.io/main/en/cpp/guide/build.html).
