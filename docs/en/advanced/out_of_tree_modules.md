# External Modules (Out-of-Tree)

External modules provide a convenient mechanism for developers to manage/group proprietary modules that they want add to (or update in) PX4 firmware.
External modules can use the same includes as internal modules and can interact with internal modules via uORB.

This topic explains how to add an external ("out of tree") module to the PX4 build.

:::tip
We encourage you to contribute your changes into PX4, where possible!
:::

## Usage

To create an external module:

- Create an _external directory_ folder for grouping the external modules:
  - This can be located anywhere outside of the **PX4-Autopilot** tree.
  - It must have the same structure as **PX4-Autopilot** (i.e. it must contain a directory called **src**).
  - Later we refer to this directory using `EXTERNAL_MODULES_LOCATION`.
- Copy an existing module (e.g. **examples/px4_simple_app**) to the external directory, or directly create a new module.
- Rename the module (including `MODULE` in **CMakeLists.txt**) or remove it from the existing PX4-Autopilot _cmake_ build config.
  This is to avoid conflicts with internal modules.
- Add a file **CMakeLists.txt** in the external directory with content:

  ```cmake
  set(config_module_list_external
      modules/<new_module>
      PARENT_SCOPE
      )
  ```

- Add a line `EXTERNAL` to the `modules/<new_module>/CMakeLists.txt` within
  `px4_add_module()`, for example like this:

  ```cmake
  px4_add_module(
  	MODULE modules__test_app
  	MAIN test_app
  	STACK_MAIN 2000
  	SRCS
  		px4_simple_app.c
  	DEPENDS
  		platforms__common
  	EXTERNAL
  	)
  ```

## Out-of-Tree uORB Message Definitions

uORB messages can also be defined out-of-tree. For this, the `$EXTERNAL_MODULES_LOCATION/msg` folder must exist.

- Place all new message definitions within the `$EXTERNAL_MODULES_LOCATION/msg` directory.
  The format of these new out-of-tree message definitions are the same as for any other [uORB message definition](../middleware/uorb.md#adding-a-new-topic).
- Add a file `$EXTERNAL_MODULES_LOCATION/msg/CMakeLists.txt` with content:

  ```cmake
  set(config_msg_list_external
      <message1>.msg
      <message2>.msg
      <message3>.msg
      PARENT_SCOPE
      )
  ```

  where `<message#>.msg` is the name of the uORB message definition file to be processed and used for uORB message generation.

The out-of-tree uORB messages will be generated in the same locations as the normal uORB messages.
The uORB topic headers are generated in `<build_dir>/uORB/topics/`, and the message source files are
generated in `<build_dir>/msg/topics_sources/`.

The new uORB messages can be used like any other uORB message as described [here](../middleware/uorb.md#adding-a-new-topic).

:::warning
The out-of-tree uORB message definitions cannot have the same name as any of the normal uORB messages.
:::

## Building External Modules and uORB Messages

Execute `make px4_sitl EXTERNAL_MODULES_LOCATION=<path>`.

Any other build target can be used, but the build directory must not yet exist.
If it already exists, you can also just set the _cmake_ variable in the build folder.

For subsequent incremental builds `EXTERNAL_MODULES_LOCATION` does not need to be specified.

## Out-of-Tree MAVLink Dialect Definitions

An external module can make its own MAVLink dialect the dialect of the build, without touching the PX4 source tree.

Call `px4_add_external_mavlink_dialect()` from a `CMakeLists.txt` under `$EXTERNAL_MODULES_LOCATION/src`:

```cmake
px4_add_external_mavlink_dialect(XML ${CMAKE_CURRENT_SOURCE_DIR}/../../../mavlink/my_dialect.xml)
```

- The XML must `<include>common.xml</include>` (or another upstream dialect) so the standard messages stay available.
- The XML is copied next to a copy of the upstream definitions in `<build_dir>/mavlink/message_definitions/v1.0/`, where mavgen resolves its includes; nothing is written into the source tree.
- The dialect replaces `CONFIG_MAVLINK_DIALECT`. Include the dialect the target would otherwise build (`common.xml` for flight controllers, `development.xml` for `px4_sitl_default`) to keep its messages.
- One external dialect per build. Messages from a second module belong in an XML that the registered dialect includes.

Modules that use the generated headers call `px4_target_use_external_mavlink_dialect(<target>)` after `px4_add_module()`.
It adds the include paths, the mavgen dependency and the warning suppressions the generated code needs.

```cmake
px4_add_module(
	MODULE modules__my_module
	MAIN my_module
	SRCS my_module.cpp
	EXTERNAL
)
px4_target_use_external_mavlink_dialect(modules__my_module)
```

External modules are configured before the in-tree libraries, so link in-tree libraries with `target_link_libraries()` rather than `DEPENDS`.

## External MAVLink Message Handlers and Streams

External modules register callbacks for custom inbound and outbound messages at runtime.
The registries are compiled into the `mavlink` module only when `EXTERNAL_MODULES_LOCATION` is set, so firmware without external modules is unchanged.

### Inbound Message Handlers

```cpp
#include <modules/mavlink/mavlink_bridge_header.h>
#include <modules/mavlink/mavlink_ext_handler.h>

static bool handle_my_message(const mavlink_message_t *msg, void *user_data)
{
    mavlink_my_message_t decoded;
    mavlink_msg_my_message_decode(msg, &decoded);
    // ...
    return true;
}

// module init
mavlink_ext_handler_register(MAVLINK_MSG_ID_MY_MESSAGE, handle_my_message, this);

// module stop
mavlink_ext_handler_unregister(MAVLINK_MSG_ID_MY_MESSAGE);
```

The receiver thread of the instance that received the message calls the handler for every message ID that `mavlink_receiver.cpp` does not handle itself.

### Outbound Streams

```cpp
#include <modules/mavlink/mavlink_ext_stream.h>

static bool send_my_message(uint8_t channel, void *user_data)
{
    mavlink_my_message_t msg{};
    // ...
    mavlink_msg_my_message_send_struct((mavlink_channel_t)channel, &msg);
    return true;
}

// 2 Hz on every link
mavlink_ext_stream_register(MAVLINK_MSG_ID_MY_MESSAGE, send_my_message, this, 500000);
```

Each mavlink instance calls the stream callback from its main loop, after the built-in streams, and rate limits it per link:

- `interval_us > 0`: minimum spacing in microseconds
- `MAVLINK_EXT_STREAM_UNLIMITED`: every loop iteration
- `MAVLINK_EXT_STREAM_DISABLED`

A GCS changes the rate of an external stream on its own link with `SET_MESSAGE_INTERVAL`, exactly like a built-in stream (`-1` stops it, `0` restores the registered interval).
The module can do the same with `mavlink_ext_stream_set_interval(channel, msg_id, interval_us)`.

### One-shot Messages

To send a message or command once, on every running link, from any module thread or from inside a handler:

```cpp
static bool send_reply(uint8_t channel, void *user_data)
{
    mavlink_msg_my_reply_send_struct((mavlink_channel_t)channel, static_cast<mavlink_my_reply_t *>(user_data));
    return true;
}

mavlink_my_reply_t reply{};
// ...
mavlink_ext_send(send_reply, &reply);
```

### Callback Rules

- Callbacks run with the registry mutex held. `unregister()` therefore returns only after any in-flight callback has completed, which makes it safe to free `user_data` (typically the module) afterwards.
- Never register or unregister from inside a callback.
- Never call `mavlink_ext_send()` from a stream callback: it runs under that instance's send lock and would take other instances' send locks in the wrong order. Stream callbacks already receive their channel.
- Keep callbacks short; they run on the mavlink threads.

### Example

`test/external_module` is a complete external module with a dialect, one handler, one stream, one-shot replies and an init script.
CI builds SITL with it and drives it with `test/external_module/test_ext_mavlink_example.py`:

```sh
make px4_sitl_default EXTERNAL_MODULES_LOCATION=$(pwd)/test/external_module
test/external_module/test_ext_mavlink_example.py --verbose
```

## Boot-Time Auto-Start

`$EXTERNAL_MODULES_LOCATION/init/rc.ext_modules`, if present, is copied into the ROMFS as `/etc/init.d/rc.ext_modules` and sourced by `rcS` after the logger starts, on NuttX and in SITL:

```sh
#!/bin/sh
my_driver start
my_mavlink_bridge start
```

Starting external modules after the logger keeps slow hardware initialization (for example an I2C secure element) from delaying flight logging.
The SD card `extras.txt` stays available as a runtime override.
