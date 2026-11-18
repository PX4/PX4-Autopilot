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

## Out-of-Tree MAVLink Message Handlers and Streams

An external module can add MAVLink message handlers and streams to the [mavlink module](../modules/modules_communication.md#mavlink) without changing its sources, by calling `px4_add_mavlink_extension()` from its `CMakeLists.txt`:

```cmake
px4_add_mavlink_extension(
	HANDLER_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/MyHandler.hpp
	HANDLERS MyHandler
	STREAM_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/MY_MESSAGE.hpp
	STREAMS MavlinkStreamMyMessage
	SRCS ${CMAKE_CURRENT_SOURCE_DIR}/MyHandler.cpp
	)
```

- Each handler class gets one instance per MAVLink instance, constructed from that instance's `Mavlink &`.
  Its `void handle_message(const mavlink_message_t *msg)` receives every message the instance receives, after the mavlink module's own handlers.
  `HANDLER_HEADER` is included where `Mavlink` is only declared, so define the handler's methods in `SRCS`, which can include `mavlink_main.h`.
- Each stream class is written like the streams in [src/modules/mavlink/streams](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/mavlink/streams), and joins the list of streams after the module's own.
  The `mavlink stream` command and `MAV_CMD_SET_MESSAGE_INTERVAL` reach it by name or message ID; give it a name and message ID no in-tree stream uses.
- The messages must be in the dialect PX4 is built with (`CONFIG_MAVLINK_DIALECT`).
- `SRCS` are compiled as part of the mavlink module, with their own directory and the directories of the two headers on its include path (`INCLUDES` adds others).
  On NuttX the module is built as a single file (a unity build), so file-static and anonymous-namespace names in `SRCS`, and header names, must not collide with the module's own or another extension's.

The mavlink module reads the registrations when it is configured, which is after every external module.
`src/modules/mavlink/extension_example` is a complete example, built with `CONFIG_MAVLINK_EXTENSION_EXAMPLE`.

## Building External Modules and uORB Messages

Execute `make px4_sitl EXTERNAL_MODULES_LOCATION=<path>`.

Any other build target can be used, but the build directory must not yet exist.
If it already exists, you can also just set the _cmake_ variable in the build folder.

For subsequent incremental builds `EXTERNAL_MODULES_LOCATION` does not need to be specified.
