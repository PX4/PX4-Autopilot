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

External modules can register custom MAVLink dialect XML files for mavgen code generation without modifying PX4 source.

### Registering a Dialect

Call `px4_add_external_mavlink_dialect()` from your module's `CMakeLists.txt`:

```cmake
px4_add_external_mavlink_dialect(
    XML ${CMAKE_CURRENT_SOURCE_DIR}/../../mavlink/my_dialect.xml
)
```

The dialect XML must `<include>common.xml</include>` so that all standard MAVLink messages remain available.
The function copies the XML into mavgen's search path and, if `CONFIG_MAVLINK_DIALECT` is `common`, automatically overrides it with your dialect name.

Multiple external dialects from different modules are supported.

### Directory Layout

```
my_external_module/
├── mavlink/
│   └── my_dialect.xml      # Custom MAVLink dialect
├── src/
│   ├── CMakeLists.txt       # Calls px4_add_external_mavlink_dialect()
│   └── modules/
│       └── my_module/
```

## External MAVLink Message Handlers and Streams

External modules can register callbacks for custom inbound and outbound MAVLink messages at runtime, without patching `mavlink_receiver.cpp` or `mavlink_main.cpp`.

### Inbound Message Handlers

Register a handler for a custom message ID from your module's `init` or `task_spawn`:

```cpp
#include <modules/mavlink/mavlink_ext_handler.h>

static bool handle_my_message(const mavlink_message_t *msg, void *user_data)
{
    // Decode and process message
    return true;
}

// Registration (typically in module init)
mavlink_ext_handler_register(MAVLINK_MSG_ID_MY_MESSAGE, handle_my_message, this);

// Cleanup (module stop)
mavlink_ext_handler_unregister(MAVLINK_MSG_ID_MY_MESSAGE);
```

Registered handlers are invoked from the MAVLink receiver thread's `default` switch case.
Registration is mutex-protected; dispatch is lock-free.

### Outbound Streams

Register a stream callback to periodically emit custom messages:

```cpp
#include <modules/mavlink/mavlink_ext_stream.h>

static bool emit_my_message(uint8_t channel, void *user_data)
{
    mavlink_my_message_t msg{};
    // Fill message fields...
    mavlink_msg_my_message_send_struct((mavlink_channel_t)channel, &msg);
    return true;
}

// Register with rate limiting (500000 = 2 Hz)
mavlink_ext_stream_register(MAVLINK_MSG_ID_MY_MESSAGE, "MY_MESSAGE",
                            emit_my_message, this, 500000);
```

The `interval_us` parameter controls rate limiting:
- `-1`: unlimited (fire every iteration)
- `0`: disabled
- `>0`: minimum microseconds between sends

External stream rates can also be controlled at runtime via the standard MAVLink `SET_MESSAGE_INTERVAL` command from QGC or pymavlink:

```cpp
// Programmatic rate change
mavlink_ext_stream_set_interval(MAVLINK_MSG_ID_MY_MESSAGE, 1000000); // 1 Hz
```

## Boot-Time Auto-Start

External modules can declare startup commands that are baked into the firmware ROMFS image at build time, eliminating the need for manual SD card `extras.txt` files.

### Setup

Create `init/rc.ext_modules` in your external module directory:

```sh
#!/bin/sh
my_driver start
my_mavlink_bridge start
```

When building with `EXTERNAL_MODULES_LOCATION`, PX4's build system automatically copies this file into the ROMFS.
At boot, `rcS` sources it after `rc.board_extras` and before the SD card `extras.txt`.

### Boot Order

```
rcS boot sequence:
├── rc.board_extras           # Board-specific init
├── extras.txt                # SD card overrides (runtime)
├── rc.logging                # Logger start
└── rc.ext_modules            # External module auto-start (ROMFS, build-time)
```

External modules run after the logger, ensuring that any slow hardware initialization (e.g. I2C secure elements) doesn't delay flight logging in a brownout recovery scenario.
The SD card `extras.txt` remains available as a runtime override for development and testing without reflashing.
