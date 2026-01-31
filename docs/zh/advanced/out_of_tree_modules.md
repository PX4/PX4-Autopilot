# 外部模块（Out-of-Tree）

外部模块为开发人员提供了一种便捷的机制，可以管理/分组他们想要添加（或更新）PX4 固件的专有模块。
外部模块可以使用与内部模块相同的includes，并可以通过uORB与内部模块交互。

本主题说明如何将外部（“out of tree”）模块添加到 PX4 编译中。

:::tip
We encourage you to contribute your changes into PX4, where possible!
:::

## 用法

要创建外部模块：

- Create an _external directory_ folder for grouping the external modules:
  - This can be located anywhere outside of the **PX4-Autopilot** tree.
  - It must have the same structure as **PX4-Autopilot** (i.e. it must contain a directory called **src**).
  - Later we refer to this directory using `EXTERNAL_MODULES_LOCATION`.

- Copy an existing module (e.g. **examples/px4_simple_app**) to the external directory, or directly create a new module.

- Rename the module (including `MODULE` in **CMakeLists.txt**) or remove it from the existing PX4-Autopilot _cmake_ build config.
  这是为了避免与内部模块发生冲突。

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

## Out-of-Tree uORB 消息定义

树外uORB消息将在与正常uORB消息相同的位置生成。 For this, the `$EXTERNAL_MODULES_LOCATION/msg` folder must exist.

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

树外 uORB 消息将在与正常 uORB 消息相同的位置生成。
The uORB topic headers are generated in `<build_dir>/uORB/topics/`, and the message source files are
generated in `<build_dir>/msg/topics_sources/`.

The new uORB messages can be used like any other uORB message as described [here](../middleware/uorb.md#adding-a-new-topic).

:::warning
The out-of-tree uORB message definitions cannot have the same name as any of the normal uORB messages.
:::

## 构建外部模块和 uORB 消息

Execute `make px4_sitl EXTERNAL_MODULES_LOCATION=<path>`.

任何其他构建目标都可以使用，但构建目录尚不存在。
If it already exists, you can also just set the _cmake_ variable in the build folder.

For subsequent incremental builds `EXTERNAL_MODULES_LOCATION` does not need to be specified.
