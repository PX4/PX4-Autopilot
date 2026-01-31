# 외부 모듈(별도)

외부 모듈은 개발자가 PX4 펌웨어에 추가(또는 업데이트)하려는 독점 모듈을 관리/그룹화할 수 있는 편리한 메커니즘을 제공합니다.
외부 모듈은 내부 모듈과 같이 사용할 수 있으며, uORB로 내부 모듈과 상호 작용할 수 있습니다.

PX4 빌드에 외부("out of tree") 모듈을 추가하는 방법을 설명합니다.

:::tip
We encourage you to contribute your changes into PX4, where possible!
:::

## 사용법

외부 모듈을 만들려면:

- Create an _external directory_ folder for grouping the external modules:
  - This can be located anywhere outside of the **PX4-Autopilot** tree.
  - It must have the same structure as **PX4-Autopilot** (i.e. it must contain a directory called **src**).
  - Later we refer to this directory using `EXTERNAL_MODULES_LOCATION`.

- Copy an existing module (e.g. **examples/px4_simple_app**) to the external directory, or directly create a new module.

- Rename the module (including `MODULE` in **CMakeLists.txt**) or remove it from the existing PX4-Autopilot _cmake_ build config.
  이것은 내부 모듈과의 충돌을 피하기 위한 것입니다.

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

## 별도 uORB 메시지 정의

uORB 메시지는 트리 외부에서 정의할 수 있습니다. For this, the `$EXTERNAL_MODULES_LOCATION/msg` folder must exist.

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

외부 uORB 메시지는 일반 uORB 메시지와 동일한 위치에 생성됩니다.
The uORB topic headers are generated in `<build_dir>/uORB/topics/`, and the message source files are
generated in `<build_dir>/msg/topics_sources/`.

The new uORB messages can be used like any other uORB message as described [here](../middleware/uorb.md#adding-a-new-topic).

:::warning
The out-of-tree uORB message definitions cannot have the same name as any of the normal uORB messages.
:::

## 외부 모듈 및 uORB 메시지 빌드

Execute `make px4_sitl EXTERNAL_MODULES_LOCATION=<path>`.

다른 빌드 대상을 사용할 수 있지만, 빌드 디렉토리가 아직 존재하지 않아야 합니다.
If it already exists, you can also just set the _cmake_ variable in the build folder.

For subsequent incremental builds `EXTERNAL_MODULES_LOCATION` does not need to be specified.
