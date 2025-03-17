# 시스템 전체 재생

It is possible to record and replay arbitrary parts of the system based on ORB messages.

Replay is useful to test the effect of different parameter values based on real data, compare different estimators, etc.

## 준비 사항

The first step is to identify the module or modules that should be replayed.
그런 다음 이 모듈에 대한 모든 입력, 즉 구독된 ORB 주제를 식별합니다.
시스템 전체 재생의 경우 센서, RC 입력, MAVLink 명령 및 파일 시스템과 같은 모든 하드웨어 입력으로 구성됩니다.

All identified topics need to be logged at full rate (see [logging](../dev_log/logging.md)).
For `ekf2` this is already the case with the default set of logged topics.

It is important that all replayed topics contain only a single absolute timestamp, which is the automatically generated field `timestamp`.
Should there be more timestamps, they must be relative to the main timestamp.
For an example, see [SensorCombined.msg](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorCombined.msg).
그 이유는 아래에서 설명합니다.

## 사용법

- First, choose the file to replay and build the target (from within the PX4-Autopilot directory):

  ```sh
  export replay=<absolute_path_to_log_file.ulg>
  make px4_sitl_default
  ```

  This will create the build/make output in a separate build directory `build/px4_sitl_default_replay` (so that the parameters don't interfere with normal builds).
  It's possible to choose any posix SITL build target for replay, since the build system knows through the `replay` environment variable that it's in replay mode.

- Add ORB publisher rules in the file `build/px4_sitl_default_replay/rootfs/orb_publisher.rules`.
  This file defines the modules that are allowed to publish particular messages.
  형식은 다음과 같습니다.

  ```sh
  restrict_topics: <topic1>, <topic2>, ..., <topicN>
  module: <module>
  ignore_others: <true/false>
  ```

  This means that the given list of topics should only be published by `<module>` (which is the command name).
  다른 모듈에서 이러한 주제에 대한 발행은 자동으로 무시됩니다.
  If `ignore_others` is `true`, publications to other topics from `<module>` are ignored.

  For replay, we only want the `replay` module to be able to publish the previously identified list of topics.
  So, for replaying `ekf2`, the rules file should look like this:

  ```sh
  restrict_topics: sensor_combined, vehicle_gps_position, vehicle_land_detected
  module: replay
  ignore_others: true
  ```

  With this, the modules that usually publish these topics don't need to be disabled for the replay.

- _(Optional)_ Setup parameter overrides (see [instructions below](#overriding-parameters-in-the-original-log)).

- _(Optional)_ Copy a `dataman` mission file from the SD card to the build directory.
  This is only necessary if a mission should be replayed.

- 재생

  ```sh
  make px4_sitl_default jmavsim
  ```

  This will automatically open the log file, apply the parameters and start the replay.
  Once done, it will report the outcome and exit.
  The newly generated log file can then be analyzed. It can be found in `rootfs/fs/microsd/log`, in subdirectories organised by date.
  Replayed log file names will have the `_replayed` suffix.

  Note that the above command will show the simulator as well, but - depending on what is being replayed - it will not show what's actually going on.
  It is still possible to connect via QGC and, for example, view the changing attitude during replay.

- 마지막으로, 일반 빌드 대상이 다시 사용되도록 환경 변수를 설정 해제합니다.

  ```sh
  unset replay
  ```

### Overriding Parameters in the Original Log

By default, all parameters from the original log file are applied during a replay.
If a parameter changes during recording, it will be changed at the right time during the replay.

Parameters can be overridden during a replay in two ways: _fixed_ and _dynamic_.
When parameters are overridden, corresponding parameter changes in the log are not applied during replay.

- **Fixed parameter overrides** will override parameters from the start of the replay.
  They are defined in the file `build/px4_sitl_default_replay/rootfs/replay_params.txt`, where each line should have the format `<param_name> <value>`.
  예:

  ```sh
  EKF2_RNG_NOISE 0.1
  ```

- **Dynamic parameter overrides** will update parameter values at specified times.
  These parameters will still be initialised to the values in the log or in the fixed overrides.
  Parameter update events should be defined in `build/px4_sitl_default_replay/rootfs/replay_params_dynamic.txt`, where each line has the format `<param_name> <value> <timestamp>`.
  The timestamp is the time in seconds from the start of the log. 예:

  ```sh
  EKF2_RNG_NOISE 0.15 23.4
  EKF2_RNG_NOISE 0.05 56.7
  EKF2_RNG_DELAY 4.5 30.0
  ```

### 중요 참고 사항

- 재생 중에 로그 파일의 모든 드롭아웃이 보고됩니다.
  These have a negative effect on the replay, so care should be taken to avoid dropouts during recording.
- It is currently only possible to replay in 'real-time': as fast as the recording was done.
  이는 향후 추가할 예정입니다.
- 타임스탬프가 0인 메시지는 유효하지 않은 것으로 간주되어 재생되지 않습니다.

## EKF2 재생

이것은 빠른 EKF2 재생을 위한 시스템 전체 재생의 전문화입니다.

:::info
The recording and replay of flight logs with [multiple EKF2 instances](../advanced_config/tuning_the_ecl_ekf.md#running-multiple-ekf-instances) is not supported.
To enable recording for EKF replay you must set the parameters to enable a [single EKF2 instance](../advanced_config/tuning_the_ecl_ekf.md#running-a-single-ekf-instance).
:::

In EKF2 mode, the replay will automatically create the ORB publisher rules described above.

To perform an EKF2 replay:

- Record the original log.
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

### Adjusting EKF2-specific Parameters for the Replay

First install `pyulog`:

```sh
pip install --user pyulog
```

Extract the original log's parameters to `replay_params.txt`:

```sh
ulog_params -i "$replay" -d ' ' | grep -e '^EKF2' > build/px4_sitl_default_replay/rootfs/replay_params.txt
```

Adjust these as desired, and add dynamic parameter overrides in `replay_params_dynamic.txt` if necessary.

## 무대 뒤에서

재생은 3가지 구성 요소로 이루어집니다.

- A replay module
  These have a negative effect on replay, so care should be taken to avoid dropouts during recording.
- It is currently only possible to replay in 'real-time': as fast as the recording was done.

The replay module reads the log and publishes the messages at the same speed as they were recorded.
현재 시스템 시간과 일치하도록 각 메시지의 타임스탬프에 상수 오프셋이 추가됩니다(이것이 다른 모든 타임스탬프가 상대적이어야 하는 이유입니다).
The command `replay tryapplyparams` is executed before all other modules are loaded and applies the parameters from the log and user-set parameters.
Then as the last command, `replay trystart` will again apply the parameters and start the actual replay.
Both commands do nothing if the environment variable `replay` is not set.

ORB 게시자 규칙을 사용하면, 위에서 설명한 대로 시스템의 어느 부분을 재생할지 선택할 수 있습니다. POSIX SITL 대상에 대해서만 컴파일됩니다.

The **time handling** is still an **open point**, and needs to be implemented.
