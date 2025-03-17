# System-wide Replay

It is possible to record and replay arbitrary parts of the system based on ORB messages.

Replay is useful to test the effect of different parameter values based on real data, compare different estimators, etc.

## Prerequisites

The first step is to identify the module or modules that should be replayed.
Then, identify all the inputs to these modules, i.e. subscribed ORB topics.
For system-wide replay, this consists of all hardware input: sensors, RC input, MAVLink commands and file system.

All identified topics need to be logged at full rate (see [logging](../dev_log/logging.md)).
For `ekf2` this is already the case with the default set of logged topics.

It is important that all replayed topics contain only a single absolute timestamp, which is the automatically generated field `timestamp`.
Should there be more timestamps, they must be relative to the main timestamp.
For an example, see [SensorCombined.msg](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorCombined.msg).
Reasons for this are given below.

## Usage

- First, choose the file to replay and build the target (from within the PX4-Autopilot directory):

  ```sh
  export replay=<absolute_path_to_log_file.ulg>
  make px4_sitl_default
  ```

  This will create the build/make output in a separate build directory `build/px4_sitl_default_replay` (so that the parameters don't interfere with normal builds).
  It's possible to choose any posix SITL build target for replay, since the build system knows through the `replay` environment variable that it's in replay mode.

- Add ORB publisher rules in the file `build/px4_sitl_default_replay/rootfs/orb_publisher.rules`.
  This file defines the modules that are allowed to publish particular messages.
  It has the following format:

  ```sh
  restrict_topics: <topic1>, <topic2>, ..., <topicN>
  module: <module>
  ignore_others: <true/false>
  ```

  This means that the given list of topics should only be published by `<module>` (which is the command name).
  Publications to any of these topics from another module are silently ignored.
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
- Start the replay:

  ```sh
  make px4_sitl_default jmavsim
  ```

  This will automatically open the log file, apply the parameters and start the replay.
  Once done, it will report the outcome and exit.
  The newly generated log file can then be analyzed. It can be found in `rootfs/fs/microsd/log`, in subdirectories organised by date.
  Replayed log file names will have the `_replayed` suffix.

  Note that the above command will show the simulator as well, but - depending on what is being replayed - it will not show what's actually going on.
  It is still possible to connect via QGC and, for example, view the changing attitude during replay.

- Finally, unset the environment variable, so that the normal build targets are used again:

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
  For example:

  ```sh
  EKF2_RNG_NOISE 0.1
  ```

- **Dynamic parameter overrides** will update parameter values at specified times.
  These parameters will still be initialised to the values in the log or in the fixed overrides.
  Parameter update events should be defined in `build/px4_sitl_default_replay/rootfs/replay_params_dynamic.txt`, where each line has the format `<param_name> <value> <timestamp>`.
  The timestamp is the time in seconds from the start of the log. For example:

  ```sh
  EKF2_RNG_NOISE 0.15 23.4
  EKF2_RNG_NOISE 0.05 56.7
  EKF2_RNG_DELAY 4.5 30.0
  ```

### Important Notes

- During replay, all dropouts in the log file are reported.
  These have a negative effect on the replay, so care should be taken to avoid dropouts during recording.
- It is currently only possible to replay in 'real-time': as fast as the recording was done.
  This is planned to be extended in the future.
- A message that has a timestamp of 0 will be considered invalid and not be replayed.

## EKF2 Replay

This is a specialization of the system-wide replay for fast EKF2 replay.

::: info
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

## Behind the Scenes

Replay is split into 3 components:

- A replay module
  These have a negative effect on replay, so care should be taken to avoid dropouts during recording.
- It is currently only possible to replay in 'real-time': as fast as the recording was done.

The replay module reads the log and publishes the messages at the same speed as they were recorded.
A constant offset is added to the timestamp of each message to match the current system time (this is the reason why all other timestamps need to be relative).
The command `replay tryapplyparams` is executed before all other modules are loaded and applies the parameters from the log and user-set parameters.
Then as the last command, `replay trystart` will again apply the parameters and start the actual replay.
Both commands do nothing if the environment variable `replay` is not set.

The ORB publisher rules allow to select which part of the system is replayed, as described above. They are only compiled for the posix SITL targets.

The **time handling** is still an **open point**, and needs to be implemented.
