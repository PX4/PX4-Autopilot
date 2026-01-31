# 소리 설명 (Pixhawk 시리즈)

[Pixhawk-series flight controllers](../flight_controller/pixhawk_series.md) use audible tones/tunes from a [buzzer](../getting_started/px4_basic_concepts.md#buzzer) and colours/sequences from a [LED](../getting_started/led_meanings.md) to indicate vehicle state and events (e.g. arming success and failure, low battery warnings).

표준 사운드 세트는 다음과 같습니다.

:::info
**Developers:** Tunes are defined in [/lib/tunes/tune_definition.desc](https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/tunes/tune_definition.desc) and can be tested using the [tune-control](../modules/modules_system.md#tune-control) module.
You can search for tune use using the string `TUNE_ID_name`(e.g. \`TUNE_ID_PARACHUTE_RELEASE)
:::

## 부팅 / 시작

부팅중에 재생되는 톤들입니다.

<!-- https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d/rcS --> 

#### 시작 톤

<audio controls><source src="../../assets/tunes/1_startup_tone.mp3" type="audio/mpeg">Your browser does not support the audio element.</audio>

<!-- tune: 1, STARTUP -->

- microSD card successfully mounted (during boot).

#### Error Tune

<audio controls><source src="../../assets/tunes/2_error_tune.mp3" type="audio/mpeg">Your browser does not support the audio element.</audio>

<!-- tune 2, ERROR_TUNE -->

- Hard fault has caused a system reboot.
- System set to use PX4IO but no IO present.
- UAVCAN is enabled but driver can't start.
- SITL/HITL enabled but _pwm_out_sim_ driver can't start.
- FMU startup failed.

#### Make File System

<audio controls><source src="../../assets/tunes/16_make_fs.mp3" type="audio/mpeg">Your browser does not support the audio element.</audio>

<!-- 14, SD_INIT (previously tune 16) -->

- Formatting microSD card.
- Mounting failed (if formatting succeeds boot sequence will try to mount again).
- No microSD card.

#### Format Failed

<audio controls><source src="../../assets/tunes/17_format_failed.mp3" type="audio/mpeg">Your browser does not support the audio element.</audio>

<!-- 15, SD_ERROR (previously 17) -->

- Formatting microSD card failed (following previous attempt to mount card).

#### Program PX4IO

<audio controls><source src="../../assets/tunes/18_program_px4io.mp3" type="audio/mpeg">Your browser does not support the audio element.</audio>

<!-- 16, PROG_PX4IO (previously id 18) -->

- Starting to program PX4IO.

#### Program PX4IO Success

<audio controls><source src="../../assets/tunes/19_program_px4io_success.mp3" type="audio/mpeg">Your browser does not support the audio element.</audio>

<!-- 17, PROG_PX4IO_OK (previously tune 19) -->

- PX4IO programming succeeded.

#### Program PX4IO Fail

<audio controls><source src="../../assets/tunes/20_program_px4io_fail.mp3" type="audio/mpeg">Your browser does not support the audio element.</audio>

<!-- 18, PROG_PX4IO_ERR (previously tune 20) -->

- PX4IO programming failed.
- PX4IO couldn't start.
- AUX Mixer not found.

## Operational

These tones/tunes are emitted during normal operation.

<a id="error_tune_operational"></a>

#### Error Tune

<audio controls><source src="../../assets/tunes/2_error_tune.mp3" type="audio/mpeg">Your browser does not support the audio element.</audio>

<!-- 2, ERROR_TUNE -->

- RC Loss

#### Notify Positive Tone

<audio controls><source src="../../assets/tunes/3_notify_positive_tone.mp3" type="audio/mpeg">Your browser does not support the audio element.</audio>

<!-- 3, NOTIFY_POSITIVE -->

- Calibration succeeded.
- Successful mode change.
- Command accepted (e.g. from MAVLink command protocol).
- Safety switch off (vehicle can be armed).

#### Notify Neutral Tone

<audio controls><source src="../../assets/tunes/4_notify_neutral_tone.mp3" type="audio/mpeg">Your browser does not support the audio element.</audio>

<!-- 4, NOTIFY_NEUTRAL -->

- Mission is valid and has no warnings.
- Airspeed calibration: supply more air pressure, or calibration complete.
- Safety switch turned on/disarmed (safe to approach vehicle).

#### Notify Negative Tone

<audio controls><source src="../../assets/tunes/5_notify_negative_tone.mp3" type="audio/mpeg">Your browser does not support the audio element.</audio>

<!-- 5, NOTIFY_NEGATIVE -->

- Calibration failed.
- Calibration already completed.
- Mission is invalid.
- Command denied, failed, temporarily rejected (e.g. from MAVLink command protocol).
- Arming/disarming transition denied (e.g. pre-flight checks failed, safety not disabled, system not in manual mode).
- Reject mode transition.

#### Arming Warning

<audio controls><source src="../../assets/tunes/6_arming_warning.mp3" type="audio/mpeg">Your browser does not support the audio element.</audio>

<!-- 6, ARMING_WARNING -->

- Vehicle is now armed.

#### Arming Failure Tune

<audio controls><source src="../../assets/tunes/10_arming_failure_tune.mp3" type="audio/mpeg">Your browser does not support the audio element.</audio>

<!-- 10, ARMING_FAILURE -->

- Arming failed

#### Battery Warning Slow

<audio controls><source src="../../assets/tunes/7_battery_warning_slow.mp3" type="audio/mpeg">Your browser does not support the audio element.</audio>

<!-- 7,  BATTERY_WARNING_SLOW -->

- Low battery warning ([failsafe](../config/safety.md#battery-level-failsafe)).

#### Battery Warning Fast

<audio controls><source src="../../assets/tunes/8_battery_warning_fast.mp3" type="audio/mpeg">Your browser does not support the audio element.</audio>

<!-- 8, BATTERY_WARNING_FAST -->

- Critical low battery warning ([failsafe](../config/safety.md#battery-level-failsafe)).

#### GPS Warning Slow

<audio controls><source src="../../assets/tunes/9_gps_warning_slow.mp3" type="audio/mpeg">Your browser does not support the audio element.</audio>

<!-- 9,  GPS_WARNING -->

#### Parachute Release

<audio controls><source src="../../assets/tunes/11_parachute_release.mp3" type="audio/mpeg">Your browser does not support the audio element.</audio>

<!-- 11, PARACHUTE_RELEASE -->

- Parachute release triggered.

#### Single Beep

<audio controls><source src="../../assets/tunes/14_single_beep.mp3" type="audio/mpeg">Your browser does not support the audio element.</audio>

<!-- 12, SINGLE_BEEP (previously was id 14 -->

- Magnetometer/Compass calibration: Notify user to start rotating vehicle.

#### Home Set Tune

<audio controls><source src="../../assets/tunes/15_home_set_tune.mp3" type="audio/mpeg">Your browser does not support the audio element.</audio>

<!-- 13, HOME_SET (previously id 15) -->

- Home position initialised (first time only).

#### Power Off Tune

<audio controls><source src="../../assets/tunes/power_off_tune.mp3" type="audio/mpeg">Your browser does not support the audio element.</audio>

- Vehicle powering off.

<!--19, POWER_OFF -->
