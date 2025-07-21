<!-- This is the single-source for autotuning docs used in the autotune_mc.md and autotune_fy.md
At time of writing, only FW, MC, and VTOL, support autotuning.
VTOL has its own doc that references the other two
-->

<div v-if="$frontmatter.frame === 'Multicopter'">

# Auto-tuning (Multicopter)

</div>
<div v-else-if="$frontmatter.frame === 'Plane'">

# Auto-tuning (Fixed-Wing)

</div>

자동 튜닝은 안정적이고 반응성은 높은 비행에 탁월한 컨트롤러인 PX4의 속도와 자세를 자동으로 튜닝합니다(다른 튜닝은 "선택 사항"에 가깝습니다).

Tuning only needs to be done once, and is recommended unless you're using a vehicle that has already been tuned by the manufacturer (and not modified since).

:::warning
Auto-tuning is performed while flying.
The airframe must fly well enough to handle moderate disturbances, and should be closely attended:

- Test that your vehicle is [stable enough for autotuning](#pre-tuning-test).
- Be ready to abort the autotuning process.
  You can do this by changing flight modes<div style="display: inline;" v-if="$frontmatter.frame === 'Plane'"> or using an auto-tune enable/disable switch ([if configured](#enable-disable-autotune-switch))</div>.
- Verify that the vehicle flies well after tuning.

:::

<lite-youtube videoid="5xswOhhqrIQ" title="QGroundControl Autotune Feature Breakdown for PX4 Autopilot"/>

## 사전 튜닝 테스트

The vehicle must be able to fly and adequately stabilize itself before running auto-tune.
This test ensures that the vehicle can fly safely in position controlled modes.

To make sure the vehicle is stable enough for auto-tuning:

1. 비행 구역이 깨끗하고 공간이 충분한 지 확인하기 위하여, 일반적인 비행 전 안전 점검을 실시합니다.

2. Take off and <div style="display: inline;" v-if="$frontmatter.frame === 'Multicopter'">hover at 1m above ground in [Altitude mode](../flight_modes_mc/altitude.md) or [Stabilized mode](../flight_modes_mc/manual_stabilized.md)</div><div style="display: inline;" v-else-if="$frontmatter.frame === 'Plane'">fly at cruise speed in [Position mode](../flight_modes_fw/position.md) or [Altitude mode](../flight_modes_fw/altitude.md)</div>.

3. Use the RC transmitter roll stick to perform the following maneuver, tilting the vehicle just a few degrees: _roll left > roll right > center_ (The whole maneuver should take about 3 seconds).
  기체는 2번의 진동 이내에서 안정화되어야 합니다.

4. 각각의 시도에서 더 큰 진폭으로 기울이면서 기동을 반복합니다.
  기체가 ~20도에서 2번의 진동 내에서 안정화될 수 있으면 다음 단계로 이동합니다.

5. 피치 축에서 동일한 동작을 반복합니다.
  As above, start with small angles and confirm that the vehicle can stabilise itself within 2 oscillations before increasing the tilt.

If the drone can stabilize itself within 2 oscillations it is ready for the [auto-tuning procedure](#auto-tuning-procedure).

:::warning
If the drone cannot stabilize itself sufficiently, follow the instructions in the [troubleshooting](#troubleshooting) section.
These explain the minimal manual tuning to prepare the vehicle for auto-tuning.
:::

## Auto-tuning Procedure

The auto-tuning sequence must be performed in a **safe flight zone, with enough space**.
It takes about 40 seconds ([between 19 and 68 seconds](#how-long-does-autotuning-take)).
For best results, we recommend running the test in calm weather conditions.

The recommended mode for autotuning is <div style="display: inline;" v-if="$frontmatter.frame === 'Multicopter'">[Altitude mode](../flight_modes_mc/altitude.md)</div><div  style="display: inline;" v-else-if="$frontmatter.frame === 'Plane'">[Hold mode](../flight_modes_fw/hold.md)</div>, but any other flight mode can be used.
The RC sticks cannot be used during autotuning (moving the sticks will stop the autotune operation).

The test steps are:

1. Perform the [pre-tuning test](#pre-tuning-test).

2. Takeoff using RC control <div style="display: inline;" v-if="$frontmatter.frame === 'Multicopter'">in [Altitude mode](../flight_modes_mc/altitude.md).
  Hover the vehicle at a safe distance and at a few meters above ground (between 4 and 20m).</div><div v-else-if="$frontmatter.frame === 'Plane'">
  Once flying at cruise speed, activate [Hold mode](../flight_modes_fw/hold.md).
  This will guide the plane to fly in circle at constant altitude and speed.</div>

3. Enable autotune.

  <div v-if="$frontmatter.frame === 'Plane'">
  <div class="tip custom-block"><p class="custom-block-title">TIP</p>

  If an [Enable/Disable Autotune Switch](#enable-disable-autotune-switch) is configured you can just toggle the switch to the "enabled" position.

  </div></div>

  1. In QGroundControl, open the menu **Vehicle setup > PID Tuning**:

    ![Tuning Setup > Autotune Enabled](../../assets/qgc/setup/autotune/autotune.png)

  2. Select either the _Rate Controller_ or _Attitude Controller_ tabs.

  3. Ensure that the **Autotune enabled** button is enabled (this will display the **Autotune** button and remove the manual tuning selectors).

  4. Read the warning popup and click on **OK** to start tuning.

4. The drone will first start to perform quick roll motions followed by pitch and yaw motions.
  The progress is shown in the progress bar, next to the _Autotune_ button.

<div style="display: inline;" v-if="$frontmatter.frame === 'Multicopter'">

5. Manually land and disarm to apply the new tuning parameters.
  Takeoff carefully and manually test that the vehicle is stable.

</div><div v-else-if="$frontmatter.frame === 'Plane'">

5. The tuning will be immediately/automatically be applied and tested in flight (by default).
  PX4 will then run a 4 second test and revert the new tuning if a problem is detected.

</div>

:::warning
If any strong oscillations occur, land immediately and follow the instructions in the [Troubleshooting](#troubleshooting) section below.
:::

Additional notes:

<div v-if="$frontmatter.frame === 'Multicopter'">

- The instructions above tune the vehicle in [Altitude mode](../flight_modes_mc/altitude.md).
  You can instead takeoff in [Takeoff mode](../flight_modes_mc/takeoff.md) and tune in [Position mode](../flight_modes_mc/position.md) if the vehicle is is _known_ to be stable in these modes.

</div>
<div v-else-if="$frontmatter.frame === 'Plane'">

- Autotuning can also be run in [Altitude mode](../flight_modes_fw/altitude.md) or [Position mode](../flight_modes_fw/position.md).
  However running the test while flying straight requires a larger safe area for tuning, and does not give a significantly better tuning result.

</div>

- Whether tuning is applied while flying or after landing can be [configured using parameters](#apply-tuning-when-in-air-landed).

<div v-if="$frontmatter.frame === 'Multicopter'">

## Autotuning Large Vehicles

For big multicopter vehicles you may need to increase the desired raise time of the step response [MC_AT_RISE_TIME](../advanced_config/parameter_reference.md#MC_AT_RISE_TIME).
This requires some trial and error as an appropriate rise time depends on both vehicle size and the rotor response.

Note that if there are slow oscillations in pretuning it may mean that the attitude loop is too fast compared to the rate loop.
In this case you should troubleshoot as described in [Drone oscillates when performing the pre-tuning test](#drone-oscillates-when-performing-the-pre-tuning-test).

<!-- Fixed wing rise time is hardcoded (it's lower than on multirotors and is probably enough for most platforms). -->

</div>

## 문제 해결

### Drone oscillates when performing the pre-tuning test

<div v-if="$frontmatter.frame === 'Multicopter'">

Slow oscillations (1 oscillation per second or slower): this often occurs on large platforms and means that the attitude loop is too fast compared to the rate loop:

- Decrease [MC_ROLL_P](../advanced_config/parameter_reference.md#MC_ROLL_P) and [MC_PITCH_P](../advanced_config/parameter_reference.md#MC_PITCH_P) by steps of 1.0.

Fast oscillations (more than 1 oscillation per second): this is because the gain of the rate loop is too high.

- Decrease `MC_[ROLL|PITCH|YAW]RATE_K` by steps of 0.02

</div>
<div v-else-if="$frontmatter.frame === 'Plane'">

Slow oscillations (1 oscillation per second or slower): this often occurs on large platforms and means that the attitude loop is too fast compared to the rate loop.

- Increase [FW_R_TC](../advanced_config/parameter_reference.md#FW_R_TC) and [FW_P_TC](../advanced_config/parameter_reference.md#FW_P_TC) by steps of 0.1.

Fast oscillations (more than 1 oscillation per second): this is because the gain of the rate loop is too high.

- Decrease [FW_RR_P](../advanced_config/parameter_reference.md#FW_RR_P), [FW_PR_P](../advanced_config/parameter_reference.md#FW_PR_P), [FW_YR_P](../advanced_config/parameter_reference.md#FW_YR_P) by steps of 0.01.

</div>

### 자동 튜닝 실패

If the drone was not moving enough during auto-tuning, the system identification algorithm might have issues to find the correct coefficients.

Increase the <div style="display: inline;" v-if="$frontmatter.frame === 'Multicopter'">[MC_AT_SYSID_AMP](../advanced_config/parameter_reference.md#MC_AT_SYSID_AMP)</div><div style="display: inline;" v-else-if="$frontmatter.frame === 'Plane'">[FW_AT_SYSID_AMP](../advanced_config/parameter_reference.md#FW_AT_SYSID_AMP)</div> parameter by steps of 1 and trigger the auto-tune again.

### The drone oscillates after auto-tuning

Due to effects not included in the mathematical model such as delays, saturation, slew-rate, airframe flexibility, the loop gain can be too high.
To fix this, follow the same steps described [when the drone oscillates in the pre-tuning test](#drone-oscillates-when-performing-the-pre-tuning-test).

### 여전히 정상 작동하지 않는 경우

Attempt manual tuning using the guides listed in [See also](#see-also) below.

## Optional Configuration

### Apply Tuning when In-Air/Landed

<div v-if="$frontmatter.frame === 'Multicopter'">

By multicopters land before parameters are applied.
This behaviour can be configured using the [MC_AT_APPLY](../advanced_config/parameter_reference.md#MC_AT_APPLY) parameter:

</div>
<div v-else-if="$frontmatter.frame === 'Plane'">

By default fixed wing tuning the parameters are applied while flying, and then PX4 runs a test to confirm that the controllers work properly.
This behaviour can be configured using the [FW_AT_APPLY](../advanced_config/parameter_reference.md#FW_AT_APPLY) parameter:

</div>

- `0`: the gains are not applied.
  자동 튜닝의 결과를 직접적으로 사용하지 않은 체로 검사하는 경우에 사용합니다.
- `1`: apply the gains after disarm (default for multirotors).
  The operator can then test the new tuning while taking-off carefully.
- `2`: apply immediately (default for fixed-wings).
  The new tuning is applied, disturbances are sent to the controller and the stability is monitored during the next 4 seconds.
  If the control loop is unstable, the control gains are immediately reverted back to their previous value.
  If the test passes, the pilot can then use the new tuning.

<div v-if="$frontmatter.frame === 'Plane'">

### Enable/Disable Autotune Switch

A remote control switch can be configured to enable/disable autotune (in any mode) using an RC AUX channel (note, this is only supported on fixed-wing vehicles).

To map a switch:

1. Select an RC channel on your controller to use for the autotune enable/disable switch.
2. Set [RC_MAP_AUX1](../advanced_config/parameter_reference.md#RC_MAP_AUX1) to match the RC channel for your switch (you can use any of `RC_MAP_AUX1` to `RC_MAP_AUX6`).
3. Set [FW_AT_MAN_AUX](../advanced_config/parameter_reference.md#FW_AT_MAN_AUX) to the selected channel (i.e. `1: Aux 1` if you mapped `RC_MAP_AUX1`).

The auto tuner will be disabled when the switch is below `0.5` (on the manual control setpoint range of of `[-1, 1]`) and enabled when the switch channel is above `0.5`.

If using an RC AUX switch to enable autotuning, make sure to [select the tuning axes](#select-tuning-axis) before flight.

### Select Tuning Axis

Fixed-wing vehicles (only) can select which axes are tuned using the [FW_AT_AXES](../advanced_config/parameter_reference.md#FW_AT_AXES) bitmask parameter:

- bit `0`: roll (default)
- bit `1`: pitch (default)
- bit `2`: yaw

</div>

## 개발자 SDK

Autotuning is started using [MAV_CMD_DO_AUTOTUNE_ENABLE](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_AUTOTUNE_ENABLE) MAVLink command.

At time of writing the message is resent at regular intervals to poll PX4 for progress: the `COMMAND_ACK` includes result that the operation is in progress, and also the progress as a percentage.
The operation completes when the progress is 100% or the vehicle lands and disarms.

:::info
This is not a MAVLink-compliant implementation of a [command protocol long running command](https://mavlink.io/en/services/command.html#long_running_commands).
PX4 should stream progress as the protocol does not allow polling.
:::

The feature is not yet supported by MAVSDK.

## 배경 및 세부 사항

PX4 uses [PID controllers](../flight_stack/controller_diagrams.md) (rate, attitude, velocity, and position) to calculate the outputs required to move a vehicle from its current estimated state to match a desired setpoint.
The controllers must be well tuned in order to get the best performance out of a vehicle.
In particular, a poorly tuned rate controller results in less stable flight in all modes, and takes longer to recover from disturbances.

Generally if you use a [frame configuration](../config/airframe.md) that is similar to your vehicle then the vehicle will be able to fly.
However unless the configuration precisely matches your hardware you should tune the rate and attitude controllers.
Tuning the velocity and position controllers is less important because they are less affected by vehicle dynamics, and the default tuning configuration for a similar airframe is often sufficient.

Autotuning provides an automatic mechanism to tune the rate and attitude controllers.
It can be used to tune fixed-wing and multicopter vehicles, and VTOL vehicles when flying as a multicopter or as a fixed-wing (transition between modes must be manually tuned).
In theory it should work for other vehicle types that have a rate controller, but currently only the above types are supported.

Automatic tuning works well for the multicopter and fixed-wing vehicle configurations supported by PX4, provided the frame is not too flexible
(see [below for more information](#does-autotuning-work-for-all-supported-airframes)).

The vehicle must be flying in an altitude-stabilized mode (such as [Altitude mode](../flight_modes_mc/altitude.md), [Hold mode](../flight_modes_mc/hold.md), or [Position mode](../flight_modes_mc/position.md)).
The flight stack will apply a small disturbance to the vehicle in each axis and then attempt to calculate the new tuning parameters.
For fixed-wing vehicles the new tuning is applied in-air by default, after which the vehicle tests the new settings and reverts the tuning if the controllers are not stable.
For multicopter, the vehicle lands and applies the new tuning parameters after disarming; the pilot is expected to then take off carefully and test the tuning.

The tuning process takes about 40 seconds ([between 19 and 70 seconds](#how-long-does-autotuning-take)).
The default behaviour can be configured using [parameters](#optional-configuration).

### 자주 묻는 질문

#### 어떤 기체 유형이 지원됩니까?

Autotuning is enabled for multicopter, fixed-wing, and hybrid VTOL fixed-wing vehicles.

While it is not yet enabled for other frame types, in theory it an be used with any frame that uses a rate controller.

#### 지원되는 모든 기체에 대해 자동 튜닝이 작동됩니까?

The mathematical model used by autotuning to estimate the dynamics of the drone assumes this it is a linear system with no coupling between the axes (SISO), and with a limited complexity (2 poles and 2 zeros).
If the real drone is too far from those conditions, the model will not be able to represent the real dynamics of the drone.

In practise, autotuning generally works well for fixed-wing and multicopter, provided the frame is not too flexible.

#### 자동 튜닝은 얼마나 걸립니까?

Tuning takes 5s-20s per axis (aborted if tuning could not be established in 20s) + 2s pause between each axis + 4s of testing if the new gains are applied in air.

A multicopter must tune all three axes, and by default does not test the new gains in-air.
Tuning will therefore take between 19s (`5 + 2 + 5 + 2 + 5`) and 64s (`20x3 + 2x2`).

By default a fixed-wing vehicle tunes all three axes and then tests the new gains in-air.
The range is therefore between 25s (`5 + 2 + 5 + 2 + 5 + 2 + 4`) and 70s (`20x3 + 3x2 + 4`).

Note however that the above settings are defaults.
A multicopter can choose to run the tests in air, and a fixed-wing can choose not to.
Further, a fixed-wing can choose to tune fewer axes.

Anecdotally, it usually takes around 40s for either vehicle.

<!--
#### How vigorous is the disturbance applied by tuning

This might be added later. I'd like to just point to a video.

If not, perhaps say "not very" but you should expect that the vehicle might deflect by as much as 20degrees and so should be able to cope with that deflection with default tuning.

-->

<div v-if="$frontmatter.frame === 'Multicopter'">

## See also

- [Multicopter PID Tuning Guide](../config_mc/pid_tuning_guide_multicopter_basic.md) (Manual/Simple)
- [Multicopter PID Tuning Guide](../config_mc/pid_tuning_guide_multicopter.md) (Advanced/Detailed)
- [Parameters > Autotuning](../advanced_config/parameter_reference.md#autotune) (prefixed with `MC_AT_`).

</div>
<div v-else-if="$frontmatter.frame === 'Plane'">

## See also

- [Fixed-Wing PID Tuning Guide](../config_fw/pid_tuning_guide_fixedwing.md)
- [Parameters > Autotuning](../advanced_config/parameter_reference.md#autotune) (prefixed with `FW_AT_`).

</div>
