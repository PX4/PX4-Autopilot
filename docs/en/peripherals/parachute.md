# Parachute

PX4 can be configured to trigger a parachute during [flight termination](#flight-termination) or in response to the MAVLink [`MAV_CMD_DO_PARACHUTE` command](#mav-cmd-do-parachute).

The parachute can be connected to a free PWM output or via MAVLink.

## Using Parachutes

Below are a few considerations when using parachutes:

- A parachute does not guarantee that the vehicle will not be destroyed or cause harm!
  You must always fly with safety in mind.
- Parachutes require careful usage to be effective.
  For example, they must be folded correctly.
- Parachutes have a minimum effective altitude.
- A parachute may trigger while the vehicle is upside down.
  This will increase the time required to slow, and may result in the drone collapsing the parachute.
- The parachute will only deploy if the flight controller is powered and PX4 is running properly (unless it is triggered independently of PX4).
  It will not deploy if something causes the flight stack to crash.

## Flight Termination {#flight-termination}

[Flight termination](../advanced_config/flight_termination.md) (and hence parachute deployment) may be triggered by safety checks such as RC Loss, geofence violation, and so on, from attitude triggers and other failure detector checks, or by a command from a ground station.

During flight termination PX4 turns off all controllers and sets PWM outputs to their "failsafe" values.
These values are set to a level that stops motors, and may also be used to trigger the parachute.
If a MAVLink parachute is connected and healthy, a command will be sent to activate it.

The setup to deploy a parachute during flight termination involves configuring:

- _Flight termination_ as the appropriate action for those safety and failure cases where the parachute should be deployed.
- PX4 to deploy the parachute during flight termination (set PWM output levels appropriately or send the MAVLink parachute deploy command).
- Configure PX4 output levels to disable motors on failsafe.
  This is the default so usually nothing is required (for servos it's the center value).

### Enable Flight Termination

To enable flight termination:

- Set [Safety](../config/safety.md) action to _Flight termination_ for checks where you want the parachute to trigger.
- Set [Failure Detector](../config/safety.md#failure-detector) pitch angles, roll angles and time triggers for crash/flip detection, and disable the failure/IMU timeout circuit breaker (i.e. set [CBRK_FLIGHTTERM=0](../advanced_config/parameter_reference.md#CBRK_FLIGHTTERM)).
- <Badge type="tip" text="PX4 v1.18" /> Set [FD_ALT_LOSS](../advanced_config/parameter_reference.md#FD_ALT_LOSS) to enable flight termination if a rotary-wing vehicle loses too much altitude below its setpoint (see [Altitude Loss Trigger](../config/safety.md#altitude-loss-trigger)).

::: info
You can also configure an [external Automatic Trigger System (ATS)](../config/safety.md#external-automatic-trigger-system-ats) for failure detection.
:::

### Parachute Output Bus Setup

If the parachute is triggered by a PWM or CAN output then it must first be connected to an unused output.
You will probably also need to separately power the parachute servo.
This is might be done by connecting a 5V BEC to the Flight Controller servo rail, and powering the parachute from it.

You then need to ensure that the parachute pin will be set to a value that will trigger the parachute when a failsafe occurs:

- Open [Actuators](../config/actuators.md) in QGroundControl
- Assign the _Parachute_ function to any unused output (below we set the `AUX6` output):

  ![Actuators - Parachute (QGC)](../../assets/config/actuators/qgc_actuators_parachute.png)

- Set appropriate PWM values for your parachute.
  The output is automatically set to the maximum PWM value when a failsafe is triggered.

  ::: info
  For the spring-loaded launcher from [Fruity Chutes](https://fruitychutes.com/uav_rpv_drone_recovery_parachutes/drone_multicopter_quadcopter_recovery_parachutes#Harrier) the minimum PWM value should be between 700 and 1000ms, and the maximum value between 1800 and 2200ms.
  :::

### MAVLink Parachute Setup for Flight Termination

<Badge type="tip" text="PX4 v1.13" />

::: tip
This setup is only required to enable parachute integration for flight termination.
It does affect parachute deployment commanded by an external system using the [MAV_CMD_DO_PARACHUTE command](#mav-cmd-do-parachute).
:::

PX4 will trigger a connected and healthy MAVLink parachute on failsafe by sending the command [MAV_CMD_DO_PARACHUTE](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_PARACHUTE) with the [PARACHUTE_RELEASE](https://mavlink.io/en/messages/common.html#PARACHUTE_ACTION) action.

MAVLink parachute support in flight termination is enabled by setting the parameter [COM_PARACHUTE](../advanced_config/parameter_reference.md#COM_PARACHUTE) to a non-zero value.
The parameter also configures the arming check and in-flight failsafe action when the parachute system is missing or unhealthy, see [Parachute Health Failsafe](../config/safety.md#parachute-health-failsafe).

PX4 will then indicate parachute status using the [MAV_SYS_STATUS_RECOVERY_SYSTEM](https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_RECOVERY_SYSTEM) bit in the [SYS_STATUS](https://mavlink.io/en/messages/common.html#SYS_STATUS) extended onboard control sensors fields:

- `SYS_STATUS.onboard_control_sensors_present_extended`: MAVLink parachute present (based on heartbeat detection).
- `SYS_STATUS.onboard_control_sensors_enabled_extended`: MAVLink parachute is enabled ([`COM_PARACHUTE > 0`](../advanced_config/parameter_reference.md#COM_PARACHUTE)).
- `SYS_STATUS.onboard_control_sensors_health_extended`: MAVLink parachute healthy (based on heartbeat detection).

A MAVLink parachute is required to emit a [HEARTBEAT](https://mavlink.io/en/messages/common.html#HEARTBEAT) with `HEARTBEAT.type` of [MAV_TYPE_PARACHUTE](https://mavlink.io/en/messages/common.html#MAV_TYPE_PARACHUTE).

<!-- PX4 v1.13 support added here: https://github.com/PX4/PX4-Autopilot/pull/18589 -->

## MAVLink Trigger (MAV_CMD_DO_PARACHUTE) {#mav-cmd-do-parachute}

<Badge type="tip" text="main (PX4 v2.0)" />

Parachutes can also be directly triggered using the [MAV_CMD_DO_PARACHUTE](https://mavlink.io/en/messages/common.html#MAV_CMD_DO_PARACHUTE) command during normal operation.

This does not trigger flight termination, and does not require any of the setup described in [MAVLink Parachute Setup for Flight Termination](#mavlink-parachute-setup-for-flight-termination).

:::warning
Unlike with flight termination, releasing the parachute by command does not stop the motors: the vehicle remains armed, is controlled by the active flight, and disarms normally after landing.
The component that sends the command is responsible for first putting the vehicle into a state that is safe under a parachute.
:::

### MAVLink Parachute

For a MAVLink parachute, the command should be addressed to the parachute's system and component id.

### Flight-Controller Connected Parachute

For a parachute connected to a flight controller output, address the command to the autopilot MAVLink component.

Setup instructions:

- Include the `parachute` module in the firmware by setting `CONFIG_MODULES_PARACHUTE=y` in the [board configuration](../hardware/porting_guide_config.md).
- Assign the _Parachute_ function to the connected output as described in [Parachute Output Bus Setup](#parachute-output-bus-setup).

Notes:

- The release is denied while the vehicle is landed, so that the parachute cannot be deployed with people around the vehicle.
- Once released, the parachute output is latched to the release value until reboot.

## Parachute Testing

:::warning
For the first test, try on the bench, without the props and with an unloaded parachute device!
:::

::: info
There is no way to recover from a Termination state!
You will need to reboot/power cycle the vehicle for subsequent tests.
:::

The parachute will trigger during [flight termination](../advanced_config/flight_termination.md).

The easiest way to test a (real) parachute is to enable the [failure detector attitude trigger](../config/safety.md#attitude-trigger) and tip the vehicle over.

You can also simulate a parachute/flight termination: [Gazebo Classic > Simulated Parachute/Flight Termination](../sim_gazebo_classic/index.md#simulated-parachute-flight-termination).
