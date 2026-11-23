# Test MC_11 - Companion Computer (ROS 2 External Mode)

## Objective

To test autonomous flight commanded from a companion computer using the [PX4 ROS 2 Interface Library](../ros2/px4_ros2_interface_lib.md) over the [uXRCE-DDS](../middleware/uxrce_dds.md) or [Zenoh](../middleware/zenoh.md) bridge.

The test runs the library's [go-to example](https://github.com/Auterion/px4-ros2-interface-lib/tree/main/examples/cpp/modes/goto), which registers an external flight mode named _Go-to Example_ and flies a fixed triangular pattern (about 20 m x 30 m) relative to the position where the mode is engaged, at constant altitude.
This exercises external mode registration, setpoint configuration, go-to setpoint streaming, and mode completion signalling across the bridge.

## Preflight

Companion computer setup (once):

- Connect a companion computer to the flight controller and start the bridge:
  - [uXRCE-DDS](../middleware/uxrce_dds.md): run the micro XRCE-DDS agent on the companion computer, or
  - [Zenoh](../middleware/zenoh.md): follow the ROS 2 Zenoh bring-up on the companion computer
- Set up a ROS 2 workspace on the companion computer with `px4_msgs` and the interface library, following steps 1 to 3 of [PX4 ROS 2 Control Interface > Installation and First Test](../ros2/px4_ros2_control_interface.md#installation-and-first-test) (skip the SITL steps).
  The go-to example is built as part of the workspace (package `example_mode_goto_cpp`).
- Use a recent QGroundControl (Daily), which supports dynamically updating the list of flight modes.

Before flight:

- Start the example on the companion computer:

  ```sh
  ros2 run example_mode_goto_cpp example_mode_goto
  ```

- Confirm the console shows the registration succeeding (`Registering 'Go-to Example'` followed by `Got RegisterExtComponentReply`)

- Confirm _Go-to Example_ shows up in the QGC flight mode list

- Confirm the airspace is clear roughly 20 m north and 30 m east of the planned engage position (the pattern is flown from wherever the mode is engaged)

## Flight Tests

❏ Mode registration

&nbsp;&nbsp;&nbsp;&nbsp;❏ With the vehicle disarmed, stop and restart the example node

&nbsp;&nbsp;&nbsp;&nbsp;❏ _Go-to Example_ disappears from and reappears in the QGC mode list

❏ Go-to Example mode

&nbsp;&nbsp;&nbsp;&nbsp;❏ Arm and take off in Position mode, climb to a safe altitude (10 m or more)

&nbsp;&nbsp;&nbsp;&nbsp;❏ Select _Go-to Example_ in QGC (mode must engage on first attempt, no rejection message)

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vehicle settles at the engage position, then flies north facing the direction of travel

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vehicle flies the east leg while spinning around yaw, slowing down as it approaches the corner

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vehicle returns to the engage position with the heading fixed on the first corner

&nbsp;&nbsp;&nbsp;&nbsp;❏ On completion the vehicle holds position; switch back to Position or Hold mode

❏ Re-engagement

&nbsp;&nbsp;&nbsp;&nbsp;❏ Fly to a different location and engage _Go-to Example_ again

&nbsp;&nbsp;&nbsp;&nbsp;❏ The same pattern is flown relative to the new engage position

❏ Companion loss failsafe

&nbsp;&nbsp;&nbsp;&nbsp;❏ Engage _Go-to Example_ once more and stop the example node (Ctrl-C) mid-pattern

&nbsp;&nbsp;&nbsp;&nbsp;❏ Vehicle triggers a failsafe and switches to Hold; QGC shows a corresponding warning

❏ Land and disarm

## Очікувані результати

- The external mode appears in QGC while the node is running and can be engaged on the first attempt
- Transitions into and out of the mode are smooth, with no twitches or setpoint jumps
- The triangular pattern is tracked accurately at constant altitude
- Stopping the node while the mode is active triggers a failsafe into Hold
- The log shows the pattern portion of the flight in the external mode (check in [Flight Review](../log/flight_review.md))
- Attach the flight log and the example's console output to the test report
