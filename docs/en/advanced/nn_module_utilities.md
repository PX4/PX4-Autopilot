# Neural Network Module Utilities

This page will explain the parts of the module that do not directly concern something with the neural network, but PX4 related implementations, so that you more easily can shape the module to your needs.

To learn more about how PX4 works in general, it is recommended to start with [Getting started](../dev_setup/getting_started.md).

## Autostart

In the PX4-Autopilot source code there are startup scripts, in these I have added a line for the mc_nn_control module. in ROMFS/px4fmu_common/init.d/rc.mc_apps it checks wether the module is included by looking for the parameter MC_NN_EN. If this is set to 1, which is the default value, the module will be started when booting PX4. Similarly you could create other parameters in the src/modules/mc_nn_control/mc_nn_control_params.c file for other startup script checks.

:::info
Note that it's only the first time you build that the default param value will overwrite the current one. So if you change it in the param file it will not be changed when flashing to the controller again. To do this you can change it in QGC or in the [terminal](../modules/modules_command.md#param).
:::

## Custom Flight Mode
The module creates its own flight mode "Neural Control" which lets you choose it from the flight mode menu in QGC and bind it to a switch on you RC controller. This is done by using the [ROS 2 Interface Library](../ros2/px4_ros2_interface_lib.md) internally. This involves several steps:

:::info
The module does not actually use ROS 2, it just uses the internal API that is exposed through uORB topics.
:::
:::info
In some QGC versions this does not work, as of 17. March 2025. You can use v4.4.0 release candidate 1, which can be found among the [QGC releases](https://github.com/mavlink/qgroundcontrol/releases/). Have only got this working SITL, in real flight you have to use a RC controller to switch to the correct external flight mode.
:::

1. Publish a [RegisterExtComponentRequest](../msg_docs/RegisterExtComponentRequest.md). This specifies what you want to create, you can read more about this in the [Control Interface](../ros2/px4_ros2_control_interface.md). In this case we register an arming check and a mode.
1. Wait for a [RegisterExtComponentReply](../msg_docs/RegisterExtComponentReply.md). This will give feedback on wether the mode registration was successful, and what the mode and arming check id is for the new mode.
1. [Optional] With the mode id, publish a [VehicleControlMode](../msg_docs/VehicleControlMode.md) message on the config_control_setpoints topic. Here you can configure what other modules run in parallel. The example controller replaces everything, so it turns off allocation. If you want to replace other parts you can enable or disable the modules accordingly.
1. [Optional] With the mode id, publish a [ConfigOverrides](../msg_docs/ConfigOverrides.md) on the config_overrides_request topic. (This is not done in the example module) This will let you defer failsafes or stop it from automatically disarming.
1. When the mode has been registered a [ArmingCheckRequest](../msg_docs/ArmingCheckRequest.md) will be sent, asking if your mode has everything it needs to run. This message must be answered with a [ArmingCheckReply](../msg_docs/ArmingCheckReply.md) so the mode is not flagged as unresponsive. In this response it is possible to set what requirements the mode needs to run, like local position. If any of these requirements are set the commander will stop you from switching to the mode if they are not fulfilled. It is also important to set health_component_index and num_events to 0 to not get a segmentation fault. Unless you have a health component or events.
1. Listen to the [VehicleStatus](../msg_docs/VehicleStatus.md) topic. If the nav_state equals the assigned mode_id, then the Neural Controller is activated.
1. When active the module will run the controller and publish to [ActuatorMotors](../msg_docs/ActuatorMotors.md). If you want to replace a different part of the controller, you should find the appropriate topic to publish to.

To see how the requests are handled you can check out src/modules/commander/ModeManagement.cpp.

## Logging
To add module specific logging a new topic has been added to [uORB](../middleware/uorb.md) called [NeuralControl](../msg_docs/NeuralControl.md). The message definition is also added in msg/CMakeLists.txt, and to src/modules/logger/logged_topics.cpp under the debug category. So for these messages to be saved in you ulog logs you need to include debug in the SDLOG_PROFILE parameter.

## Timing

The module has two includes for measuring the inference times. The first one is a driver that works on the actual flight controller units, but a second one, chrono, is loaded for SITL testing. Which timing library is included and used is based on wether PX4 is built with NUTTX or not.
