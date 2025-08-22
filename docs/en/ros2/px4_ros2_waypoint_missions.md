# PX4 ROS 2 Waypoint Missions

<Badge type="tip" text="PX4 v1.16" /> <Badge type="warning" text="Experimental" />

The [PX4 ROS 2 Interface Library](../ros2/px4_ros2_interface_lib.md) provides a high-level interface for executing waypoint missions in ROS 2.

Mission definitions can be loaded from JSON files.
The implementation can be extended with actions to customize behavior within a mission.

::: tip
This completely bypasses the existing PX4 mission mode and waypoint logic, while still allowing to use existing modes like takeoff or RTL.
:::

## Comparison to Existing PX4 Missions
There are some benefits and drawbacks to using ROS-based missions, which are provided in the following paragraphs.

The main use-case is for predefined waypoint missions where a custom behavior is required, such as a pickup action within a mission.

### Benefits
- Allows to extend mission capabilities by registering custom actions.
- More control over how the mission is executed. A custom trajectory executor can be implemented, which can use any of the existing PX4 setpoint types to track the trajectory.
- Reduced complexity on the flight controller side by running non-safety-critical and non-real-time code on a more high-level companion computer.
- It can be extended to support other trajectory types, like bezier or dubin curves.

### Drawbacks
- By not using the existing PX4 mission logic, QGroundControl currently does not display the mission or progress during execution.
  It can also not upload or download a mission.
  Therefore you will need another mechanism to provide a mission, such as from a web server, a custom GCS, or by generating it directly inside the application.
- The current implementation supports only multicopters, but is designed to be extendable to any other vehicle type.

## Overview

This diagram provides a conceptual overview of the main classes and their interactions:

![Waypoint missions overview diagram](../../assets/middleware/ros2/px4_ros2_interface_lib/waypoint_missions.svg)

<!-- Source: https://drive.google.com/file/d/1BXx4fegVE71eq0kDMMuRnhcLnJeDawWe/view?usp=sharing -->

Missions can be defined in JSON, either as a file, or directly inside the application.
There is a file change monitor (`MissionFileMonitor`), that can be used to automatically load a mission from a specific file whenever it is created by another application (e.g. upload via MAVFTP or a cloud service).

The **`MissionExecutor`** class contains the state machine to progress the mission index, and is at the core of the implementation:
- Internally, it builds on top of the [Modes and Mode Executors](px4_ros2_control_interface.md#overview) and registers itself through a custom mode and executor with PX4.
- It handles switching in and out of missions: it gets activated when the user switches to the corresponding mode and the vehicle is armed.
- Custom actions can be registered.
- The mission can be set. It then checks that all the actions which the mission defines are available and can be run.
- The state can be stored persistently by providing a file name, allowing for battery swapping.

The **`ActionInterface`** is an interface class for custom actions.
They are identified by a name, and any number of these can be registered with the `MissionExecutor`.
A custom action is then run whenever a mission item with matching name is executed, and any extra arguments from the JSON definition are passed as arguments (for example an altitude for a takeoff action).
Actions can call other actions, run any mode (PX4 or external by its ID), run a trajectory, or run any other external action before deciding when to continue the mission.

There is a set of default actions, for example for RTL, Land, etc. These simply trigger the corresponding PX4 mode.
They can be disabled or replaced with custom implementations.
There are also some special actions (which can be replaced as well):
- `OnFailure`: this is called in case of a failure, e.g. a mode switch failed, a non-existing action is called (by another action) or by an explicit call to `MissionExecutor::abort()`. The default is to run RTL, with fallback to Land.
- `OnResume`: this is called when resuming a mission (either from the ground or in-air). It handles a number of cases:
  - when called with an argument `action="storeState"`: this can be used to store the current position when the mission is deactivated, so it can be resumed from the same position. Currently it does not store anything.
  - otherwise, when called without a valid mission index or 0, it directly continues.
  - otherwise, when called while in-air, it also directly continues.
  - otherwise, if landed and if the current mission item is an action that supports resuming from landed, it continues to let the action handle the resuming.
  - otherwise, if landed, it finds the takeoff action from the mission, runs it, and then flies to the previous waypoint from the current index to continue the mission.
- `ChangeSettings`: this can be used to change the mission settings, such as the velocity.
  The settings are applied to all following items in the mission.

The **`TrajectoryExecutorInterface`** is an interface class to execute segments of a trajectory.
It can use any setpoint that PX4 and the current vehicle supports for tracking the trajectory.
This class is vehicle-type specific.
The current default implementation (`multicopter::WaypointTrajectoryExecutor`) uses a Goto setpoint (and thus is limited to multicopters).
The default can be replaced with a custom implementation.

## Usage

The following provides a small example. It defines a custom action and a mission that uses it.

```c++
class CustomAction : public px4_ros2::ActionInterface {
public:
  CustomAction(px4_ros2::ModeBase & mode) : _node(mode.node()) { }

  std::string name() const override {return "customAction";}

  void run(
    const std::shared_ptr<px4_ros2::ActionHandler> & handler,
    const px4_ros2::ActionArguments & arguments,
    const std::function<void()> & on_completed) override
  {
    RCLCPP_INFO(_node.get_logger(), "Running custom action");
    // Do something...

    on_completed();
  }
private:
  rclcpp::Node & _node;
};

class MyMission {
public:
  MyMission(const std::shared_ptr<rclcpp::Node> & node) : _node(node)
  {
    const auto mission =  px4_ros2::Mission(nlohmann::json::parse(R"(
  {
    "version": 1,
    "mission": {
        "items": [
           {
               "type": "takeoff"
           },
           {
               "type": "navigation",
               "navigationType": "waypoint",
               "x": 47.3977419,
               "y": 8.5455939,
               "z": 500,
               "frame": "global"
           },
           {
               "type": "navigation",
               "navigationType": "waypoint",
               "x": 47.39791657,
               "y": 8.54595214,
               "z": 500,
               "frame": "global"
           },
           {
               "type": "customAction"
           },
           {
               "type": "rtl"
           }
        ]
    }
  }
)"));
    _mission_executor = std::make_unique<px4_ros2::MissionExecutor>("My Mission",
      px4_ros2::MissionExecutor::Configuration().addCustomAction<CustomAction>(), *node);

    if (!_mission_executor->doRegister()) {
      throw std::runtime_error("Failed to register mission executor");
    }
    _mission_executor->setMission(mission);

    _mission_executor->onProgressUpdate([&](int current_index) {
        RCLCPP_INFO(_node->get_logger(), "Current mission index: %i", current_index);
      });
    _mission_executor->onCompleted([&] {
        RCLCPP_INFO(_node->get_logger(), "Mission completed callback");
      });
  }

private:
  std::shared_ptr<rclcpp::Node> _node;
  std::unique_ptr<px4_ros2::MissionExecutor> _mission_executor;
};
```

A full example with a few custom actions can be found under [github.com/Auterion/px4-ros2-interface-lib/tree/main/examples/cpp/modes/mission/include](https://github.com/Auterion/px4-ros2-interface-lib/tree/main/examples/cpp/modes/mission/include).

## Mission Definition
The mission JSON file can contain mission defaults and a list of mission items, including user-defined types with custom arguments.
Waypoint coordinates currently need to be defined in global frame, and other frame types might be added in future.

The schema can be found under [github.com/Auterion/px4-ros2-interface-lib/blob/main/mission/schema.yaml](https://github.com/Auterion/px4-ros2-interface-lib/blob/main/mission/schema.yaml).
It provides more details and can be used to validate a JSON file.
