# Gazebo Plugins

Gazebo plugins extend the simulator with custom functionality not provided by default. They can be attached to different entity types and allow you to add new sensors, modify world physics, or interact with the simulation environment.

## Plugin Types

Plugins can be attached to these entity types:

- **World** - Global simulation behavior
- **Model** - Specific model functionality
- **Sensor** - Custom sensor implementations
- **Actor** - Dynamic entity behavior

## Supported Plugins

PX4 currently supports these plugins:

- [OpticalFlowSystem](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/gz_plugins/optical_flow): Provides optical flow sensor simulation using OpenCV-based flow calculation.
- [GstCameraSystem](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/gz_plugins/gstreamer): Streams camera feeds via UDP (RTP/H.264) or RTMP with optional NVIDIA CUDA hardware acceleration.
- [MovingPlatformController](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/gz_plugins/moving_platform_controller): Controls moving platforms (ships, trucks, etc.) for takeoff and landing scenarios.
  Includes configurable velocity, heading, and random fluctuations.
- [CatapultSystem](#catapult): Applies a launch force to a vehicle link, as in Gazebo Classic.

## Plugin Registration

Plugins must be registered in the [server.config](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/simulation/gz_bridge/server.config) file to be available in your world:

```xml
<server_config>
  <plugins>
    <!-- Core Gazebo systems -->
    <plugin entity_name="*" entity_type="world" filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>

    <!-- Custom PX4 plugins -->
    <plugin entity_name="*" entity_type="world" filename="libOpticalFlowSystem.so" name="custom::OpticalFlowSystem"/>
    <plugin entity_name="*" entity_type="world" filename="libGstCameraSystem.so" name="custom::GstCameraSystem"/>
  </plugins>
</server_config>
```

## Creating Custom Plugins

When developing new plugins:

1. **Follow the plugin architecture** - Implement desired interfaces.

   You can start by copying the [Template plugin](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/gz_plugins/template_plugin) which is a simple example that only implements `ISystemPreUpdate` and `ISystemPostUpdate`.
   The interfaces are specified in the official [Gazebo documentation](https://gazebosim.org/api/sim/9/createsystemplugins.html).

2. **Register your plugin** - Add it to [server.config](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/simulation/gz_bridge/server.config) for discovery.
3. **Use the custom namespace** - Follow the pattern `custom::YourPluginName`.

### Example Plugin Structure

```cpp
class YourCustomSystem :
    public gz::sim::System,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
public:
    void PreUpdate(const gz::sim::UpdateInfo &_info,
                   gz::sim::EntityComponentManager &_ecm) final;
    void PostUpdate(const gz::sim::UpdateInfo &_info,
                    const gz::sim::EntityComponentManager &_ecm) final;
};

// Plugin registration
GZ_ADD_PLUGIN(YourCustomSystem, gz::sim::System,
              YourCustomSystem::ISystemPreUpdate,
              YourCustomSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(YourCustomSystem, "custom::YourCustomSystem")
```

## Enabling a Plugin

For world plugins all you need to do is [register the plugin](#plugin-registration) (add it to the `server.config`).
It will then be available to all worlds and vehicles.

Model plugins are added inside the vehicle's SDF `<model>` element.
For example, the catapult below is enabled only for the model containing it.

## Catapult

`CatapultSystem` ports Gazebo Classic's motor-triggered catapult to modern Gazebo.
It applies a force at the selected link's centre of mass, along a normalised world-frame direction, without a visible launcher.
Add it inside the vehicle's SDF `<model>` element:

```xml
<plugin filename="libCatapultSystem.so" name="custom::CatapultSystem">
  <link_name>base_link</link_name>
  <motorNumber>0</motorNumber>
  <commandSubTopic>/command/motor_speed</commandSubTopic>
  <force>50.0</force>
  <duration>0.5</duration>
  <direction>1 0 2</direction>
</plugin>
```

The plugin subscribes to `/<model_name>/command/motor_speed` and starts the launch when the selected motor command exceeds 100 rad/s.
`motorNumber` is zero-based and defaults to 0; Classic's `plane_catapult` uses motor 4, while Gazebo's RC Cessna uses motor 0.
`commandSubTopic` is appended to `/<model_name>`.
`link_name` is required and supports nested links such as `plane::base_link`.
The force and duration above match Classic's `plane_catapult` model; plugin defaults are 1 N, 0.01 seconds, and direction `1 0 2`.

As in Classic, the first qualifying update starts the timer, subsequent qualifying updates apply the full force, and the launch finishes after an update exceeds the configured duration.
Force is applied only while the motor command exceeds the threshold; lowering it does not restart the timer.
Pausing stops simulation time, and resetting the simulation reloads the catapult.
Empty, short, or non-finite motor commands do not trigger a launch.

For fixed-wing takeoff, configure [catapult launch detection](../flight_modes_fw/takeoff.md#catapult-hand-launch) and enough idle motor output to exceed the trigger threshold.

## Resources

- **PX4 Plugins**: [Repository source code](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/gz_plugins)
- **Official Gazebo Documentation**: [System Plugins Guide](https://gazebosim.org/api/sim/9/createsystemplugins.html)
- **Server Configuration**: [Configuration Reference](https://gazebosim.org/api/sim/9/server_config.html)
- **PX4 Gazebo-classic Plugins**: [PX4 Gazebo Classic Plugins](https://github.com/PX4/PX4-SITL_gazebo-classic/tree/main/src)

::: info
Plugins for modern Gazebo are still evolving. The plugin system differs from Gazebo Classic.
:::
