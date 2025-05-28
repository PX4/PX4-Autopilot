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

1. **Follow the plugin architecture** - Implement appropriate interfaces (`ISystemPreUpdate`, `ISystemPostUpdate`, etc.)
2. **Register your plugin** - Add it to `server.config` for discovery
3. **Use the custom namespace** - Follow the pattern `custom::YourPluginName`

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

## Resources

- **PX4 Plugins**: [Repository source code](https://github.com/PX4/PX4-Autopilot/tree/main/src/modules/simulation/gz_plugins)
- **Official Gazebo Documentation**: [System Plugins Guide](https://gazebosim.org/api/sim/8/createsystemplugins.html)
- **Server Configuration**: [Configuration Reference](https://gazebosim.org/api/sim/9/server_config.html)
- **PX4 Gazebo-classic Plugins**: [PX4 Gazebo Classic Plugins](https://github.com/PX4/PX4-SITL_gazebo-classic/tree/main/src)

::: info
Plugins for modern Gazebo are still evolving. The plugin system differs from Gazebo Classic.
:::
