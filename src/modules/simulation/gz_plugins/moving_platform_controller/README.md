# Moving Platform Controller

This plugin controls a moving platform by sending velocity commands, so it can
emulate ships/trucks/etc. to takeoff and land on. The platform moves at a
constant mean velocity, with added random fluctuations in velocity and angular
velocity.

## Dependencies

This depends on the [moving platform world](https://github.com/PX4/PX4-gazebo-models/blob/moving_platform_world/worlds/moving_platform.sdf) in the `PX4-gazebo-models` repo, so ensure that the `Tools/simulation/gz` submodule is recent enough.

That world contains the corresponding [moving platform model](https://github.com/PX4/PX4-gazebo-models/blob/moving_platform_world/models/moving_platform/model.sdf). Within that, we include this plugin, and also the VelocityControl plugin to directly modify the model's velocity.


## Usage & Configuration

Start by selecting the moving_platform world, which loads the plugin. We need to set the pose so the aircraft is above the platform, which is at a height of 2m.

```
PX4_GZ_MODEL_POSE=0,0,2.2,0,0,0 PX4_GZ_WORLD=moving_platform make px4_sitl gz_standard_vtol
```

The velocity (in m/s) can be set with the `PX4_GZ_PLATFORM_VEL` environment variable. By default it is 1 m/s.

```
PX4_GZ_PLATFORM_VEL=5 PX4_GZ_MODEL_POSE=0,0,2.2,0,0,0 PX4_GZ_WORLD=moving_platform make px4_sitl gz_standard_vtol
```

To use the plugin with a *different* world or model, add the following to the model.sdf:

```
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="flat_platform">

    <!-- the rest of your model -->

    <plugin
      filename="gz-sim-velocity-control-system"
      name="gz::sim::systems::VelocityControl">
    </plugin>

    <plugin
      filename="libMovingPlatformController.so"
      name="custom::MovingPlatformController">
    </plugin>

  </model>
</sdf>
```

## Limitations & Future Ideas

 - Apart from the velocity, nothing is configurable: Noise amplitude and frequency spectrum, movement direction, initial acceleration.
 - This plugin does not communicate the state of the platform with PX4. If that is needed, there are a couple options:
    - Read the pose of the platform in `GZBridge::poseInfoCallback`
    - Add an IMU sensor to the platform link, listen to it and the existing NavSat sensor from `GZBridge`.
    - Add a custom gazebo transport message containing all needed info about the platform, populate and publish it from the plugin here, listen to that in `GZBridge`.
