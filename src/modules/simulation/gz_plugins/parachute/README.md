# Parachute Plugin

Simulates a parachute for a PX4 vehicle model. The parachute deploys on the release
message that the PX4 gz bridge forwards on `/model/<name>/parachute`. From then on the plugin
applies a velocity-dependent canopy drag force (`F = -1/2 * rho * CdA * |v_air| * v_air`,
drifting with the world wind) plus an angular damping torque to the configured link.
Deployment holds until the simulation is reset.

The resulting steady-state sink rate is `sqrt(2 * m * g / (rho * CdA))` for vehicle mass
`m`. Pick `cd_a` to match the configuration of the vehicle.

## Usage

Add to the model SDF:

```xml
<plugin filename="ParachutePlugin" name="gz::sim::systems::ParachuteSystem">
  <link_name>base_link</link_name>
  <cd_a>0.64</cd_a>
  <angular_damping>1.0</angular_damping>
</plugin>
```

All elements are optional (defaults: `base_link`, 1.3, 1.0). `<trigger_topic>` overrides
the trigger subscription (`gz.msgs.Boolean`).
