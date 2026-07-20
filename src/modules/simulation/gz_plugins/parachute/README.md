# Parachute Plugin

Simulates a parachute for a PX4 vehicle model. The parachute deploys when the observed
servo channel (the output the PX4 `Parachute` output function is mapped to) crosses a
threshold, and from then on applies a velocity-dependent canopy drag force
(`F = -1/2 * rho * CdA * |v_air| * v_air`, drifting with the world wind) plus an angular
damping torque to the configured link. Deployment latches until the simulation is reset.

The resulting steady-state sink rate is `sqrt(2 * m * g / (rho * CdA))` for vehicle mass
`m`. Pick `cd_a` to match the parachute of the vehicle.

## Usage

Map the PX4 parachute output to a free servo channel, e.g. `SIM_GZ_SV_FUNC7 = 401`
(note: `SIM_GZ_SV_FUNCn` publishes on gz topic `servo_<n-1>`), and add to the model SDF:

```xml
<plugin filename="ParachutePlugin" name="gz::sim::systems::ParachuteSystem">
  <link_name>base_link</link_name>
  <servo_index>6</servo_index>
  <cd_a>1.3</cd_a>
  <angular_damping>1.0</angular_damping>
</plugin>
```

All elements are optional (defaults shown). `<trigger_topic>` overrides the servo topic,
`<trigger_threshold>` (default 0.0) sets the deployment threshold on the received value.
