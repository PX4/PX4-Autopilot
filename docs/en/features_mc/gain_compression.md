# Gain compression

<Badge type="tip" text="PX4 v2.0" />

Automatic gain compression reduces the gains of the angular-rate PID whenever oscillations are detected.
It monitors the angular-rate controller output (the torque setpoint) through a band-pass filter to identify these oscillations, and scales the output down until they stop.
The gain recovers to 1.0 once the oscillation is gone.

This approach is a safe adaptive mechanism: the PID gains remain unchanged when no oscillations are present, they are never increased beyond their nominal values, and they are bounded by a minimum limit ([MC_GC_GAIN_MIN](../advanced_config/parameter_reference.md#MC_GC_GAIN_MIN)).

Gain compression can help prevent actuator damage, and even loss of the vehicle, when the rate loop becomes oscillatory in flight: for example after a payload change that shifts the CG or inertia, a partial propeller failure, or gains that were tuned for a different configuration.
It removes the need to retune manually for the oscillation to stop.

The algorithm is the same one used on fixed-wing vehicles, where you can also find a [block diagram](../features_fw/gain_compression.md) of the adaptive law (the multicopter rate loop has no airspeed scaling stage).

## Usage

Gain compression is disabled by default and is enabled with [MC_GC_EN](../advanced_config/parameter_reference.md#MC_GC_EN).

It should be disabled during multicopter [manual tuning](../config_mc/pid_tuning_guide_multicopter.md) to avoid over-tuning: with compression active, an oscillation caused by gains that are too high is damped out, which hides the very symptom the tuning process relies on.
It does not need to be disabled when [autotuning](../config/autotune_mc.md).

Compression is automatically reset (gain back to 1.0) while the vehicle is disarmed or landed, so that vibration from ground contact cannot compress the gains before takeoff.

## Parameters

- [MC_GC_EN](../advanced_config/parameter_reference.md#MC_GC_EN)
- [MC_GC_GAIN_MIN](../advanced_config/parameter_reference.md#MC_GC_GAIN_MIN)
