# GNSS-Degraded & Denied Flight ("Dead-Reckoning" Mode)

<Badge type="tip" text="PX4 v1.17" /> <Badge type="warning" text="Experimental" />

::: warning Experimental
This is a new feature with limited real-world testing.
It is intended for GNSS dropout scenarios (not pure GNSS-denied from takeoff), and requires that alternative velocity/position sensors are available.

Please [share your related test logs](../getting_started/flight_reporting.md#sharing-the-log-files-for-review-by-px4-developers) to help us verify and harden it.
:::

PX4 is default-configured for outdoor flight with a reliable GNSS signal, but it can also be set up in "dead-reckoning mode" to more gracefully handle environments where GNSS is intermittently degraded or denied during flight.

This section describes the differences between automatic and dead-reckoning modes, the circumstances in which each should be used, and how dead-reckoning is configured.

## Overview

PX4's EKF2 navigation has two modes for handling when GNSS data is determined to be unreliable:

- **Automatic mode** (the default): Used for flying outdoors in environments where a GNSS signal is expected to be largely reliable.
- **Dead-reckoning mode**: Recommended when you want to fly missions or other position controlled modes when there is intermittent GNSS loss, such as when flying under a bridge, from outdoors into an indoor setting, or when there is GNSS jamming (it is not suitable for pure-indoor use, as a GNSS signal is required before arming).

::: info
Dead-reckoning mode helps for both Fixed-Wing and Multicopter vehicles.
MC vehicles benefit more because they can hover when transitioning between sensor regimes.
FW needs continuous accurate velocity/position during the entire mission arc, making sensor transitions trickier.
:::

## Mode Comparison

The following sections provide more detail about each of the modes and when they should be used.

### Automatic Mode

In Automatic mode the EKF2 resets if GNSS is lost and no other sources of position are available.
This can result in a [position loss failsafe](../config/safety.md#position-loss-failsafe) and may trigger a shift into a mode that does not require global position, including stopping missions.

This is desirable if the GNSS signal is likely to be recovered quickly and there are no mechanisms to estimate position when GNSS is unavailable.

Use Automatic (default) when:

- Flying in open sky with reliable GNSS throughout the mission.
- You want the EKF to reset to GNSS when it becomes available again.
- Operating in environments where GNSS is either good or completely unavailable (binary state).

### Dead-Reckoning Mode

In dead-reckoning mode, EKF2 stops fusing GNSS data when it becomes unreliable and prevents EKF2 resets — provided there are other sources of position or velocity data that can be fused.
This ensures that the vehicle can continue flying missions and other position controlled modes when GNSS is lost.

When GNSS is recovered it will be fused with other measurements when tests indicate it can be trusted.
This may cause jerky movements in position controlled modes if the estimate has drifted.
This mode relies on having additional position or velocity sensors and must also have a reliable GNSS signal at boot.

Use Dead-Reckoning when:

- **Transitioning between GNSS and non-GNSS environments** (flying into buildings, under bridges, through tree cover).
- You have **redundant sensors** (optical flow, VIO, rangefinder, quality baro) that can maintain position estimation.
- Flying **missions that cross GPS-denied areas** where you want continuous operation rather than failsafe.
- **Urban environments** or other areas with intermittent GNSS quality.
- You want to **avoid EKF resets and jumps** when GNSS recovers (smoother transitions).

## Configuration

To use dead-reckoning mode, the vehicle must have an alternative source of position or velocity information, such as an [Optical Flow](../sensor/optical_flow.md) sensor or [VIO](../computer_vision/visual_inertial_odometry.md) setup.

To enable the mode:

1. Set [EKF2_GPS_MODE](../advanced_config/parameter_reference.md#EKF2_GPS_MODE) to `1`.
2. Ensure that GNSS arming checks are enabled (a reliable GNSS signal is required before arming):
   - [COM_ARM_WO_GPS](../advanced_config/parameter_reference.md#COM_ARM_WO_GPS) - set to `0`
   - [EKF2_GPS_CHECK](../advanced_config/parameter_reference.md#EKF2_GPS_CHECK) - set to default.

## See Also

- [GNSS Fault Detection](../advanced_config/tuning_the_ecl_ekf.md#gnss-fault-detection) in _Using PX4's Navigation Filter (EKF2)_
- [Fuse, Reset, or Reject? Handling Various Data-sources in EKF2](https://www.youtube.com/watch?v=CMGQJNPiTJg) - _PX4 Developer Summit 2025_, Marco Hauswirth, Auterion AG
