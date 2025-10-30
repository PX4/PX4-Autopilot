# X-Plane SITL Simulation

:::warning
This simulator is [community supported and maintained](../simulation/community_supported_simulators.md).
It may or may not work with current versions of PX4 and may be removed in future releases.
See [Toolchain Installation](../dev_setup/dev_env.md) for information about the environments and tools supported by the core development team.
:::

X-Plane is a comprehensive flight simulator that can be used with PX4 for Software-In-The-Loop (SITL) simulation.
This integration allows testing PX4 with highly realistic flight dynamics across multiple vehicle types.

## Overview

The X-Plane SITL integration provides:

- **Realistic aerodynamics**: X-Plane's industry-leading flight model
- **Multiple vehicle types**: Fixed-wing, multicopter, and VTOL support
- **Visual simulation**: 3D cockpit and external views
- **Weather simulation**: Wind, turbulence, and atmospheric conditions
- **Sensor simulation**: GPS, IMU, barometer, and airspeed sensors

## Supported Aircraft

PX4 includes pre-configured airframes for X-Plane:

| Airframe ID | Name               | Type        | Description                           |
| ----------- | ------------------ | ----------- | ------------------------------------- |
| 5001        | Cessna 172         | Fixed-Wing  | General aviation trainer              |
| 5002        | TB2                | Fixed-Wing  | UAV with A-tail configuration         |
| 5010        | Ehang 184          | Multicopter | Electric airtaxi (quadcopter)         |
| 5020        | Alia-250           | VTOL        | eVTOL quad + pusher configuration     |
| 5021        | Quantix Tailsitter | VTOL        | Quad tailsitter (differential thrust) |

## Prerequisites

### X-Plane Installation

1. **X-Plane 11 or 12** (X-Plane 12 recommended)
2. Purchase and install from [x-plane.com](https://www.x-plane.com)
3. Ensure X-Plane runs properly on your system

### px4xplane Bridge Plugin

The integration requires a bridge plugin that connects PX4 to X-Plane using the X-Plane SDK.

1. Download the latest **px4xplane** plugin from:
   - [https://github.com/alireza787b/px4xplane](https://github.com/alireza787b/px4xplane)

2. Follow the installation instructions in the px4xplane repository:
   - Place the plugin in `X-Plane/Resources/plugins/`
   - Configure connection settings (default: localhost:4560)

### PX4 Build

Build PX4 for SITL:

```bash
cd PX4-Autopilot
make px4_sitl_default
```

## Quick Start

### 1. Start X-Plane

- Launch X-Plane
- Load the aircraft model matching your desired PX4 airframe
- Position aircraft on runway or in flight

### 2. Start px4xplane Plugin

- The plugin should auto-load with X-Plane
- Verify plugin is active in X-Plane's Plugin menu
- Check that it's listening for PX4 connection

### 3. Launch PX4 with X-Plane Airframe

```bash
# Cessna 172
make px4_sitl xplane_cessna172

# TB2 Fixed-Wing UAV
make px4_sitl xplane_tb2

# Ehang 184 Airtaxi (Quadcopter)
make px4_sitl xplane_ehang184

# Alia-250 eVTOL
make px4_sitl xplane_alia250

# Quantix Tailsitter
make px4_sitl xplane_qtailsitter
```

### 4. Connect Ground Control Station

Connect QGroundControl or another GCS to PX4:

- Default connection: UDP on localhost:14550

## Configuration

### Airframe Parameters

Each X-Plane airframe has pre-tuned parameters optimized for X-Plane's flight dynamics:

- **EKF2 sensor fusion**: Optimized barometer noise (0.003m), GPS configuration
- **Flight controller tuning**: PID gains tuned for X-Plane response characteristics
- **Performance limits**: Airspeed, climb rates, loiter radius configured per aircraft
- **VTOL transitions**: Transition airspeeds and durations tuned for each VTOL type

### Network Configuration

Default connection settings:

- **PX4 → X-Plane**: Sends actuator commands
- **X-Plane → PX4**: Sends sensor data (IMU, GPS, barometer, airspeed)
- **Port**: 4560 (configurable in px4xplane plugin)

## Flight Testing

### Pre-Flight Checks

1. Verify X-Plane aircraft matches PX4 airframe
2. Check px4xplane plugin is connected
3. Verify PX4 receives sensor data (check `sensor combined` topic)
4. Arm checks pass in QGroundControl

### Basic Flight Test Procedure

#### Fixed-Wing (Cessna 172, TB2)

1. Position aircraft on runway
2. Arm vehicle in QGroundControl
3. Set takeoff mission or use guided mode
4. Monitor altitude hold and airspeed control
5. Test waypoint navigation
6. Execute landing approach

#### Multicopter (Ehang 184)

1. Position aircraft on ground or in hover
2. Arm vehicle
3. Test altitude hold and position hold
4. Test waypoint missions
5. Execute landing

#### VTOL (Alia-250, Quantix)

1. Start in multicopter mode
2. Takeoff vertically to 50m+ AGL
3. Trigger front transition (MC → FW)
4. Fly fixed-wing pattern
5. Trigger back transition (FW → MC)
6. Land vertically

## Troubleshooting

### PX4 Won't Arm

- **Check**: px4xplane plugin is loaded and connected
- **Check**: X-Plane aircraft matches PX4 airframe type
- **Check**: GPS has 3D fix (may require simulating GPS in X-Plane)
- **Solution**: Enable HIL mode or use `commander arm force`

### Altitude Oscillations

- **Cause**: Barometer noise too high
- **Solution**: Verify `EKF2_BARO_NOISE = 0.003` (not 0.1)
- **Check**: Barometer priority (`CAL_BARO1_PRIO = 100`, `CAL_BARO0_PRIO = 0`)

### Aircraft Won't Follow Waypoints

- **Check**: Flight mode is AUTO or MISSION
- **Check**: Mission uploaded and validated
- **Check**: Loiter radius `NAV_LOITER_RAD` matches aircraft performance
- **Solution**: Increase loiter radius for larger/faster aircraft

### VTOL Transition Failures

- **Altitude loss during front transition**:
  - Increase `VT_F_TRANS_THR` (more throttle)
  - Decrease `VT_F_TRANS_DUR` (faster transition)
- **Overshoot during back transition**:
  - Increase `VT_B_DEC_MSS` (more aggressive deceleration)
  - Increase transition altitude (`VT_FW_MIN_ALT`)

### X-Plane FPS Impact

- **Symptom**: Jerky flight or sensor delays
- **Solution**: Reduce X-Plane graphics settings
- **Note**: EKF2 innovation gates increased to 5.0 for FPS tolerance

## Advanced Topics

### Custom Aircraft

To add your own X-Plane aircraft:

1. Create a new airframe file in `ROMFS/px4fmu_common/init.d-posix/airframes/`
2. Use 50xx numbering (X-Plane reserved range)
3. Add entry to `sitl_targets_xplane.cmake`
4. Tune EKF2 and flight controller parameters
5. Reference existing airframes for parameter templates

### Parameter Tuning

Each airframe file contains detailed documentation:

- Aircraft specifications
- Parameter interdependencies
- Tuning guidelines
- Troubleshooting section

See airframe files for 700+ lines of inline documentation.

### MAVLink HIL Protocol

The px4xplane plugin uses MAVLink Hardware-In-The-Loop protocol:

- `HIL_SENSOR`: IMU data from X-Plane → PX4
- `HIL_GPS`: GPS data from X-Plane → PX4
- `HIL_STATE_QUATERNION`: Ground truth for analysis
- `HIL_ACTUATOR_CONTROLS`: PX4 control outputs → X-Plane

## Known Limitations

1. **Frame rate dependency**: X-Plane simulation quality depends on maintaining adequate FPS
2. **Single aircraft**: One PX4 instance per X-Plane instance
3. **Sensor simulation**: Limited to basic sensors (no lidar, cameras)
4. **Operating systems**: px4xplane plugin compatibility varies by platform

## Additional Resources

- **px4xplane Repository**: [https://github.com/alireza787b/px4xplane](https://github.com/alireza787b/px4xplane)
- **X-Plane Documentation**: [x-plane.com/kb](https://x-plane.com/kb)
- [PX4 SITL Documentation](../simulation/index.md)
- **Community Forum**: [discuss.px4.io](https://discuss.px4.io)

## Maintainer

- **Alireza Ghaderi** ([@alireza787b](https://github.com/alireza787b))
- Email: p30planets@gmail.com

## Contributing

Contributions welcome! Please submit pull requests to:

- PX4 Autopilot (airframe configurations): [PX4/PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)
- px4xplane plugin: [alireza787b/px4xplane](https://github.com/alireza787b/px4xplane)
