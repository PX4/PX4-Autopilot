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

- **Realistic aerodynamics**: X-Plane's industry-leading flight model based on Blade Element Theory
- **Multiple vehicle types**: Fixed-wing, multicopter, and VTOL support
- **Visual simulation**: 3D cockpit and external views, with VR support
- **Weather simulation**: Wind, turbulence, and atmospheric conditions
- **Sensor simulation**: GPS, IMU, barometer, and airspeed sensors
- **Full-world environment**: Real-world terrain, buildings, and customizable scenery

## Key Features

### Blade Element Theory Physics

X-Plane uses Blade Element Theory (BET) for flight dynamics modeling, which divides aircraft surfaces into small elements and calculates forces based on airflow over each element. This provides:

- **Accurate aerodynamics**: Realistic stall behavior, ground effect, and control surface response
- **Physics-based simulation**: Flight characteristics derived from aircraft geometry, not lookup tables
- **Best suited for**: Larger aircraft, slower dynamics, and complex aerodynamic scenarios

### Platform Support

X-Plane SITL integration works across multiple platforms:

- **Linux**: Native support (recommended for best performance)
- **macOS**: Full support with native X-Plane installation
- **Windows**: Via WSL (Windows Subsystem for Linux) - PX4 runs in WSL while X-Plane runs natively on Windows

See [Prerequisites](#prerequisites) for WSL configuration details.

### Full World Simulation

X-Plane provides comprehensive environmental simulation:

- **Global scenery**: Entire Earth with real-world terrain and elevation data
- **Weather systems**: Dynamic weather including rain, snow, fog, and thunderstorms
- **Atmospheric effects**: Turbulence, thermals, wind shear, and realistic atmosphere modeling
- **Day/night cycles**: Full lighting simulation with realistic shadows
- **Custom scenery**: Ability to add custom airports, buildings, and environmental objects
- **VR support**: Immersive flight simulation with VR headsets

All X-Plane visual and environmental features work seamlessly with PX4 flight control.

### Custom Airframe Configuration

Beyond the 5 pre-configured airframes, you can integrate any X-Plane aircraft:

- **Flexible config.ini system**: Map any X-Plane aircraft to PX4 control channels
- **Plane Maker integration**: Any vehicle modeled in X-Plane's Plane Maker can be controlled by PX4
- **Vehicle type agnostic**: Fixed-wing, multicopter, VTOL, helicopter, or hybrid configurations
- **Custom parameter tuning**: Full access to PX4's parameter system for flight controller optimization

See [Custom Aircraft](#custom-aircraft) section and the [custom airframe guide](https://github.com/alireza787b/px4xplane/blob/master/docs/custom-airframe-config.md) for detailed configuration instructions.

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

### Platform Setup

The X-Plane SITL integration supports multiple platforms with different setup requirements:

#### Linux (Recommended)

- **Setup**: Install PX4 and X-Plane both on Linux
- **Network**: Both run on same machine, use `localhost` (127.0.0.1)
- **Performance**: Best performance, no network overhead

#### macOS

- **Setup**: Install PX4 and X-Plane both on macOS
- **Network**: Both run on same machine, use `localhost` (127.0.0.1)
- **Performance**: Native macOS performance

#### Windows (WSL)

- **Setup**: PX4 runs in WSL (Windows Subsystem for Linux), X-Plane runs natively on Windows
- **Network**: Requires WSL to Windows IP address configuration
- **Steps**:
  1. Install WSL 2 with Ubuntu 20.04 or 22.04
  2. Install PX4 development environment in WSL
  3. Find Windows host IP from WSL:

     ```bash
     # Get Windows IP address from WSL
     ip route | grep default | awk '{print $3}'
     ```

  4. Configure px4xplane plugin to use WSL IP (PX4 listens on WSL IP, not localhost)
  5. Ensure Windows Firewall allows UDP traffic on port 4560

- **Troubleshooting**: If connection fails, verify:
  - WSL can ping Windows IP
  - Firewall rules allow UDP 4560
  - px4xplane plugin shows correct PX4 IP address

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

This guide assumes you have completed the [Prerequisites](#prerequisites) setup.

### 1. Install and Configure px4xplane Plugin

1. Download the latest px4xplane plugin from [https://github.com/alireza787b/px4xplane](https://github.com/alireza787b/px4xplane)
2. Extract and place the plugin folder in `X-Plane/Resources/plugins/`
3. Configure network settings in the plugin configuration file:
   - **Linux/macOS**: PX4 IP = `127.0.0.1` (localhost)
   - **Windows (WSL)**: PX4 IP = WSL IP address (see [Platform Setup](#platform-setup))
   - Default port: `4560`

### 2. Launch X-Plane and Load Aircraft

1. Start X-Plane 11 or 12
2. Load the aircraft matching your desired PX4 airframe:
   - Cessna 172 Skyhawk (for 5001_xplane_cessna172)
   - General aviation or UAV model (for 5002_xplane_tb2)
   - Quadcopter or Ehang 184 (for 5010_xplane_ehang184)
   - VTOL or Alia-250 (for 5020_xplane_alia250)
   - Tailsitter or Quantix (for 5021_xplane_qtailsitter)
3. Position aircraft:
   - **Runway**: For takeoff testing
   - **In-flight**: For immediate flight control testing (set altitude, enable flight)
4. Verify px4xplane plugin is loaded:
   - Check **Plugins → Plugin Admin** menu
   - Plugin should show as loaded and waiting for PX4 connection

### 3. Launch PX4 SITL

Open a terminal (or WSL terminal on Windows) and run:

```bash
cd PX4-Autopilot

# Choose your airframe:

# Cessna 172 Fixed-Wing
make px4_sitl xplane_cessna172

# TB2 Fixed-Wing UAV
make px4_sitl xplane_tb2

# Ehang 184 Quadcopter
make px4_sitl xplane_ehang184

# Alia-250 eVTOL
make px4_sitl xplane_alia250

# Quantix Tailsitter VTOL
make px4_sitl xplane_qtailsitter
```

**Expected output:**

- PX4 boots with selected airframe configuration
- Plugin establishes connection (check plugin UI in X-Plane)
- Sensor data flows from X-Plane to PX4

### 4. Connect Ground Control Station

1. Launch QGroundControl (or your preferred GCS)
2. GCS should auto-connect to PX4 via UDP on `localhost:14550`
3. Verify connection:
   - Check vehicle status in QGroundControl
   - Verify GPS position matches X-Plane location
   - Check attitude indicators respond to X-Plane aircraft movement

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

### Connection Issues (No Data Flow)

- **Symptoms**: PX4 starts but receives no sensor data, plugin shows disconnected
- **Common Causes**:
  - **WSL IP mismatch**: Plugin configured with wrong PX4 IP address
  - **Firewall blocking**: Windows Firewall blocking UDP port 4560
  - **Wrong startup order**: PX4 started before X-Plane/plugin ready
- **Solutions**:
  1. Verify IP configuration:
     - **WSL**: Get IP with `ip route | grep default | awk '{print $3}'`
     - **Linux/macOS**: Use `127.0.0.1`
  2. Check firewall: Allow UDP port 4560 in Windows Firewall
  3. Startup order: Start X-Plane first, wait for plugin to load, then start PX4
  4. Verify connection in plugin UI (should show PX4 connected)

### Motors Not Rotating (Alia-250 Specific)

- **Symptom**: VTOL motors don't spin in X-Plane even when armed
- **Cause**: Known X-Plane bug with some aircraft models where motors don't animate
- **Impact**: Visual only - flight dynamics and control work normally
- **Workaround**: Ignore visual issue, verify motor outputs in QGroundControl MAVLink Inspector
- **Note**: Does not affect actual simulation, only X-Plane rendering

### Lockstep Scheduler Error

- **Symptom**: `[lockstep_scheduler] requested timestamp in the past` errors in console
- **Cause**: X-Plane FPS drops below acceptable threshold, timing desynchronization
- **Solutions**:
  1. Reduce X-Plane graphics settings (render quality, objects, shadows)
  2. Close other applications to free CPU/GPU resources
  3. Ensure X-Plane maintains at least 20 FPS
  4. If persistent, increase `LOCKSTEP_TIMEOUT` parameter

### High Accelerometer Bias Errors

- **Symptom**: `EKF2 IMU0 accel bias limit reached` warnings
- **Cause**: Large accelerometer bias estimation exceeding safety limits
- **Solutions**:
  - Verify `EKF2_ABL_LIM = 1.2` (increased from default 0.4)
  - Check `EKF2_ACC_B_NOISE = 0.0010` (enables bias learning)
  - If errors persist, aircraft may have unrealistic acceleration in X-Plane
  - Review X-Plane aircraft weight/CoG configuration

### Preflight Fail: Not Ready to Fly

- **Symptoms**: Cannot arm, QGC shows "Preflight Fail" errors
- **Common Issues**:
  1. **No GPS fix**: Wait for GPS initialization (30-60 seconds in X-Plane)
  2. **EKF not initialized**: Wait for `EKF2` to converge (check console output)
  3. **Barometer not ready**: Verify barometer priority settings
  4. **Mode switch failure**: Check RC calibration or use virtual joystick
- **Quick fix for testing**: `commander arm force` (use only for debugging)

### EKF2 Height/Velocity Unstable

- **Symptoms**: Altitude jumps, velocity estimates oscillate, "height above ground estimate not valid" warnings
- **Causes**:
  - Incorrect barometer noise settings
  - GPS vertical accuracy issues
  - Innovation gate rejections
- **Solutions**:
  1. Verify barometer configuration:
     - `EKF2_BARO_NOISE = 0.0035` (3.5mm, not default 0.1)
     - `EKF2_BARO_GATE = 8.0` (increased from default 5.0)
  2. Check EKF2 status: `listener estimator_status`
  3. Monitor innovations: `listener ekf2_innovations`
  4. If GPS issues: Verify GPS quality in X-Plane settings
  5. Increase innovation gates if needed (already set to 5.0 for X-Plane)

## Advanced Topics

### Custom Aircraft

The X-Plane SITL integration supports any aircraft that can be modeled in X-Plane. The px4xplane plugin uses a flexible **config.ini** system that maps PX4 control outputs to X-Plane control surfaces and motors.

#### Flexible Configuration System

**config.ini Channel Mapping:**

- Maps PX4's 16 actuator outputs to X-Plane control datarefs
- Supports any vehicle type: fixed-wing, multicopter, VTOL, helicopter, hybrid
- Allows custom mixing and control allocation
- No hardcoded assumptions about aircraft type

**Example mapping** (from config.ini):

```
[ChannelMapping]
channel_0 = sim/flightmodel/engine/ENGN_thro[0]  # Motor 1
channel_1 = sim/cockpit2/controls/aileron_ratio   # Aileron
channel_2 = sim/cockpit2/controls/elevator_ratio  # Elevator
# ... up to channel_15
```

Any aircraft modeled in X-Plane's **Plane Maker** can be controlled by PX4 using appropriate channel mapping.

#### Integration Steps

**For PX4 developers** (contributing new airframe to PX4 repository):

1. **Create airframe file**: `ROMFS/px4fmu_common/init.d-posix/airframes/50xx_xplane_youraircraft`
   - Use 50xx numbering (X-Plane reserved range)
   - Base on existing airframe templates (5001-5021)
   - Include comprehensive parameter documentation

2. **Add CMake target**: Update `src/modules/simulation/simulator_mavlink/sitl_targets_xplane.cmake`

   ```cmake
   add_xplane_target(xplane_youraircraft 50xx 50xx_xplane_youraircraft)
   ```

3. **Configure px4xplane plugin**: Edit `config.ini` to map PX4 outputs to X-Plane controls

4. **Tune parameters**: EKF2, flight controller, control allocation (see Parameter Tuning section)

5. **Test thoroughly**: Verify all flight modes, transitions (VTOL), edge cases

**For end users** (using custom aircraft without modifying PX4):

1. Use one of the 5 pre-configured airframes as base
2. Modify only the px4xplane plugin's **config.ini** for your aircraft
3. Tune PX4 parameters via QGroundControl
4. No PX4 code changes required

See the comprehensive [custom airframe configuration guide](https://github.com/alireza787b/px4xplane/blob/master/docs/custom-airframe-config.md) in the px4xplane repository for detailed instructions, examples, and troubleshooting.

### Parameter Tuning

Each airframe file contains detailed documentation:

- Aircraft specifications
- Parameter interdependencies
- Tuning guidelines
- Troubleshooting section

See airframe files for 700+ lines of inline documentation.

### Simulation Protocol

The px4xplane plugin uses PX4's SITL (Software-In-The-Loop) protocol for custom simulators. This protocol leverages MAVLink HIL messages for data exchange:

- `HIL_SENSOR`: IMU data from X-Plane → PX4
- `HIL_GPS`: GPS data from X-Plane → PX4
- `HIL_STATE_QUATERNION`: Ground truth state for analysis and debugging
- `HIL_ACTUATOR_CONTROLS`: PX4 control outputs → X-Plane

Note: Despite using HIL message types, this is a SITL setup (no physical hardware). The HIL messages provide a standardized interface for external simulator integration.

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
