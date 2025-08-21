# VTOL Takeoff Loiter Land Flight Mode

## Overview

The `VTOL_TAKEOFF_LOITER_LAND` flight mode is a custom autonomous mode designed specifically for VTOL (Vertical Take-Off and Landing) vehicles. This mode performs a complete flight sequence:

1. **Takeoff**: Vertical takeoff to a specified altitude
2. **Transition to Fixed-Wing**: Transition from multicopter to fixed-wing mode
3. **Fixed-Wing Loiter**: Loiter in a circle at fixed-wing altitude
4. **Transition to Multicopter**: Transition back to multicopter mode
5. **Multicopter Loiter**: Loiter in a circle at multicopter altitude
6. **Landing**: Descend and land at the takeoff position

## Navigation State

- **Navigation State**: `NAVIGATION_STATE_AUTO_VTOL_TAKEOFF_LOITER_LAND` (23)
- **RC Mode Mapping**: Parameter value 16

## Parameters

### VTOL Loiter Radius (`VT_LOITER_RAD`)
- **Default**: 50.0 m
- **Range**: 10-500 m
- **Description**: Radius of the loiter circle for both fixed-wing and multicopter phases

### VTOL Loiter Duration (`VT_LOITER_TIME`)
- **Default**: 60.0 s
- **Range**: 10-600 s
- **Description**: Duration of each loiter phase (both fixed-wing and multicopter)

### VTOL Landing Altitude (`VT_LAND_ALT`)
- **Default**: 5.0 m
- **Range**: 1-50 m
- **Description**: Altitude above ground for landing phase

### Existing VTOL Parameters Used
- `VTO_LOITER_ALT`: Takeoff and loiter altitude
- `VT_TRANS_MIN_TM`: Minimum transition time
- `VT_FW_MIN_ALT`: Minimum fixed-wing altitude

## State Machine

The flight mode implements a state machine with the following states:

1. **TAKEOFF**: Vertical climb to takeoff altitude
2. **TRANSITION_TO_FW**: Transition from MC to FW mode
3. **LOITER_FW**: Fixed-wing loiter in circle
4. **TRANSITION_TO_MC**: Transition from FW to MC mode
5. **LOITER_MC**: Multicopter loiter in circle
6. **LAND**: Descend to landing altitude
7. **COMPLETE**: Sequence finished, hold position

## Files Created/Modified

### New Files
- `src/modules/flight_mode_manager/tasks/VtolTakeoffLoiterLand/FlightTaskVtolTakeoffLoiterLand.hpp`
- `src/modules/flight_mode_manager/tasks/VtolTakeoffLoiterLand/FlightTaskVtolTakeoffLoiterLand.cpp`
- `src/modules/flight_mode_manager/tasks/VtolTakeoffLoiterLand/CMakeLists.txt`
- `src/modules/flight_mode_manager/tasks/VtolTakeoffLoiterLand/vtol_takeoff_loiter_land_params.c`

### Modified Files
- `msg/versioned/VehicleStatus.msg`: Added new navigation state
- `src/modules/flight_mode_manager/CMakeLists.txt`: Added new flight task
- `src/modules/flight_mode_manager/FlightModeManager.cpp`: Added mode handling
- `src/modules/manual_control/ManualControl.cpp`: Added RC mapping
- `src/modules/commander/ModeUtil/conversions.hpp`: Added mode conversion
- `src/modules/commander/ModeUtil/mode_requirements.cpp`: Added mode requirements

## Usage

1. **Enable the mode**: Set RC switch to parameter value 16
2. **Configure parameters**: Adjust loiter radius, duration, and landing altitude as needed
3. **Arm and engage**: Arm the vehicle and switch to the mode
4. **Monitor progress**: The mode will automatically execute the complete sequence

## Safety Features

- **VTOL-only**: Mode only works on VTOL vehicles
- **Position holding**: Maintains position during transitions
- **Altitude monitoring**: Tracks altitude changes during transitions
- **Automatic completion**: Sequence completes automatically

## Testing

This mode should be tested in simulation first:
```bash
make px4_sitl gazebo-classic vtol_standard
```

Then configure the mode and test the complete sequence.
