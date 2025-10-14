# Multi-Vehicle Gazebo Simulation Script

## Overview

`gz_multi_vehicle.sh` is a script to easily launch multiple PX4 gz_x500 drones in Gazebo simulation with configurable positioning. Each drone instance runs in its own terminal window for easy monitoring.

**Platform**: Ubuntu Linux only

## Prerequisites

1. PX4 SITL must be built:
   ```bash
   make px4_sitl
   ```

2. Gazebo (gz) must be installed (Harmonic, Ionic, or Jetty)

3. A terminal emulator (terminator or gnome-terminal):
   ```bash
   # Recommended:
   sudo apt install terminator

   # Or use gnome-terminal:
   sudo apt install gnome-terminal
   ```

## Usage

```bash
./Tools/simulation/gz_multi_vehicle.sh [num_vehicles] [spacing]
```

### Parameters

- `num_vehicles` (optional, default: 2, max: 10): Number of drones to spawn
- `spacing` (optional, default: 0.5): Distance between drones in meters along the Y-axis

### Examples

Launch 3 drones with default 0.5m spacing:
```bash
./Tools/simulation/gz_multi_vehicle.sh 3
```

Launch 5 drones with 1.0m spacing:
```bash
./Tools/simulation/gz_multi_vehicle.sh 5 1.0
```

Launch 2 drones (default configuration):
```bash
./Tools/simulation/gz_multi_vehicle.sh
```

## What the Script Does

1. **Validates** that PX4 SITL is built and terminal emulator is available
2. **Cleans up** any existing PX4 and Gazebo instances
3. **Launches the first vehicle in a new terminal** (instance 0):
   - Starts Gazebo server and GUI
   - Spawns x500_0 at position (0, 0, 0)
   - MAV_SYS_ID = 1
   - Terminal title: "PX4 Vehicle 0 (MAV_SYS_ID=1)"
4. **Launches subsequent vehicles in separate terminals** (instances 1, 2, ...):
   - Connects to running Gazebo instance
   - Spawns x500_N at position (0, N×spacing, 0)
   - MAV_SYS_ID = N+1
   - Terminal title: "PX4 Vehicle N (MAV_SYS_ID=N+1)"

Each vehicle runs in its own terminal window, making it easy to monitor the output and debug individual drones.

## Vehicle Configuration

Each vehicle gets:
- **Unique instance number**: 0, 1, 2, ...
- **Unique MAV_SYS_ID**: instance + 1 (1, 2, 3, ...)
- **Unique model name**: x500_0, x500_1, x500_2, ...
- **Unique position**: Spaced along Y-axis
- **Unique ROS 2 namespace** (if using ROS 2): px4_1, px4_2, px4_3, ...
- **Dedicated terminal window**: Each instance displays its output in a separate terminal

## Using with QGroundControl

QGroundControl should automatically detect and connect to all vehicles. You can switch between them in the vehicle dropdown menu.

## Using with ROS 2

To use the vehicles with ROS 2:

1. Start the MicroXRCE-DDS Agent (all vehicles connect to the same agent):
   ```bash
   MicroXRCEAgent udp4 -p 8888
   ```

2. List topics to see all vehicles:
   ```bash
   ros2 topic list
   ```

You should see topics namespaced by vehicle:
- `/px4_1/fmu/in/...`
- `/px4_1/fmu/out/...`
- `/px4_2/fmu/in/...`
- `/px4_2/fmu/out/...`
- etc.

## Stopping the Simulation

To stop all vehicles, you can either:

1. **Close the terminal windows** - Each terminal will prompt you to press Enter before closing
2. **Kill all PX4 processes**:
   ```bash
   pkill -x px4
   ```

To stop Gazebo:
```bash
gz sim -k
```

## Troubleshooting

### Vehicles not spawning
- Check the terminal window for the specific vehicle - all output is displayed there
- Ensure Gazebo is installed: `gz sim --versions`
- Make sure the first vehicle (instance 0) has fully started Gazebo before others spawn

### Gazebo GUI not appearing
- Check if running in headless mode
- Try setting display: `export DISPLAY=:0`

### Terminal emulator not found
- Install terminator (recommended): `sudo apt install terminator`
- Or install gnome-terminal: `sudo apt install gnome-terminal`

### Port conflicts
- The script automatically handles MAVLink ports (14540 + instance)
- UXRCE-DDS uses unique keys for namespacing

## Monitoring Output

Each vehicle displays its output in its own terminal window. The terminal title shows the vehicle instance and MAV_SYS_ID for easy identification. If a vehicle crashes or exits, the terminal will remain open and prompt you to press Enter before closing, allowing you to review any error messages.

## Architecture Details

The script implements the multi-vehicle pattern described in the PX4 docs:
- First instance launches Gazebo server (no PX4_GZ_STANDALONE)
- Subsequent instances connect to running server (PX4_GZ_STANDALONE=1)
- Each instance uses PX4_GZ_MODEL_POSE for positioning
- Automatic MAV_SYS_ID and UXRCE_DDS_KEY assignment based on instance number

## References

- [PX4 Multi-Vehicle Gazebo Docs](https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation.html)
- [PX4 ROS 2 Multi-Vehicle](https://docs.px4.io/main/en/ros2/multi_vehicle.html)
