# Multi-Vehicle Gazebo Simulation Script

## Overview

`gz_multi_vehicle.sh` is a script to easily launch multiple PX4 gz_x500 drones in Gazebo simulation with configurable positioning. All drone instances run in a tmux grid layout within your current terminal, making it easy to monitor all vehicles simultaneously.

**Platform**: Ubuntu Linux only

## Prerequisites

1. PX4 SITL must be built:
   ```bash
   make px4_sitl
   ```

2. Gazebo (gz) must be installed (Harmonic, Ionic, or Jetty)

3. **Required**: terminator and tmux:
   ```bash
   sudo apt install terminator tmux
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

1. **Validates** that PX4 SITL is built, terminator and tmux are installed
2. **Prepares** working directories for all vehicle instances
3. **Creates a tmux session** with a grid layout in your current terminal
4. **Launches all vehicles** in separate tmux panes:
   - **Instance 0** (first pane):
     - Starts Gazebo server and GUI
     - Spawns x500_0 at position (0, 0, 0)
     - MAV_SYS_ID = 1
   - **Instances 1+** (additional panes):
     - Connect to running Gazebo instance
     - Spawn x500_N at position (0, N×spacing, 0)
     - MAV_SYS_ID = N+1
     - Wait appropriately for Gazebo to be ready

All vehicles are displayed in a tiled grid layout within the current terminal window, allowing you to see all outputs simultaneously.

## Vehicle Configuration

Each vehicle gets:
- **Unique instance number**: 0, 1, 2, ...
- **Unique MAV_SYS_ID**: instance + 1 (1, 2, 3, ...)
- **Unique model name**: x500_0, x500_1, x500_2, ...
- **Unique position**: Spaced along Y-axis
- **Unique ROS 2 namespace** (if using ROS 2): px4_1, px4_2, px4_3, ...
- **Dedicated tmux pane**: Each instance displays its output in a separate pane in the grid

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

To stop all vehicles:

1. **Exit tmux session**: Press `Ctrl+B` then `D` to detach (vehicles keep running) or `Ctrl+C` in each pane to stop
2. **Kill the tmux session**:
   ```bash
   tmux kill-session -t px4_multi_<timestamp>
   ```
   Or kill all PX4 multi-vehicle sessions:
   ```bash
   tmux list-sessions | grep px4_multi | cut -d: -f1 | xargs -I {} tmux kill-session -t {}
   ```
3. **Kill all PX4 processes**:
   ```bash
   pkill -x px4
   ```

## Tmux Navigation

Useful tmux commands while in the session:
- **Navigate panes**: `Ctrl+B` then arrow keys
- **Zoom pane**: `Ctrl+B` then `Z` (toggle fullscreen for current pane)
- **Scroll in pane**: `Ctrl+B` then `[`, then use arrow keys or Page Up/Down (press `q` to exit scroll mode)
- **Detach from session**: `Ctrl+B` then `D`
- **Reattach to session**: `tmux attach-session -t px4_multi_<timestamp>`

## Troubleshooting

### Vehicles not spawning
- Check the tmux pane for the specific vehicle - all output is displayed there
- Ensure Gazebo is installed: `gz sim --versions`
- Make sure the first vehicle (instance 0) has fully started Gazebo before others spawn
- Zoom into the first pane (`Ctrl+B` then `Z`) to see full output

### Gazebo GUI not appearing
- Check the first pane (instance 0) for Gazebo startup messages
- Try setting display: `export DISPLAY=:0`

### Missing terminator or tmux
- Install required tools:
  ```bash
  sudo apt install terminator tmux
  ```

### Port conflicts
- The script automatically handles MAVLink ports (14540 + instance)
- UXRCE-DDS uses unique keys for namespacing

## Monitoring Output

All vehicles are displayed in a grid layout using tmux panes. Each pane shows the output for one vehicle instance. You can:
- View all vehicles simultaneously in the grid
- Zoom into any pane for detailed viewing (`Ctrl+B` then `Z`)
- Scroll through output history in any pane (`Ctrl+B` then `[`)
- Navigate between panes using arrow keys (`Ctrl+B` then arrow key)

## Architecture Details

The script uses tmux for terminal multiplexing and implements the multi-vehicle pattern described in the PX4 docs:
- **Tmux session**: Creates a single tmux session with multiple panes in a tiled grid layout
- **Instance 0**: Launches Gazebo server (no PX4_GZ_STANDALONE)
- **Instances 1+**: Connect to running Gazebo server (PX4_GZ_STANDALONE=1)
- **Positioning**: Each instance uses PX4_GZ_MODEL_POSE for spawn location
- **Identifiers**: Automatic MAV_SYS_ID and UXRCE_DDS_KEY assignment based on instance number
- **Synchronization**: Sequential startup with delays to ensure Gazebo is ready

## References

- [PX4 Multi-Vehicle Gazebo Docs](https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation.html)
- [PX4 ROS 2 Multi-Vehicle](https://docs.px4.io/main/en/ros2/multi_vehicle.html)
