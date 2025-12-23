# Flip After Crash Module

Module to handle flipping after a crash detection.

## Building the Module

### Prerequisites
- PX4-Autopilot source code
- CMake build system configured

### Build Steps

1. **Configure the build** (if not already done):
   ```bash
   cd /home/naor/Desktop/formic/PX4-Autopilot
   make px4_sitl
   ```
   Or for a specific board:
   ```bash
   make px4_fmu-v5_default
   ```

2. **Enable the module** in the configuration:
   - Run `make menuconfig` or `make px4_sitl menuconfig`
   - Navigate to: `Modules` → `flip_after_crash`
   - Enable the module (press Space to toggle)
   - Save and exit

3. **Build the firmware**:
   ```bash
   make px4_sitl
   ```
   Or for your target board:
   ```bash
   make px4_fmu-v5_default
   ```

4. **Start the module** (after flashing/booting):
   ```bash
   flip_after_crash start
   ```

5. **Check module status**:
   ```bash
   flip_after_crash status
   ```

6. **Stop the module**:
   ```bash
   flip_after_crash stop
   ```

## Module Structure

- `CMakeLists.txt` - Build configuration
- `Kconfig` - Module configuration options
- `flip_after_crash_main.cpp` - Main module implementation

## Implementation Notes

The module currently provides a basic framework. You'll need to implement:
- Crash detection logic
- Flip recovery maneuver logic
- Integration with vehicle control systems
