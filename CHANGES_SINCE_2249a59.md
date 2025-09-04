# Changes Since Commit 2249a5976ecb5e6b855951dde2b861ab5acc786a

This document provides a detailed analysis of all changes made to the PX4 Autopilot Rocket codebase since commit `2249a5976ecb5e6b855951dde2b861ab5acc786a` (Update GZ submodule to use new repository: https://gitlab.forge.hefr.ch/grid/rocket/px4-gazebo-models-rocket.git).

Other possibility is to check the changes with the compare revisions tab in Gitlab :
https://gitlab.forge.hefr.ch/grid/rocket/px4-autopilot-rocket/-/compare/2249a5976ecb5e6b855951dde2b861ab5acc786a...ddb0a52bd867a3e5f0a6232ed98fcffea50f7e46

## Overview

**Total Commits:** 5
**Author:** Noé Renevey <noe.renevey@edu.hefr.ch>
**Date Range:** August 11, 2025 - September 3, 2025
**Branch:** GRID-Rocket

---

## Commit 1: 46f29ce31ca6f79ed0a29faa43db23098eb75a90
**Date:** Mon Aug 11 18:11:00 2025 +0200
**Message:** Added first draft of rocket mode manager

### Summary
This commit introduces the foundation for rocket-specific flight modes and simulation support. It adds a completely new module called `rocket_mode_manager` and creates the necessary infrastructure for rocket flight simulation.

### New Files Added (A):

1. **`ROMFS/px4fmu_common/init.d-posix/airframes/71010_gz_rocket_plane`**
   - **Purpose:** Airframe configuration file for rocket simulation in Gazebo
   - **Description:** Defines the rocket plane airframe parameters, sensors, and initial configuration for SITL (Software In The Loop) simulation
   - **Lines:** 138 lines

2. **`rocket_thrust.sh`**
   - **Purpose:** Shell script for thrust management
   - **Description:** contains commands or configurations related to rocket thrust control in gazebo
   - **Lines:** 17 lines

3. **`src/modules/rocket_mode_manager/CMakeLists.txt`**
   - **Purpose:** Build configuration for the rocket mode manager module
   - **Description:** Defines how the rocket mode manager module should be compiled and linked

4. **`src/modules/rocket_mode_manager/Kconfig`**
   - **Purpose:** Kernel configuration options for the rocket mode manager
   - **Description:** Defines configuration options that can be enabled/disabled during build

5. **`src/modules/rocket_mode_manager/module.yaml`**
   - **Purpose:** Module metadata and parameter definitions
   - **Description:** Contains module description, parameters, and configuration options
   - **Lines:** 119 lines

6. **`src/modules/rocket_mode_manager/rocket_mode_manager.cpp`**
   - **Purpose:** Main implementation of the rocket mode manager
   - **Description:** Core logic for managing rocket flight phases, mode transitions, and rocket-specific behaviors
   - **Lines:** 568 lines

7. **`src/modules/rocket_mode_manager/rocket_mode_manager.hpp`**
   - **Purpose:** Header file for rocket mode manager
   - **Description:** Class definitions, function declarations, and constants for the rocket mode manager
   - **Lines:** 167 lines

### Modified Files (M):

1. **`ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`**
   - **Purpose:** Updated to include the new rocket airframe in the build system

2. **`Tools/simulation/gz`**
   - **Purpose:** Updated Gazebo simulation tools (submodule update)

3. **`boards/px4/sitl/default.px4board`**
   - **Purpose:** Board configuration updated to enable rocket mode manager module in SITL builds

### Key Features Introduced:
- Complete rocket mode management system
- Gazebo simulation support for rocket
- Foundation for rocket-specific flight modes
- Integration with PX4's existing flight stack

---

## Commit 2: 5b8c0a52c16ea6a974fb1a9831ebd4dde1289a2e
**Date:** Mon Aug 18 19:55:32 2025 +0200
**Message:** Added RocketRoll mode, WingDeploy function for servo and dynamic CA_TRQ and CS_TYPE allocation during flight.

### Summary
This commit significantly expands the rocket capabilities by adding new flight modes (RocketRoll), wing deployment functionality, and dynamic control allocation. It also adds new message types and integrates rocket modes into the commander system.

### New Files Added (A):

1. **`msg/wing_deploy_command.msg`**
   - **Purpose:** uORB message definition for wing deployment commands
   - **Description:** Defines the message structure for commanding wing deployment
   - **Lines:** 3 lines

2. **`src/lib/mixer_module/functions/FunctionWingDeploy.hpp`**
   - **Purpose:** Wing deployment function implementation
   - **Description:** Implements the logic for controlling wing deployment servos
   - **Lines:** 67 lines

3. **`src/modules/CMakeLists.txt`**
   - **Purpose:** Empty CMake file not used

### Modified Files (M):

#### Core System Files:
1. **`ROMFS/px4fmu_common/init.d-posix/airframes/71010_gz_rocket_plane`**
   - **Changes:** Enhanced rocket airfram sitl configuration with new parameters and settings

#### Message System:
2. **`msg/CMakeLists.txt`**
   - **Changes:** Added wing_deploy_command message to build system

3. **`msg/versioned/VehicleStatus.msg`**
   - **Changes:** Added new vehicle status fields for rocket modes (6 lines changed)

#### Driver Updates:
4. **`src/drivers/osd/atxxxx/atxxxx.cpp`**
   - **Changes:** Updated OSD driver to support rocket modes (4 lines added)

5. **`src/drivers/rc/crsf_rc/CrsfRc.cpp`**
   - **Changes:** Enhanced CRSF RC support (4 lines added)

6. **`src/drivers/rc_input/crsf_telemetry.cpp`**
   - **Changes:** Updated CRSF telemetry support (4 lines added)

#### Mixer Module Enhancements:
7. **`src/lib/mixer_module/mixer_module.cpp`**
   - **Changes:** Integrated wing deploy functionality

8. **`src/lib/mixer_module/mixer_module.hpp`**
   - **Changes:** Added wing deploy function declarations

9. **`src/lib/mixer_module/output_functions.yaml`**
   - **Changes:** Added wing deploy to output functions (3 lines added)

#### UI and Mode System:
10. **`src/lib/modes/ui.hpp`**
    - **Changes:** Updated UI definitions for rocket modes (8 lines changed)

#### Commander System (Flight Mode Management):
11. **`src/modules/commander/Commander.cpp`**
    - **Changes:** Integrated rocket modes into main commander logic (8 lines changed)

12. **`src/modules/commander/ModeManagement.cpp`**
    - **Changes:** Added rocket mode management (12 lines added)

13. **`src/modules/commander/ModeUtil/control_mode.cpp`**
    - **Changes:** Extensive modifications to support rocket control modes (207 lines modified)

14. **`src/modules/commander/ModeUtil/conversions.hpp`**
    - **Changes:** Added rocket mode conversions (5 lines changed)

15. **`src/modules/commander/ModeUtil/mode_requirements.cpp`**
    - **Changes:** Updated mode requirements for rocket modes (5 lines changed)

16. **`src/modules/commander/module.yaml`**
    - **Changes:** Updated commander module configuration

17. **`src/modules/commander/px4_custom_mode.h`**
    - **Changes:** Added custom mode definitions for rocket (10 lines changed)

#### Manual Control and Navigation:
18. **`src/modules/manual_control/ManualControl.cpp`**
    - **Changes:** Integrated rocket mode support in manual control

19. **`src/modules/navigator/navigator_main.cpp`**
    - **Changes:** Updated navigator for rocket modes

#### Rocket Mode Manager Updates:
20. **`src/modules/rocket_mode_manager/module.yaml`**
    - **Changes:** Significantly updated param configuration (121 lines modified)

21. **`src/modules/rocket_mode_manager/rocket_mode_manager.cpp`**
    - **Changes:** Major enhancements to rocket mode logic (397 lines modified)

22. **`src/modules/rocket_mode_manager/rocket_mode_manager.hpp`**
    - **Changes:** Updated header with new functionality (33 lines modified)

### Key Features Introduced:
- **RocketRoll Flight Mode:** New flight mode specifically for rocket roll control
- **Wing Deploy System:** Complete servo-based wing deployment functionality
- **Dynamic Control Allocation:** Runtime changes to CA_TRQ and CS_TYPE parameters
- **Enhanced Message System:** New uORB messages for rocket operations
- **Improved Integration:** Better integration with PX4's commander and control systems

---

## Commit 3: 47a7ac914ee442a3912c0a4300f3f83ca31c9852
**Date:** Fri Aug 29 16:50:23 2025 +0200
**Message:** Multiple enhancements including RC control, Rocket Passive mode, and production-ready configuration

### Summary
This is the most comprehensive commit, adding production-ready features including RC control for wing deploy and camera trigger, a new Rocket Passive flight mode, CRSF support for FMU-v6x, and a complete plug-and-play airframe configuration. It also includes documentation and addresses real-world flight testing requirements.

### New Files Added (A):

1. **`ROMFS/px4fmu_common/init.d/airframes/71010_rocket_plane`**
   - **Purpose:** Production airframe configuration (non-SITL version)
   - **Description:** Complete plug-and-play rocket plane configuration with pre-configured parameters for minimal QGC setup
   - **Lines:** 287 lines
   - **Key Features:** Prepared for 01.09.25 flight test of Par-Belenos

### Modified Files (M):

#### Airframe System:
1. **`ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt`**
   - **Changes:** Added production rocket airframe to build system (3 lines added)

#### Hardware Configuration:
2. **`boards/px4/fmu-v6x/default.px4board`**
   - **Changes:** Enabled CRSF (Crossfire) protocol support for FMU-v6x hardware (3 lines changed)

#### Message System:
3. **`msg/versioned/VehicleStatus.msg`**
   - **Changes:** Added Rocket Passive mode to vehicle status (10 lines changed)

#### Camera Trigger System:
4. **`src/drivers/camera_trigger/camera_trigger.cpp`**
   - **Changes:** Major enhancement adding RC control capability (91 lines added)
   - **Features:** RC switch control for camera triggering

5. **`src/drivers/camera_trigger/camera_trigger_params.c`**
   - **Changes:** Added new parameters for RC control (34 lines added)

#### Wing Deploy Enhancements:
6. **`src/lib/mixer_module/functions/FunctionWingDeploy.hpp`**
   - **Changes:** Enhanced with RC control functionality (48 lines changed)

7. **`src/lib/mixer_module/params.c`**
   - **Changes:** Added new mixer parameters (20 lines added)

#### Flight Mode System:
8. **`src/lib/modes/ui.hpp`**
   - **Changes:** Updated UI for Rocket Passive mode (16 lines changed)

#### Commander System Updates:
9. **`src/modules/commander/Commander.cpp`**
   - **Changes:** Integrated Rocket Passive mode (8 lines changed)

10. **`src/modules/commander/ModeUtil/control_mode.cpp`**
    - **Changes:** Added control logic for Rocket Passive mode (28 lines changed)

11. **`src/modules/commander/ModeUtil/conversions.hpp`**
    - **Changes:** Added conversion support for new mode (7 lines changed)

12. **`src/modules/commander/ModeUtil/mode_requirements.cpp`**
    - **Changes:** Updated mode requirements (8 lines changed)

13. **`src/modules/commander/module.yaml`**
    - **Changes:** Updated commander configuration (3 lines changed)

14. **`src/modules/commander/px4_custom_mode.h`**
    - **Changes:** Added Rocket Passive custom mode (7 lines changed)

#### Manual Control:
15. **`src/modules/manual_control/ManualControl.cpp`**
    - **Changes:** Added support for new rocket modes

#### Rocket Mode Manager Updates:
16. **`src/modules/rocket_mode_manager/module.yaml`**
    - **Changes:** Removed 24 lines (cleanup and optimization)

17. **`src/modules/rocket_mode_manager/rocket_mode_manager.cpp`**
    - **Changes:** Major refactoring and optimization (314 lines modified)
    - **Key Changes:**
      - Boost phase now uses Rocket Passive mode
      - Coast phase activates roll control with Rocket Roll
      - Rate-only control for Rocket Roll mode (no attitude control)

18. **`src/modules/rocket_mode_manager/rocket_mode_manager.hpp`**
    - **Changes:** Updated header definitions (11 lines changed)

### Key Features Introduced:
- **Rocket Passive Flight Mode:** Completely disables all control for passive flight phases
- **RC Control Integration:** Wing deploy and camera trigger now controllable via RC switches
- **CRSF Protocol Support:** Enabled Crossfire protocol for FMU-v6x hardware
- **Production Configuration:** Plug-and-play airframe with minimal QGC setup required
- **Enhanced Documentation:** Comprehensive parameter documentation

### TODO Items (as noted in commit):
- Fix HIGH state of PWM ports/GPIO during reboot (pending Dronecode foundation input)
- Fix `configure_fixedwing_ca_parameters()` function to properly set CA for Servos 5 and 6 (ailerons)

---

## Commit 4: 63c775fbd7ea07c26c41b35cc4af1036c3065715 (HEAD)
**Date:** Mon Sep 1 18:05:28 2025 +0200
**Message:** Launch detection refinements and parameter cleanup

### Summary
This commit focuses on refining the launch detection system by simplifying the logic to use only acceleration-based detection and removing velocity-based parameters. It also adjusts default altitude thresholds for better performance.

### Modified Files (M):

1. **`ROMFS/px4fmu_common/init.d/airframes/71010_rocket_plane`**
   - **Changes:** Updated default altitude threshold parameter (14 lines modified)
   - **Key Change:** RKT_ALT_thresh default changed to 1 meter

2. **`src/modules/rocket_mode_manager/module.yaml`**
   - **Changes:** Removed RKT_LAUNCH_V parameter and related configuration (30 lines removed)
   - **Cleanup:** Simplified parameter set by removing velocity-based launch detection

3. **`src/modules/rocket_mode_manager/rocket_mode_manager.cpp`**
   - **Changes:** Refactored launch and boost detection logic (36 lines modified)
   - **Key Changes:**
     - Removed velocity-based launch detection logic
     - Simplified to use only X-axis acceleration for launch detection
     - Improved boost detection using only X-axis acceleration

4. **`src/modules/rocket_mode_manager/rocket_mode_manager.hpp`**
   - **Changes:** Removed velocity-related member variables (1 line removed)

### Key Improvements:
- **Simplified Launch Detection:** Now uses only acceleration-based detection, removing complex velocity logic
- **X-Axis Focus:** Both launch and boost detection now focus on X-axis acceleration only
- **Better Default Values:** Altitude threshold set to more practical 1-meter default
- **Code Cleanup:** Removed unnecessary parameters and simplified codebase

---

## Commit 5: ddb0a52bd867a3e5f0a6232ed98fcffea50f7e46 (HEAD)
**Date:** Wed Sep 3 13:46:11 2025 +0200
**Message:** -Cleaned unused file (rocketparam.md and cmakelist) -Updated GZ airframe config from the flight tested one -Added missing description of "Rocket Passive" mode in crsf telem, OSD and Nav. -Updated and cleaned comment in code.

### Summary
This commit performs cleanup operations, updates the Gazebo airframe configuration based on flight-tested parameters, and adds missing Rocket Passive mode support to telemetry drivers. It also includes code comment improvements and removes unused files.

### Files Deleted (D):

1. **`docs/rocket_params.md`**
   - **Changes:** File removed (40 lines deleted)
   - **Purpose:** Cleanup of unused documentation file

2. **`docs/rocket_params.tex`**
   - **Changes:** Empty file removed
   - **Purpose:** Cleanup of unused LaTeX file

### Modified Files (M):

1. **`ROMFS/px4fmu_common/init.d-posix/airframes/71010_gz_rocket_plane`**
   - **Changes:** Major configuration update based on flight-tested parameters (167 lines modified)
   - **Key Updates:** Gazebo airframe configuration synchronized with production airframe

2. **`src/drivers/osd/atxxxx/atxxxx.cpp`**
   - **Changes:** Added Rocket Passive mode support to OSD display (4 lines added)
   - **Key Addition:** OSD now displays "ROCKET PASSIVE" for NAVIGATION_STATE_ROCKET_PASSIVE

3. **`src/drivers/rc/crsf_rc/CrsfRc.cpp`**
   - **Changes:** Added Rocket Passive mode support to CRSF telemetry (4 lines added)
   - **Key Addition:** CRSF telemetry now reports "Rocket Passive" mode

4. **`src/drivers/rc_input/crsf_telemetry.cpp`**
   - **Changes:** Added Rocket Passive mode support to CRSF input (4 lines added)
   - **Key Addition:** CRSF input now handles NAVIGATION_STATE_ROCKET_PASSIVE

5. **`src/modules/CMakeLists.txt`**
   - **Changes:** Empty file cleaned up (no content changes)
   - **Purpose:** File maintenance

6. **`src/modules/navigator/navigator_main.cpp`**
   - **Changes:** Added Rocket Passive mode to navigation handling (1 line added)
   - **Key Addition:** Navigator now properly handles NAVIGATION_STATE_ROCKET_PASSIVE

7. **`src/modules/rocket_mode_manager/module.yaml`**
   - **Changes:** Parameter cleanup and optimization (5 lines removed)
   - **Purpose:** Removed redundant or unused parameter definitions

8. **`src/modules/rocket_mode_manager/rocket_mode_manager.cpp`**
   - **Changes:** Code cleanup and comment improvements (30 lines modified)
   - **Key Updates:** Enhanced code documentation and comment clarity

9. **`src/modules/rocket_mode_manager/rocket_mode_manager.hpp`**
   - **Changes:** Header file cleanup (2 lines changed)
   - **Purpose:** Code maintenance and comment updates

### Key Improvements:
- **Complete Rocket Passive Mode Support:** All telemetry drivers (OSD, CRSF, Navigator) now properly support Rocket Passive mode
- **Gazebo Configuration Update:** Simulation airframe synchronized with flight-tested production configuration
- **Code Cleanup:** Removed unused documentation files and empty CMake files
- **Enhanced Documentation:** Improved code comments and documentation clarity
---
