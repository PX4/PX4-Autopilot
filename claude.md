# PX4 RTCM Injection Pipeline - Development Notes

## Overview

This branch (`dev/ppk2`) implements Moving Baseline RTK between two ARK RTK F9P GPS modules over DroneCAN, with PPK (Post-Processing Kinematic) logging support.

### Hardware Architecture

The ARK RTK GPS modules are CAN nodes running PX4 uavcannode firmware. Each GPS module has its own STM32 MCU running PX4 that communicates with the F9P over serial, then bridges data to/from DroneCAN.

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          DroneCAN Bus (1Mbps)                           │
└─────────────────────────────────────────────────────────────────────────┘
        │                           │                           │
        ▼                           ▼                           ▼
┌───────────────────┐   ┌───────────────────┐   ┌───────────────────┐
│  ARK RTK GPS #1   │   │  ARK RTK GPS #2   │   │ Flight Controller │
│   (MovingBase)    │   │     (Rover)       │   │                   │
│    node 116       │   │     node 52       │   │                   │
├───────────────────┤   ├───────────────────┤   ├───────────────────┤
│ PX4 uavcannode    │   │ PX4 uavcannode    │   │ PX4 Autopilot     │
│ firmware          │   │ firmware          │   │                   │
│                   │   │                   │   │ - uavcan driver   │
│ - gps.cpp         │   │ - gps.cpp         │   │   (gnss.cpp)      │
│ - MBD Publisher   │   │ - MBD Subscriber  │   │                   │
├───────────────────┤   ├───────────────────┤   └───────────────────┘
│ u-blox F9P        │   │ u-blox F9P        │
│ (UART)            │   │ (UART)            │
└───────────────────┘   └───────────────────┘
```

### Firmware Variants

- **Flight Controller**: Full PX4 autopilot (`ark_fmu-v6x_default`)
- **ARK RTK GPS nodes**: PX4 uavcannode variant (`ark_can-rtk-gps_default`)
  - Runs `gps.cpp` driver talking to F9P over serial
  - Runs uavcannode Publishers/Subscribers for CAN communication

### Data Flow: RTCM from MovingBase to Rover

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    ARK RTK GPS #1 (MovingBase, node 116)                │
├─────────────────────────────────────────────────────────────────────────┤
│  F9P generates RTCM (MSM4/MSM7 observations)                            │
│      │                                                                  │
│      ▼                                                                  │
│  gps.cpp reads RTCM from F9P serial                                     │
│      │ publishRTCMCorrections() -> gps_inject_data uORB                 │
│      ▼                                                                  │
│  uavcannode/Publishers/MovingBaselineData.hpp                           │
│      │ Subscribes gps_inject_data, publishes to CAN                     │
│      ▼                                                                  │
│  ardupilot::gnss::MovingBaselineData CAN message                        │
└──────────────────────────────────┬──────────────────────────────────────┘
                               │
                         DroneCAN Bus
                               │
        ┌──────────────────────┴──────────────────────┐
        ▼                                             ▼
┌───────────────────────────────────┐  ┌─────────────────────────────────┐
│  Flight Controller                │  │ ARK RTK GPS #2 (Rover, node 52) │
├───────────────────────────────────┤  ├─────────────────────────────────┤
│  uavcan/sensors/gnss.cpp          │  │ uavcannode/Subscribers/         │
│    │ UAVCAN_SUB_MBD=1             │  │   MovingBaselineData.hpp        │
│    │ Logs MBD to gps_dump for PPK │  │     │ Receives MBD from CAN     │
│    ▼                              │  │     │ Publishes gps_inject_data │
│  gps_dump uORB -> SD card logging │  │     ▼                           │
└───────────────────────────────────┘  │ gps.cpp                         │
                                       │     │ drainRTCMFromORB()        │
                                       │     │ RTCM parser buffer        │
                                       │     │ getNextMessage()          │
                                       │     │ injectData()              │
                                       │     ▼                           │
                                       │ F9P UART TX (RTCM injection)    │
                                       │     │                           │
                                       │     ▼                           │
                                       │ F9P computes RTK fix            │
                                       └─────────────────────────────────┘
```

## Current Status: FIXED - Data Loss Resolved

### Root Cause Found & Fixed

The 42% data loss was caused by a **double-read bug** in `drainRTCMFromORB()`:

```cpp
// BUG: Both copy() and update() advance the generation counter!
while (...) {
    gps_inject_data_sub.copy(&msg);   // Reads msg N, advances gen
    // ... process msg N ...
    gps_inject_data_sub.update(&msg); // Reads msg N+1, advances gen again!
}
// Result: Every other message was skipped silently
```

**Fix:** Use only `update()` to read messages, not `copy()` then `update()`.

### Changes Made

1. **Fixed double-read bug** in `src/drivers/gps/gps.cpp::drainRTCMFromORB()`
2. **Increased uORB queue** from 16 to 32 in `msg/GpsInjectData.msg`
3. **Added publish failure tracking** in `MovingBaselineData.hpp`

## Key Files

### On CAN GPS Nodes (uavcannode firmware)
- `src/drivers/gps/gps.cpp` - GPS driver, RTCM parsing/injection
- `src/drivers/uavcannode/Subscribers/MovingBaselineData.hpp` - CAN→uORB (Rover receives RTCM)
- `src/drivers/uavcannode/Publishers/MovingBaselineData.hpp` - uORB→CAN (MovingBase sends RTCM)

### On Flight Controller
- `src/drivers/uavcan/sensors/gnss.cpp` - UAVCAN GPS bridge, MBD subscription for PPK logging

### Shared Libraries
- `src/lib/gnss/rtcm.h` / `rtcm.cpp` - RTCM3 frame parser

### Analysis Tools
- `Tools/rtcm_inject_analyzer.py` - ULog analyzer for RTCM pipeline stats

## Instrumentation (DEBUG_RTCM_INJECT)

Diagnostic instrumentation is controlled by `DEBUG_RTCM_INJECT` compile flag.

### To Enable

Add to the board's `board_config.h`:
```cpp
// In boards/ark/can-rtk-gps/src/board_config.h (or equivalent)
#define DEBUG_RTCM_INJECT
```

Then build normally:
```bash
make ark_can-rtk-gps_default
```

To disable, comment out or remove the `#define DEBUG_RTCM_INJECT` line.

### Instrumented Code Locations

**gps.cpp:**
- RTCM frame TX/RX/INJ counters and rates
- Latency tracking (uORB receive to serial TX)
- Diagnostic counters: `uorb`, `gaps`, `drop`, `tx_nr`, `tx_blk`
- CPU load monitoring
- TX injection timing stats

**MovingBaselineData.hpp (Subscriber):**
- RTCM frame counting via parser
- MBD message stats
- Publish failure counter

**MovingBaselineData.hpp (Publisher):**
- RTCM frame counting
- MBD TX stats

**gnss.cpp (FC):**
- RTCM LOG frame counting for PPK

### Log Output Format (when enabled)

```
# MovingBase (node 116)
[uavcan:116:gps] RTCM TX: 211 frames (42.2/s), 48251 B (9650 B/s)
[uavcan:116:uavcannode] MBD TX: 372 msgs (74.4/s), 48251 bytes

# Flight Controller
[uavcan] RTCM LOG: 211 frames (42.2/s), 48251 B (9650 B/s)

# Rover (node 52)
[uavcan:52:uavcannode] RTCM RX: 211 frames (42.2/s), 48251 B (9650 B/s)
[uavcan:52:gps] RTCM INJ: 211 frames (42.2/s), 48251 B (9650 B/s)
[uavcan:52:gps] RTCM diag: uorb=250 gaps=0 drop=0 tx_nr=0 tx_blk=0
[uavcan:52:gps] RTCM latency ms: min=2.1 avg=5.3 max=47.2
```

## Build & Test

```bash
# Build for Flight Controller
make ark_fmu-v6x_default

# Build for ARK RTK GPS nodes
make ark_can-rtk-gps_default

# Analyze a log (from FC SD card)
python3 Tools/rtcm_inject_analyzer.py <log_file> --plot analysis.png
```

## Log Message Source Identification

In ULog `logged_messages`, the source is identified in brackets:
- `[uavcan:52:gps]` - GPS driver on node 52 (Rover)
- `[uavcan:116:gps]` - GPS driver on node 116 (MovingBase)
- `[uavcan:52:uavcannode]` - uavcannode subscriber on node 52 (Rover MBD RX)
- `[uavcan:116:uavcannode]` - uavcannode publisher on node 116 (MovingBase MBD TX)
- `[uavcan]` - FC's uavcan driver (MBD RX for PPK logging)

## Relevant Parameters

### On Flight Controller
- `UAVCAN_SUB_MBD` - Enable MBD subscription for PPK logging

### On CAN GPS Nodes
- `UAVCAN_PUB_MBD` - Enable MBD publishing for moving baseline
- `GPS_UBX_MODE` - UBX mode (1=heading/moving base, 2=moving base only)
