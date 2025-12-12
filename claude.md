# PX4 RTCM Injection Pipeline - Moving Baseline RTK & PPK Support

## Overview

This branch (`dev/ppk2`) implements Moving Baseline RTK between two u-blox F9P GPS modules with support for both DroneCAN and serial connections. It also adds PPK (Post-Processing Kinematic) logging capability by capturing RTCM data to the SD card.

**Key capabilities:**
- Moving Baseline RTK for precise heading (dual GPS)
- Support for MSM4 (RTK) and MSM7 (PPK) RTCM message types
- Multiple hardware configurations (dual CAN, dual serial, mixed)
- RTCM data logging to SD card for PPK post-processing

---

## Hardware Architecture

### Dual CAN GPS Configuration

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
│    node_id=116    │   │    node_id=52     │   │                   │
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

### Serial GPS Configuration

For GPS modules connected directly to the flight controller via serial:

```
┌────────────────────────────────────────────────────────────────────────────┐
│                           Flight Controller                                 │
├────────────────────────────────────────────────────────────────────────────┤
│                              PX4 Autopilot                                  │
│                                                                            │
│  ┌─────────────────┐                      ┌─────────────────┐              │
│  │   gps (main)    │                      │   gps (aux)     │              │
│  │   MovingBase    │                      │     Rover       │              │
│  │                 │   gps_inject_data    │                 │              │
│  │  RTCM TX ──────────────────────────────── RTCM RX       │              │
│  │  (uORB pub)     │      (uORB)          │  (uORB sub)     │              │
│  └────────┬────────┘                      └────────┬────────┘              │
│           │ UART                                   │ UART                  │
└───────────┼────────────────────────────────────────┼───────────────────────┘
            │                                        │
     ┌──────▼──────┐                          ┌──────▼──────┐
     │  u-blox F9P │                          │  u-blox F9P │
     │ (GPS1/UART) │                          │ (GPS2/UART) │
     └─────────────┘                          └─────────────┘
```

### Mixed CAN + Serial Configuration

One GPS on CAN, one on serial (supports both MovingBase and Rover on either interface).

---

## Data Flow: RTCM from MovingBase to Rover (Dual CAN)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    ARK RTK GPS #1 (MovingBase, node 116)                │
├─────────────────────────────────────────────────────────────────────────┤
│  F9P generates RTCM (MSM4/MSM7 observations + 4072 proprietary)        │
│      │                                                                  │
│      ▼                                                                  │
│  gps.cpp reads RTCM from F9P serial                                     │
│      │ publishRTCMCorrections() -> gps_inject_data uORB                 │
│      ▼                                                                  │
│  uavcannode/Publishers/MovingBaselineData.hpp                           │
│      │ Subscribes gps_inject_data, publishes to CAN                     │
│      ▼                                                                  │
│  ardupilot::gnss::MovingBaselineData CAN message                        │
└──────────────────────────────┬──────────────────────────────────────────┘
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
│    │ Receives MBD, writes to      │  │     │ Receives MBD from CAN     │
│    │ gps_dump for PPK logging     │  │     │ Publishes gps_inject_data │
│    ▼                              │  │     ▼                           │
│  gps_dump uORB -> SD card logging │  │ gps.cpp                         │
└───────────────────────────────────┘  │     │ handleInjectDataTopic()   │
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

---

## Implementation Summary

### New Components

| Component | Path | Description |
|-----------|------|-------------|
| RTCM3 Parser Library | `src/lib/gnss/rtcm.h`, `rtcm.cpp` | Frame-aligned RTCM3 parser with CRC-24Q validation |
| `GPS_UBX_PPK` parameter | `src/drivers/gps/params.c` | Enable MSM7 output for PPK workflow |
| `UAVCAN_SUB_MBD` parameter | `src/drivers/uavcan/uavcan_params.c` | Enable MBD subscription on FC for PPK logging |

### Modified Components

| Component | Changes |
|-----------|---------|
| `src/drivers/gps/gps.cpp` | Frame-aligned injection via RTCM parser, txSpaceAvailable() check, DEBUG_RTCM_INJECT instrumentation |
| `src/drivers/uavcannode/Publishers/MovingBaselineData.hpp` | uORB generation tracking, drain loop, DEBUG stats |
| `src/drivers/uavcannode/Subscribers/MovingBaselineData.hpp` | RTCM frame counting, DEBUG stats |
| `src/drivers/uavcan/sensors/gnss.cpp` | MBD subscription, writes to gps_dump for PPK, generation tracking |
| `platforms/*/Serial*.cpp` | Added `txSpaceAvailable()` and `poll()` methods |
| `msg/GpsDump.msg` | Changed `instance` to `device_id`, queue 8→16 |

---

## Key Files Reference

### On CAN GPS Nodes (uavcannode firmware)

- `src/drivers/gps/gps.cpp` - GPS driver with RTCM parsing/injection
- `src/drivers/uavcannode/Subscribers/MovingBaselineData.hpp` - CAN→uORB (Rover receives RTCM)
- `src/drivers/uavcannode/Publishers/MovingBaselineData.hpp` - uORB→CAN (MovingBase sends RTCM)

### On Flight Controller

- `src/drivers/uavcan/sensors/gnss.cpp` - UAVCAN GPS bridge, MBD subscription for PPK logging
- `src/drivers/gps/gps.cpp` - GPS driver (for serial-connected GPS)

### Shared Libraries

- `src/lib/gnss/rtcm.h` / `rtcm.cpp` - RTCM3 frame parser with CRC validation

---

## Build & Flash

```bash
# Flight Controller (ARK FMU v6X)
make ark_fmu-v6x_default
# Flash via bootloader or DFU

# ARK RTK GPS CAN nodes
make ark_can-rtk-gps_default
# Flash via CAN bootloader
```

---

## Parameters

### GPS Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `GPS_UBX_MODE` | UBX mode: 0=normal, 1=heading/moving base, 2=moving base only | 0 |
| `GPS_UBX_PPK` | Enable MSM7 RTCM output for PPK workflow | 0 |

### UAVCAN Parameters (Flight Controller)

| Parameter | Description | Default |
|-----------|-------------|---------|
| `UAVCAN_SUB_MBD` | Subscribe to MovingBaselineData for PPK logging | 0 |

### UAVCAN Parameters (CAN GPS Nodes)

| Parameter | Description | Default |
|-----------|-------------|---------|
| `UAVCAN_PUB_MBD` | Publish MovingBaselineData (enable on MovingBase) | 0 |

---

## Debug Instrumentation

Diagnostic instrumentation is controlled by the `DEBUG_RTCM_INJECT` compile flag.

### To Enable

Add to the board's `board_config.h`:

```cpp
// boards/ark/can-rtk-gps/src/board_config.h
#define DEBUG_RTCM_INJECT

// boards/ark/fmu-v6x/src/board_config.h (for serial GPS debugging)
#define DEBUG_RTCM_INJECT
```

The flag is already present but commented out in both board configs.

### Instrumented Metrics

**gps.cpp (on Rover):**
- RTCM INJ: frames injected to F9P serial
- TX interval timing (min/avg/max)
- Latency: uORB receive to serial TX
- Parser stats: CRC errors, bytes discarded

**MovingBaselineData.hpp (Publisher, on MovingBase):**
- MBD TX: messages sent to CAN, bytes

**MovingBaselineData.hpp (Subscriber, on Rover):**
- RTCM RX: frames received via CAN
- MBD RX: CAN messages received

**gnss.cpp (on FC):**
- RTCM LOG: frames logged for PPK

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
[uavcan:52:gps] RTCM latency ms: min=2.1 avg=5.3 max=47.2
```

---

## Analysis Tools

Python scripts for analyzing RTCM data from ULog files:

### gps_dump_analyzer.py

Parses `gps_dump` uORB topic from ULog, validates RTCM message CRCs, and provides statistics.

```bash
# Basic analysis with console output
python3 Tools/gps_dump_analyzer.py flight.ulg

# Save plots to file
python3 Tools/gps_dump_analyzer.py flight.ulg --plot analysis.png

# Console only (no plot)
python3 Tools/gps_dump_analyzer.py flight.ulg --no-plot
```

**Output:**
- Message counts by type (MSM4, MSM7, 4072, etc.)
- Message rates
- CRC validation results
- Timing statistics

### rtcm_burst_analyzer.py

Analyzes RTCM message timing to understand burst patterns from the moving base.

```bash
# Analyze with default 50ms gap threshold
python3 Tools/rtcm_burst_analyzer.py flight.ulg

# Custom gap threshold
python3 Tools/rtcm_burst_analyzer.py flight.ulg --gap 100
```

**Output:**
- Burst detection and statistics
- Inter-message intervals
- Burst composition (which MSM types appear together)
- Timing jitter analysis

### rtcm_inject_analyzer.py

Parses `DEBUG_RTCM_INJECT` instrumentation from `logged_messages` in ULog.

```bash
python3 Tools/rtcm_inject_analyzer.py flight.ulg
```

**Note:** Requires `DEBUG_RTCM_INJECT` to be enabled during flight.

**Output:**
- Pipeline stage comparison (TX → LOG → RX → INJ)
- Frame loss detection
- Latency statistics
- MBD message rates

### rtcm_test_report.py

Generates a comprehensive test report with analysis and plots.

```bash
python3 Tools/rtcm_test_report.py flight.ulg --name dual_can_msm4 --config "Dual CAN GPS, MSM4"
```

**Output:** Creates `reports/<name>/` directory with:
- `report.md` - Markdown report
- `gps_dump_analysis.png` - RTCM message plots
- `burst_analysis.png` - Burst timing plots

---

## Test Matrix

| Configuration | MovingBase | Rover | RTCM Type | Status |
|--------------|------------|-------|-----------|--------|
| Dual CAN GPS | CAN | CAN | MSM4 | ✅ Tested |
| Dual CAN GPS | CAN | CAN | MSM7 | ⏳ Pending |
| Serial + CAN | Serial (GPS1) | CAN | MSM4 | ⏳ Pending |
| Serial + CAN | CAN | Serial (GPS2) | MSM4 | ⏳ Pending |
| Dual Serial | Serial (GPS1) | Serial (GPS2) | MSM4 | ⏳ Pending |

---

## Log Message Source Identification

In ULog `logged_messages`, the source is identified in brackets:

| Pattern | Source |
|---------|--------|
| `[uavcan:52:gps]` | GPS driver on CAN node 52 (Rover) |
| `[uavcan:116:gps]` | GPS driver on CAN node 116 (MovingBase) |
| `[uavcan:52:uavcannode]` | uavcannode subscriber on node 52 (Rover MBD RX) |
| `[uavcan:116:uavcannode]` | uavcannode publisher on node 116 (MovingBase MBD TX) |
| `[uavcan]` | FC's uavcan driver (MBD RX for PPK logging) |
