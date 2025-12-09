# PPK RTCM Data Logging Implementation Summary

## Overview

This implementation enables Post-Processing Kinematic (PPK) workflows by configuring the u-blox F9P GNSS receiver to output high-precision MSM7 RTCM messages and logging them through the PX4 system for later processing.

## Data Flow

```
F9P Receiver --> GPS Driver --> gps_inject_data (uORB)
                                       |
                                       v
                      CANnode MovingBaselineDataPub
                                       |
                                       v (UAVCAN)
                        FC uavcan gnss subscriber
                                       |
                                       v
                        gps_dump (uORB) --> Logger --> ULog file
```

## Key Changes

### 1. GPS Devices Submodule (`src/drivers/gps/devices`)

**Commit:** `c7175852c3f` - "ubx: add PPK output option and enable MSM7 message output"

- Added `Settings` struct to consolidate driver configuration parameters
- Added `ppk_output` member to enable MSM7 message output
- Modified `configureDevice()` to configure MSM7 messages (1077, 1087, 1097, 1127, 1230) at 5Hz when PPK is enabled
- In Normal mode with PPK: configures I2C output for MSM7 messages
- In MovingBase mode with PPK: configures UART1/UART2 output for MSM7 messages
- Without PPK: continues using MSM4 messages (1074, 1084, 1094, 1124) at 1Hz

### 2. GPS Driver (`src/drivers/gps/params.c`)

- Added `GPS_UBX_PPK` parameter (boolean) to enable MSM7 message output for PPK workflow

### 3. UAVCAN GNSS Bridge (`src/drivers/uavcan/sensors/gnss.cpp`)

- Added `UAVCAN_SUB_MBD` parameter handling to subscribe to MovingBaselineData
- Added `moving_baseline_data_sub_cb()` callback that:
  - Receives RTCM data from UAVCAN MovingBaselineData messages
  - Chunks data into 79-byte segments
  - Publishes to `gps_dump` uORB topic for logging
- Added `_gps_dump_pub` publisher for PPK data logging

### 4. CANnode Publisher (`src/drivers/uavcannode/Publishers/MovingBaselineData.hpp`)

- Already existed for RTK base station functionality
- Subscribes to `gps_inject_data` uORB topic
- Publishes as UAVCAN `ardupilot::gnss::MovingBaselineData` messages
- Handles message chunking for UAVCAN capacity limits
- Includes queue overflow detection and warning

## MSM7 Messages Configured

| Message | Constellation | Rate |
|---------|---------------|------|
| 1077 | GPS MSM7 | 5Hz |
| 1087 | GLONASS MSM7 | 5Hz |
| 1097 | Galileo MSM7 | 5Hz |
| 1127 | BeiDou MSM7 | 5Hz |
| 1230 | GLONASS code-phase biases | 5Hz |

## GpsDump Message Format

| Field | Type | Description |
|-------|------|-------------|
| timestamp | uint64 | Time since system start (microseconds) |
| instance | uint8 | GNSS receiver instance |
| len | uint8 | Data length (MSB = direction flag) |
| data | uint8[79] | Raw RTCM bytes |

## Purpose

The logged RTCM data can be post-processed with base station correction data to achieve centimeter-level positioning accuracy, enabling PPK workflows for surveying and mapping applications.
