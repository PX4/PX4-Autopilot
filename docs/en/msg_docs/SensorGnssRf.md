---
pageClass: is-wide-page
---

# SensorGnssRf (UORB message)

GNSS RF status.

Reports RF block status for a GNSS receiver, decoded from the u-blox UBX-MON-RF message.
Includes antenna status/power, jamming/interference indicators, automatic gain control (AGC),
noise level, and I/Q demodulation metrics. Published by the `gps` driver (UBX protocol only), once
per RF block, via the block-specific topics below.

**TOPICS:** sensor_gnss_rf_block0 sensor_gnss_rf_block1 sensor_gnss_rf_block2

## Fields

| Name                                                          | Type     | Unit [Frame] | Range/Enum                        | Description                                                                         |
| ------------------------------------------------------------- | -------- | ------------ | --------------------------------- | ----------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                           | `uint64` | us           |                                   | Time since system start                                                             |
| <a id="fld_timestamp_sample"></a>timestamp_sample             | `uint64` | us           |                                   | Timestamp of the raw data                                                           |
| <a id="fld_device_id"></a>device_id                           | `uint32` |              |                                   | Unique device ID for the sensor that does not change between power cycles           |
| <a id="fld_block_id"></a>block_id                             | `uint8`  |              |                                   | Only used for multi-topic publishing                                                |
| <a id="fld_antenna_status"></a>antenna_status                 | `uint8`  |              | [ANTENNA_STATUS](#ANTENNA_STATUS) | Status of the antenna supervisor state machine                                      |
| <a id="fld_antenna_power"></a>antenna_power                   | `uint8`  |              | [ANTENNA_POWER](#ANTENNA_POWER)   | Antenna power state                                                                 |
| <a id="fld_post_status"></a>post_status                       | `uint32` |              |                                   | Power-on self-test (POST) status word (vendor-specific bitmask)                     |
| <a id="fld_noise_per_ms"></a>noise_per_ms                     | `uint16` |              |                                   | Noise level as measured by the GNSS receiver core                                   |
| <a id="fld_automatic_gain_control"></a>automatic_gain_control | `uint16` |              | [0 : 8191]                        | Automatic Gain Control (AGC) monitor value (8191 = 100% of max gain)                |
| <a id="fld_jamming_indicator"></a>jamming_indicator           | `uint8`  |              | [0 : 255]                         | Continuous wave (CW) jamming indicator (0 = no CW jamming, 255 = strong CW jamming) |
| <a id="fld_jamming_state"></a>jamming_state                   | `uint8`  |              | [JAMMING_STATE](#JAMMING_STATE)   | Jamming state                                                                       |
| <a id="fld_i_offset"></a>i_offset                             | `int8`   |              | [-128 : 127]                      | Imbalance of the I-part of the complex signal (0 = no imbalance)                    |
| <a id="fld_i_magnitude"></a>i_magnitude                       | `uint8`  |              | [0 : 255]                         | Magnitude of the I-part of the complex signal (0 = no signal, 255 = max magnitude)  |
| <a id="fld_q_offset"></a>q_offset                             | `int8`   |              | [-128 : 127]                      | Imbalance of the Q-part of the complex signal (0 = no imbalance)                    |
| <a id="fld_q_magnitude"></a>q_magnitude                       | `uint8`  |              | [0 : 255]                         | Magnitude of the Q-part of the complex signal (0 = no signal, 255 = max magnitude)  |
| <a id="fld_center_frequency"></a>center_frequency             | `uint32` | Hz           |                                   | Nominal center frequency of the RF block. 0 = unknown.                              |

## Enums

### ANTENNA_STATUS {#ANTENNA_STATUS}

Used in field(s): [antenna_status](#fld_antenna_status)

| Name                                                        | Type    | Value | Description            |
| ----------------------------------------------------------- | ------- | ----- | ---------------------- |
| <a id="#ANTENNA_STATUS_INIT"></a> ANTENNA_STATUS_INIT       | `uint8` | 0     | Initializing           |
| <a id="#ANTENNA_STATUS_UNKNOWN"></a> ANTENNA_STATUS_UNKNOWN | `uint8` | 1     | Status unknown         |
| <a id="#ANTENNA_STATUS_OK"></a> ANTENNA_STATUS_OK           | `uint8` | 2     | Antenna OK             |
| <a id="#ANTENNA_STATUS_SHORT"></a> ANTENNA_STATUS_SHORT     | `uint8` | 3     | Short circuit detected |
| <a id="#ANTENNA_STATUS_OPEN"></a> ANTENNA_STATUS_OPEN       | `uint8` | 4     | Open circuit detected  |

### ANTENNA_POWER {#ANTENNA_POWER}

Used in field(s): [antenna_power](#fld_antenna_power)

| Name                                                      | Type    | Value | Description         |
| --------------------------------------------------------- | ------- | ----- | ------------------- |
| <a id="#ANTENNA_POWER_OFF"></a> ANTENNA_POWER_OFF         | `uint8` | 0     | Power off           |
| <a id="#ANTENNA_POWER_ON"></a> ANTENNA_POWER_ON           | `uint8` | 1     | Power on            |
| <a id="#ANTENNA_POWER_UNKNOWN"></a> ANTENNA_POWER_UNKNOWN | `uint8` | 2     | Power state unknown |

### JAMMING_STATE {#JAMMING_STATE}

Used in field(s): [jamming_state](#fld_jamming_state)

| Name                                                        | Type    | Value | Description                     |
| ----------------------------------------------------------- | ------- | ----- | ------------------------------- |
| <a id="#JAMMING_STATE_UNKNOWN"></a> JAMMING_STATE_UNKNOWN   | `uint8` | 0     | Unknown                         |
| <a id="#JAMMING_STATE_OK"></a> JAMMING_STATE_OK             | `uint8` | 1     | No jamming detected             |
| <a id="#JAMMING_STATE_WARNING"></a> JAMMING_STATE_WARNING   | `uint8` | 2     | Interference visible but fix OK |
| <a id="#JAMMING_STATE_CRITICAL"></a> JAMMING_STATE_CRITICAL | `uint8` | 3     | Interference visible and no fix |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/SensorGnssRf.msg)

::: details Click here to see original file

```c
# GNSS RF status
#
# Reports RF block status for a GNSS receiver, decoded from the u-blox UBX-MON-RF message.
# Includes antenna status/power, jamming/interference indicators, automatic gain control (AGC),
# noise level, and I/Q demodulation metrics. Published by the `gps` driver (UBX protocol only), once
# per RF block, via the block-specific topics below.

uint64 timestamp # [us] Time since system start
uint64 timestamp_sample # [us] Timestamp of the raw data

uint32 device_id # Unique device ID for the sensor that does not change between power cycles

uint8 block_id  # [-] Only used for multi-topic publishing

uint8 antenna_status # [@enum ANTENNA_STATUS] Status of the antenna supervisor state machine
uint8 ANTENNA_STATUS_INIT = 0 # Initializing
uint8 ANTENNA_STATUS_UNKNOWN = 1 # Status unknown
uint8 ANTENNA_STATUS_OK = 2 # Antenna OK
uint8 ANTENNA_STATUS_SHORT = 3 # Short circuit detected
uint8 ANTENNA_STATUS_OPEN = 4 # Open circuit detected

uint8 antenna_power # [@enum ANTENNA_POWER] Antenna power state
uint8 ANTENNA_POWER_OFF = 0 # Power off
uint8 ANTENNA_POWER_ON = 1 # Power on
uint8 ANTENNA_POWER_UNKNOWN = 2 # Power state unknown

uint32 post_status # [-] Power-on self-test (POST) status word (vendor-specific bitmask)

uint16 noise_per_ms # [-] Noise level as measured by the GNSS receiver core
uint16 automatic_gain_control # [-] [@range 0, 8191] Automatic Gain Control (AGC) monitor value (8191 = 100% of max gain)
uint8 jamming_indicator # [-] [@range 0, 255] Continuous wave (CW) jamming indicator (0 = no CW jamming, 255 = strong CW jamming)

uint8 jamming_state # [@enum JAMMING_STATE] Jamming state
uint8 JAMMING_STATE_UNKNOWN = 0 # Unknown
uint8 JAMMING_STATE_OK = 1 # No jamming detected
uint8 JAMMING_STATE_WARNING = 2 # Interference visible but fix OK
uint8 JAMMING_STATE_CRITICAL = 3 # Interference visible and no fix

int8 i_offset # [-] [@range -128, 127] Imbalance of the I-part of the complex signal (0 = no imbalance)
uint8 i_magnitude # [-] [@range 0, 255] Magnitude of the I-part of the complex signal (0 = no signal, 255 = max magnitude)
int8 q_offset # [-] [@range -128, 127] Imbalance of the Q-part of the complex signal (0 = no imbalance)
uint8 q_magnitude # [-] [@range 0, 255] Magnitude of the Q-part of the complex signal (0 = no signal, 255 = max magnitude)

uint32 center_frequency # [Hz] Nominal center frequency of the RF block. 0 = unknown.

# One topic per RF block ID
# TOPICS sensor_gnss_rf_block0 sensor_gnss_rf_block1 sensor_gnss_rf_block2
```

:::
