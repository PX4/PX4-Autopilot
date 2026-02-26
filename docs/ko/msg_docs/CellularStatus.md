---
pageClass: is-wide-page
---

# CellularStatus (UORB message)

Cellular status.

This is currently used only for logging cell status from MAVLink.

**TOPICS:** cellular_status

## Fields

| 명칭                                  | 형식       | Unit [Frame] | Range/Enum                                                                                                                 | 설명                                                                                                |
| ----------------------------------- | -------- | ---------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------- |
| timestamp                           | `uint64` | us                                                               |                                                                                                                            | Time since system start                                                                           |
| status                              | `uint16` |                                                                  | [STATUS_FLAG](#STATUS_FLAG)                                                                           | Status bitmap                                                                                     |
| failure_reason | `uint8`  |                                                                  | [FAILURE_REASON](#FAILURE_REASON)                                                                     | Failure reason                                                                                    |
| type                                | `uint8`  |                                                                  | [CELLULAR_NETWORK_RADIO_TYPE](#CELLULAR_NETWORK_RADIO_TYPE) | Cellular network radio type                                                                       |
| quality                             | `uint8`  | dBm                                                              |                                                                                                                            | Cellular network RSSI/RSRP, absolute value                                                        |
| mcc                                 | `uint16` |                                                                  |                                                                                                                            | Mobile country code (Invalid: UINT16_MAX) |
| mnc                                 | `uint16` |                                                                  |                                                                                                                            | Mobile network code (Invalid: UINT16_MAX) |
| lac                                 | `uint16` |                                                                  |                                                                                                                            | Location area code (Invalid: 0)                                |

## Enums

### STATUS_FLAG {#STATUS_FLAG}

| 명칭                                                                                                            | 형식       | Value | 설명                                                                                                                                                                                                                    |
| ------------------------------------------------------------------------------------------------------------- | -------- | ----- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a href="#STATUS_FLAG_UNKNOWN"></a> STATUS_FLAG_UNKNOWN             | `uint16` | 1     | State unknown or not reportable                                                                                                                                                                                       |
| <a href="#STATUS_FLAG_FAILED"></a> STATUS_FLAG_FAILED               | `uint16` | 2     | Modem is unusable                                                                                                                                                                                                     |
| <a href="#STATUS_FLAG_INITIALIZING"></a> STATUS_FLAG_INITIALIZING   | `uint16` | 4     | Modem is being initialized                                                                                                                                                                                            |
| <a href="#STATUS_FLAG_LOCKED"></a> STATUS_FLAG_LOCKED               | `uint16` | 8     | Modem is locked                                                                                                                                                                                                       |
| <a href="#STATUS_FLAG_DISABLED"></a> STATUS_FLAG_DISABLED           | `uint16` | 16    | Modem is not enabled and is powered down                                                                                                                                                                              |
| <a href="#STATUS_FLAG_DISABLING"></a> STATUS_FLAG_DISABLING         | `uint16` | 32    | Modem is currently transitioning to the STATUS_FLAG_DISABLED state                                                                                                          |
| <a href="#STATUS_FLAG_ENABLING"></a> STATUS_FLAG_ENABLING           | `uint16` | 64    | Modem is currently transitioning to the STATUS_FLAG_ENABLED state                                                                                                           |
| <a href="#STATUS_FLAG_ENABLED"></a> STATUS_FLAG_ENABLED             | `uint16` | 128   | Modem is enabled and powered on but not registered with a network provider and not available for data connections                                                                                                     |
| <a href="#STATUS_FLAG_SEARCHING"></a> STATUS_FLAG_SEARCHING         | `uint16` | 256   | Modem is searching for a network provider to register                                                                                                                                                                 |
| <a href="#STATUS_FLAG_REGISTERED"></a> STATUS_FLAG_REGISTERED       | `uint16` | 512   | Modem is registered with a network provider, and data connections and messaging may be available for use                                                                                                              |
| <a href="#STATUS_FLAG_DISCONNECTING"></a> STATUS_FLAG_DISCONNECTING | `uint16` | 1024  | Modem is disconnecting and deactivating the last active packet data bearer. This state will not be entered if more than one packet data bearer is active and one of the active bearers is deactivated |
| <a href="#STATUS_FLAG_CONNECTING"></a> STATUS_FLAG_CONNECTING       | `uint16` | 2048  | Modem is activating and connecting the first packet data bearer. Subsequent bearer activations when another bearer is already active do not cause this state to be entered                            |
| <a href="#STATUS_FLAG_CONNECTED"></a> STATUS_FLAG_CONNECTED         | `uint16` | 4096  | One or more packet data bearers is active and connected                                                                                                                                                               |

### FAILURE_REASON {#FAILURE_REASON}

| 명칭                                                                                                                                   | 형식      | Value | 설명                                              |
| ------------------------------------------------------------------------------------------------------------------------------------ | ------- | ----- | ----------------------------------------------- |
| <a href="#FAILURE_REASON_NONE"></a> FAILURE_REASON_NONE                                    | `uint8` | 0     | No error                                        |
| <a href="#FAILURE_REASON_UNKNOWN"></a> FAILURE_REASON_UNKNOWN                              | `uint8` | 1     | Error state is unknown                          |
| <a href="#FAILURE_REASON_SIM_MISSING"></a> FAILURE_REASON_SIM_MISSING | `uint8` | 2     | SIM is required for the modem but missing       |
| <a href="#FAILURE_REASON_SIM_ERROR"></a> FAILURE_REASON_SIM_ERROR     | `uint8` | 3     | SIM is available, but not usable for connection |

### CELLULAR_NETWORK_RADIO_TYPE {#CELLULAR_NETWORK_RADIO_TYPE}

| 명칭                                                                                                                                                                      | 형식      | Value | 설명    |
| ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------- | ----- | ----- |
| <a href="#CELLULAR_NETWORK_RADIO_TYPE_NONE"></a> CELLULAR_NETWORK_RADIO_TYPE_NONE   | `uint8` | 0     | None  |
| <a href="#CELLULAR_NETWORK_RADIO_TYPE_GSM"></a> CELLULAR_NETWORK_RADIO_TYPE_GSM     | `uint8` | 1     | GSM   |
| <a href="#CELLULAR_NETWORK_RADIO_TYPE_CDMA"></a> CELLULAR_NETWORK_RADIO_TYPE_CDMA   | `uint8` | 2     | CDMA  |
| <a href="#CELLULAR_NETWORK_RADIO_TYPE_WCDMA"></a> CELLULAR_NETWORK_RADIO_TYPE_WCDMA | `uint8` | 3     | WCDMA |
| <a href="#CELLULAR_NETWORK_RADIO_TYPE_LTE"></a> CELLULAR_NETWORK_RADIO_TYPE_LTE     | `uint8` | 4     | LTE   |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/CellularStatus.msg)

:::details
Click here to see original file

```c
# Cellular status
#
# This is currently used only for logging cell status from MAVLink.

uint64 timestamp  # [us] Time since system start

uint16 status                            # [@enum STATUS_FLAG] Status bitmap
uint16 STATUS_FLAG_UNKNOWN = 1           # State unknown or not reportable
uint16 STATUS_FLAG_FAILED = 2            # Modem is unusable
uint16 STATUS_FLAG_INITIALIZING = 4      # Modem is being initialized
uint16 STATUS_FLAG_LOCKED = 8            # Modem is locked
uint16 STATUS_FLAG_DISABLED = 16         # Modem is not enabled and is powered down
uint16 STATUS_FLAG_DISABLING = 32        # Modem is currently transitioning to the STATUS_FLAG_DISABLED state
uint16 STATUS_FLAG_ENABLING = 64         # Modem is currently transitioning to the STATUS_FLAG_ENABLED state
uint16 STATUS_FLAG_ENABLED = 128         # Modem is enabled and powered on but not registered with a network provider and not available for data connections
uint16 STATUS_FLAG_SEARCHING = 256       # Modem is searching for a network provider to register
uint16 STATUS_FLAG_REGISTERED = 512      # Modem is registered with a network provider, and data connections and messaging may be available for use
uint16 STATUS_FLAG_DISCONNECTING = 1024  # Modem is disconnecting and deactivating the last active packet data bearer. This state will not be entered if more than one packet data bearer is active and one of the active bearers is deactivated
uint16 STATUS_FLAG_CONNECTING = 2048     # Modem is activating and connecting the first packet data bearer. Subsequent bearer activations when another bearer is already active do not cause this state to be entered
uint16 STATUS_FLAG_CONNECTED = 4096      # One or more packet data bearers is active and connected

uint8 failure_reason                  # [@enum FAILURE_REASON] Failure reason
uint8 FAILURE_REASON_NONE = 0         # No error
uint8 FAILURE_REASON_UNKNOWN = 1      # Error state is unknown
uint8 FAILURE_REASON_SIM_MISSING = 2  # SIM is required for the modem but missing
uint8 FAILURE_REASON_SIM_ERROR = 3    # SIM is available, but not usable for connection

uint8 type                                   # [@enum CELLULAR_NETWORK_RADIO_TYPE] Cellular network radio type
uint8 CELLULAR_NETWORK_RADIO_TYPE_NONE = 0   # None
uint8 CELLULAR_NETWORK_RADIO_TYPE_GSM = 1    # GSM
uint8 CELLULAR_NETWORK_RADIO_TYPE_CDMA = 2   # CDMA
uint8 CELLULAR_NETWORK_RADIO_TYPE_WCDMA = 3  # WCDMA
uint8 CELLULAR_NETWORK_RADIO_TYPE_LTE = 4    # LTE

uint8 quality  # [dBm] Cellular network RSSI/RSRP, absolute value
uint16 mcc     # [@invalid UINT16_MAX] Mobile country code
uint16 mnc     # [@invalid UINT16_MAX] Mobile network code
uint16 lac     # [@invalid 0] Location area code
```

:::
