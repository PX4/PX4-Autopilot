---
pageClass: is-wide-page
---

# CellularStatus (UORB message)

Cellular status.

This is currently used only for logging cell status from MAVLink 1:1.

**TOPICS:** cellular_status

## Fields

| 参数名                                                                                   | 类型        | Unit [Frame] | Range/Enum                                                                                                                 | 描述                                                                                                                                                                                                                                                                                                                                                            |
| ------------------------------------------------------------------------------------- | --------- | ---------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="fld_timestamp"></a>timestamp                                                   | `uint64`  | us                                                               |                                                                                                                            | Time since system start                                                                                                                                                                                                                                                                                                                                       |
| <a id="fld_status"></a>status                                                         | `uint8`   |                                                                  | [STATUS_FLAG](#STATUS_FLAG)                                                                           | Status                                                                                                                                                                                                                                                                                                                                                        |
| <a id="fld_failure_reason"></a>failure_reason                    | `uint8`   |                                                                  | [FAILURE_REASON](#FAILURE_REASON)                                                                     | Failure reason                                                                                                                                                                                                                                                                                                                                                |
| <a id="fld_type"></a>type                                                             | `uint8`   |                                                                  | [CELLULAR_NETWORK_RADIO_TYPE](#CELLULAR_NETWORK_RADIO_TYPE) | Cellular network radio type                                                                                                                                                                                                                                                                                                                                   |
| <a id="fld_quality"></a>quality                                                       | `uint8`   | %                                                                |                                                                                                                            | Cellular network signal quality in percent. May be used for RSSI (Invalid: UINT8_MAX)                                                                                                                                                                                                 |
| <a id="fld_mcc"></a>mcc                                                               | `uint16`  |                                                                  |                                                                                                                            | Mobile country code (Invalid: UINT16_MAX)                                                                                                                                                                                                                                                             |
| <a id="fld_mnc"></a>mnc                                                               | `uint16`  |                                                                  |                                                                                                                            | Mobile network code (Invalid: UINT16_MAX)                                                                                                                                                                                                                                                             |
| <a id="fld_lac"></a>lac                                                               | `uint16`  |                                                                  |                                                                                                                            | Location area code (Invalid: 0)                                                                                                                                                                                                                                                                                            |
| <a id="fld_id"></a>id                                                                 | `uint8`   |                                                                  |                                                                                                                            | Cellular instance number. Indexed from 1. A value of 0 indicates the sender does not support reporting of multiple modems                                                                                                                                                                                                     |
| <a id="fld_link_tx_rate"></a>link_tx_rate   | `uint32`  | KiB/s                                                            |                                                                                                                            | Download rate (Invalid: 0)                                                                                                                                                                                                                                                                                                 |
| <a id="fld_link_rx_rate"></a>link_rx_rate   | `uint32`  | KiB/s                                                            |                                                                                                                            | Upload rate (Invalid: 0)                                                                                                                                                                                                                                                                                                   |
| <a id="fld_cell_tower_id"></a>cell_tower_id | `char[9]` |                                                                  |                                                                                                                            | ID of the currently connected cell tower. Must be NULL terminated if the length is less than 9 human-readable characters, and without NULL termination character if the length is exactly 9 characters (Invalid: 0)                                                                                        |
| <a id="fld_band_number"></a>band_number                          | `uint8`   |                                                                  |                                                                                                                            | LTE frequency band number (Invalid: 0)                                                                                                                                                                                                                                                                                     |
| <a id="fld_band_frequency"></a>band_frequency                    | `float32` | MHz                                                              |                                                                                                                            | LTE radio frequency (Invalid: 0)                                                                                                                                                                                                                                                                                           |
| <a id="fld_channel_number"></a>channel_number                    | `uint32`  |                                                                  |                                                                                                                            | Channel Number (CN), Absolute radio-frequency (ARFCN) / E-UTRA (EARFCN) / UTRA (UARFCN) / New radio (NR_CH) (Invalid: 0)                                                                               |
| <a id="fld_rx_level"></a>rx_level                                | `float32` | dBm                                                              |                                                                                                                            | Receiver signal level. On 3G is Received Signal Code Power (RSCP). On LTE Reference Signal Received Power (RSRP). On 5G is New Radio Reference Signal Received Power (NR_RSRP) (Invalid: 0)  |
| <a id="fld_tx_level"></a>tx_level                                | `float32` | dBm                                                              |                                                                                                                            | Transmitter signal absolute level (Invalid: 0)                                                                                                                                                                                                                                                                             |
| <a id="fld_rx_quality"></a>rx_quality                            | `float32` | dBm                                                              |                                                                                                                            | Received signal quality. On 3G is Receiver Quality (RxQual). On LTE is Reference Signal Received Quality (RSRQ). On 5G is New Radio Reference Signal Received Quality (NR_RSRQ) (Invalid: 0) |
| <a id="fld_sinr"></a>sinr                                                             | `float32` | dB                                                               |                                                                                                                            | Signal to Interference plus Noise Ratio (Invalid: 0)                                                                                                                                                                                                                                                                       |

## Enums

### STATUS_FLAG {#STATUS_FLAG}

Used in field(s): [status](#fld_status)

| 参数名                                                                                                         | 类型      | 值  | 描述                                                                                                                                                                                                                    |
| ----------------------------------------------------------------------------------------------------------- | ------- | -- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| <a id="#STATUS_FLAG_UNKNOWN"></a> STATUS_FLAG_UNKNOWN             | `uint8` | 0  | State unknown or not reportable                                                                                                                                                                                       |
| <a id="#STATUS_FLAG_FAILED"></a> STATUS_FLAG_FAILED               | `uint8` | 1  | Modem is unusable                                                                                                                                                                                                     |
| <a id="#STATUS_FLAG_INITIALIZING"></a> STATUS_FLAG_INITIALIZING   | `uint8` | 2  | Modem is being initialized                                                                                                                                                                                            |
| <a id="#STATUS_FLAG_LOCKED"></a> STATUS_FLAG_LOCKED               | `uint8` | 3  | Modem is locked                                                                                                                                                                                                       |
| <a id="#STATUS_FLAG_DISABLED"></a> STATUS_FLAG_DISABLED           | `uint8` | 4  | Modem is not enabled and is powered down                                                                                                                                                                              |
| <a id="#STATUS_FLAG_DISABLING"></a> STATUS_FLAG_DISABLING         | `uint8` | 5  | Modem is currently transitioning to the STATUS_FLAG_DISABLED state                                                                                                          |
| <a id="#STATUS_FLAG_ENABLING"></a> STATUS_FLAG_ENABLING           | `uint8` | 6  | Modem is currently transitioning to the STATUS_FLAG_ENABLED state                                                                                                           |
| <a id="#STATUS_FLAG_ENABLED"></a> STATUS_FLAG_ENABLED             | `uint8` | 7  | Modem is enabled and powered on but not registered with a network provider and not available for data connections                                                                                                     |
| <a id="#STATUS_FLAG_SEARCHING"></a> STATUS_FLAG_SEARCHING         | `uint8` | 8  | Modem is searching for a network provider to register                                                                                                                                                                 |
| <a id="#STATUS_FLAG_REGISTERED"></a> STATUS_FLAG_REGISTERED       | `uint8` | 9  | Modem is registered with a network provider, and data connections and messaging may be available for use                                                                                                              |
| <a id="#STATUS_FLAG_DISCONNECTING"></a> STATUS_FLAG_DISCONNECTING | `uint8` | 10 | Modem is disconnecting and deactivating the last active packet data bearer. This state will not be entered if more than one packet data bearer is active and one of the active bearers is deactivated |
| <a id="#STATUS_FLAG_CONNECTING"></a> STATUS_FLAG_CONNECTING       | `uint8` | 11 | Modem is activating and connecting the first packet data bearer. Subsequent bearer activations when another bearer is already active do not cause this state to be entered                            |
| <a id="#STATUS_FLAG_CONNECTED"></a> STATUS_FLAG_CONNECTED         | `uint8` | 12 | One or more packet data bearers is active and connected                                                                                                                                                               |

### FAILURE_REASON {#FAILURE_REASON}

Used in field(s): [failure_reason](#fld_failure_reason)

| 参数名                                                                                                                                | 类型      | 值 | 描述                                              |
| ---------------------------------------------------------------------------------------------------------------------------------- | ------- | - | ----------------------------------------------- |
| <a id="#FAILURE_REASON_NONE"></a> FAILURE_REASON_NONE                                    | `uint8` | 0 | No error                                        |
| <a id="#FAILURE_REASON_UNKNOWN"></a> FAILURE_REASON_UNKNOWN                              | `uint8` | 1 | Error state is unknown                          |
| <a id="#FAILURE_REASON_SIM_MISSING"></a> FAILURE_REASON_SIM_MISSING | `uint8` | 2 | SIM is required for the modem but missing       |
| <a id="#FAILURE_REASON_SIM_ERROR"></a> FAILURE_REASON_SIM_ERROR     | `uint8` | 3 | SIM is available, but not usable for connection |

### CELLULAR_NETWORK_RADIO_TYPE {#CELLULAR_NETWORK_RADIO_TYPE}

Used in field(s): [type](#fld_type)

| 参数名                                                                                                                                                                   | 类型      | 值 | 描述    |
| --------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------- | - | ----- |
| <a id="#CELLULAR_NETWORK_RADIO_TYPE_NONE"></a> CELLULAR_NETWORK_RADIO_TYPE_NONE   | `uint8` | 0 | None  |
| <a id="#CELLULAR_NETWORK_RADIO_TYPE_GSM"></a> CELLULAR_NETWORK_RADIO_TYPE_GSM     | `uint8` | 1 | GSM   |
| <a id="#CELLULAR_NETWORK_RADIO_TYPE_CDMA"></a> CELLULAR_NETWORK_RADIO_TYPE_CDMA   | `uint8` | 2 | CDMA  |
| <a id="#CELLULAR_NETWORK_RADIO_TYPE_WCDMA"></a> CELLULAR_NETWORK_RADIO_TYPE_WCDMA | `uint8` | 3 | WCDMA |
| <a id="#CELLULAR_NETWORK_RADIO_TYPE_LTE"></a> CELLULAR_NETWORK_RADIO_TYPE_LTE     | `uint8` | 4 | LTE   |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/CellularStatus.msg)

:::details
Click here to see original file

```c
# Cellular status
#
# This is currently used only for logging cell status from MAVLink 1:1.

uint64 timestamp # [us] Time since system start

uint8 status # [@enum STATUS_FLAG] Status
uint8 STATUS_FLAG_UNKNOWN = 0 # State unknown or not reportable
uint8 STATUS_FLAG_FAILED = 1 # Modem is unusable
uint8 STATUS_FLAG_INITIALIZING = 2 # Modem is being initialized
uint8 STATUS_FLAG_LOCKED = 3 # Modem is locked
uint8 STATUS_FLAG_DISABLED = 4 # Modem is not enabled and is powered down
uint8 STATUS_FLAG_DISABLING = 5 # Modem is currently transitioning to the STATUS_FLAG_DISABLED state
uint8 STATUS_FLAG_ENABLING = 6 # Modem is currently transitioning to the STATUS_FLAG_ENABLED state
uint8 STATUS_FLAG_ENABLED = 7 # Modem is enabled and powered on but not registered with a network provider and not available for data connections
uint8 STATUS_FLAG_SEARCHING = 8 # Modem is searching for a network provider to register
uint8 STATUS_FLAG_REGISTERED = 9 # Modem is registered with a network provider, and data connections and messaging may be available for use
uint8 STATUS_FLAG_DISCONNECTING = 10 # Modem is disconnecting and deactivating the last active packet data bearer. This state will not be entered if more than one packet data bearer is active and one of the active bearers is deactivated
uint8 STATUS_FLAG_CONNECTING = 11 # Modem is activating and connecting the first packet data bearer. Subsequent bearer activations when another bearer is already active do not cause this state to be entered
uint8 STATUS_FLAG_CONNECTED = 12 # One or more packet data bearers is active and connected

uint8 failure_reason # [@enum FAILURE_REASON] Failure reason
uint8 FAILURE_REASON_NONE = 0 # No error
uint8 FAILURE_REASON_UNKNOWN = 1 # Error state is unknown
uint8 FAILURE_REASON_SIM_MISSING = 2 # SIM is required for the modem but missing
uint8 FAILURE_REASON_SIM_ERROR = 3 # SIM is available, but not usable for connection

uint8 type # [@enum CELLULAR_NETWORK_RADIO_TYPE] Cellular network radio type
uint8 CELLULAR_NETWORK_RADIO_TYPE_NONE = 0 # None
uint8 CELLULAR_NETWORK_RADIO_TYPE_GSM = 1 # GSM
uint8 CELLULAR_NETWORK_RADIO_TYPE_CDMA = 2 # CDMA
uint8 CELLULAR_NETWORK_RADIO_TYPE_WCDMA = 3 # WCDMA
uint8 CELLULAR_NETWORK_RADIO_TYPE_LTE = 4 # LTE

uint8 quality # [%] [@invalid UINT8_MAX] Cellular network signal quality in percent. May be used for RSSI
uint16 mcc # [@invalid UINT16_MAX] Mobile country code
uint16 mnc # [@invalid UINT16_MAX] Mobile network code
uint16 lac # [@invalid 0] Location area code

uint8 id # [-] Cellular instance number. Indexed from 1. A value of 0 indicates the sender does not support reporting of multiple modems
uint32 link_tx_rate # [KiB/s] [@invalid 0] Download rate
uint32 link_rx_rate # [KiB/s] [@invalid 0] Upload rate
char[9] cell_tower_id # [@invalid 0] ID of the currently connected cell tower. Must be NULL terminated if the length is less than 9 human-readable characters, and without NULL termination character if the length is exactly 9 characters
uint8 band_number # [-] [@invalid 0] LTE frequency band number
float32 band_frequency # [MHz] [@invalid 0] LTE radio frequency
uint32 channel_number # [@invalid 0] Channel Number (CN), Absolute radio-frequency (ARFCN) / E-UTRA (EARFCN) / UTRA (UARFCN) / New radio (NR_CH)
float32 rx_level # [dBm] [@invalid 0] Receiver signal level. On 3G is Received Signal Code Power (RSCP). On LTE Reference Signal Received Power (RSRP). On 5G is New Radio Reference Signal Received Power (NR_RSRP)
float32 tx_level # [dBm] [@invalid 0] Transmitter signal absolute level
float32 rx_quality # [dBm] [@invalid 0] Received signal quality. On 3G is Receiver Quality (RxQual). On LTE is Reference Signal Received Quality (RSRQ). On 5G is New Radio Reference Signal Received Quality (NR_RSRQ)
float32 sinr # [dB] [@invalid 0] Signal to Interference plus Noise Ratio
```

:::
