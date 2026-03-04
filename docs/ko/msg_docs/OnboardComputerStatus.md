---
pageClass: is-wide-page
---

# OnboardComputerStatus (UORB message)

ONBOARD_COMPUTER_STATUS message data.

**TOPICS:** onboard_computerstatus

## Fields

| 명칭                                                     | 형식          | Unit [Frame] | Range/Enum | 설명                                                                                                                                                                                                                                                                                              |
| ------------------------------------------------------ | ----------- | ---------------------------------------------------------------- | ---------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| timestamp                                              | `uint64`    | us                                                               |            | time since system start (microseconds)                                                                                                                                                                                                                                       |
| uptime                                                 | `uint32`    | ms                                                               |            | time since system boot of the companion (milliseconds)                                                                                                                                                                                                                       |
| type                                                   | `uint8`     |                                                                  |            | type of onboard computer 0: Mission computer primary, 1: Mission computer backup 1, 2: Mission computer backup 2, 3: Compute node, 4-5: Compute spares, 6-9: Payload computers. |
| cpu_cores                         | `uint8[8]`  |                                                                  |            | CPU usage on the component in percent                                                                                                                                                                                                                                                           |
| cpu_combined                      | `uint8[10]` |                                                                  |            | Combined CPU usage as the last 10 slices of 100 MS                                                                                                                                                                                                                                              |
| gpu_cores                         | `uint8[4]`  |                                                                  |            | GPU usage on the component in percent                                                                                                                                                                                                                                                           |
| gpu_combined                      | `uint8[10]` |                                                                  |            | Combined GPU usage as the last 10 slices of 100 MS                                                                                                                                                                                                                                              |
| temperature_board                 | `int8`      | degC                                                             |            | Temperature of the board                                                                                                                                                                                                                                                                        |
| temperature_core                  | `int8[8]`   | degC                                                             |            | Temperature of the CPU core                                                                                                                                                                                                                                                                     |
| fan_speed                         | `int16[4]`  | rpm                                                              |            | Fan speeds                                                                                                                                                                                                                                                                                      |
| ram_usage                         | `uint32`    | MB                                                               |            | Amount of used RAM on the component system                                                                                                                                                                                                                                                      |
| ram_total                         | `uint32`    | MB                                                               |            | Total amount of RAM on the component system                                                                                                                                                                                                                                                     |
| storage_type                      | `uint32[4]` |                                                                  |            | Storage type: 0: HDD, 1: SSD, 2: EMMC, 3: SD card (non-removable), 4: SD card (removable)                                                                 |
| storage_usage                     | `uint32[4]` | MB                                                               |            | Amount of used storage space on the component system                                                                                                                                                                                                                                            |
| storage_total                     | `uint32[4]` | MB                                                               |            | Total amount of storage space on the component system                                                                                                                                                                                                                                           |
| link_type                         | `uint32[6]` | Kb/s                                                             |            | Link type: 0-9: UART, 10-19: Wired network, 20-29: Wifi, 30-39: Point-to-point proprietary, 40-49: Mesh proprietary                                                                             |
| link_tx_rate | `uint32[6]` | Kb/s                                                             |            | Network traffic from the component system                                                                                                                                                                                                                                                       |
| link_rx_rate | `uint32[6]` | Kb/s                                                             |            | Network traffic to the component system                                                                                                                                                                                                                                                         |
| link_tx_max  | `uint32[6]` | Kb/s                                                             |            | Network capacity from the component system                                                                                                                                                                                                                                                      |
| link_rx_max  | `uint32[6]` | Kb/s                                                             |            | Network capacity to the component system                                                                                                                                                                                                                                                        |

## Source Message

[Source file (GitHub)](https://github.com/PX4/PX4-Autopilot/blob/main/msg/OnboardComputerStatus.msg)

:::details
Click here to see original file

```c
# ONBOARD_COMPUTER_STATUS message data
uint64 timestamp # [us] time since system start (microseconds)
uint32 uptime	 # [ms] time since system boot of the companion (milliseconds)

uint8 type	 # type of onboard computer 0: Mission computer primary, 1: Mission computer backup 1, 2: Mission computer backup 2, 3: Compute node, 4-5: Compute spares, 6-9: Payload computers.

uint8[8] cpu_cores # CPU usage on the component in percent
uint8[10] cpu_combined # Combined CPU usage as the last 10 slices of 100 MS
uint8[4] gpu_cores # GPU usage on the component in percent
uint8[10] gpu_combined # Combined GPU usage as the last 10 slices of 100 MS
int8 temperature_board # [degC] Temperature of the board
int8[8] temperature_core # [degC] Temperature of the CPU core
int16[4] fan_speed # [rpm] Fan speeds
uint32 ram_usage # [MB] Amount of used RAM on the component system
uint32 ram_total # [MB] Total amount of RAM on the component system
uint32[4] storage_type # Storage type: 0: HDD, 1: SSD, 2: EMMC, 3: SD card (non-removable), 4: SD card (removable)
uint32[4] storage_usage # [MB] Amount of used storage space on the component system
uint32[4] storage_total # [MB] Total amount of storage space on the component system
uint32[6] link_type # [Kb/s] Link type: 0-9: UART, 10-19: Wired network, 20-29: Wifi, 30-39: Point-to-point proprietary, 40-49: Mesh proprietary
uint32[6] link_tx_rate # [Kb/s] Network traffic from the component system
uint32[6] link_rx_rate # [Kb/s] Network traffic to the component system
uint32[6] link_tx_max # [Kb/s] Network capacity from the component system
uint32[6] link_rx_max # [Kb/s] Network capacity to the component system
```

:::
