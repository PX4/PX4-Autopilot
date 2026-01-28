# OnboardComputerStatus (повідомлення UORB)

ONBOARD_COMPUTER_STATUS ДАНІ ПОВІДОМЛЕННЯ ПРО СТАН БОРТОВОГО КОМП'ЮТЕРА

[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/OnboardComputerStatus.msg)

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
