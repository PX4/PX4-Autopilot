uint64 timestamp # time since system start (microseconds)
uint32 uptime	 # time since system boot of the companion (milliseconds)

uint8 type	 # type of onboard computer

uint8[8] cpu_cores
uint8[10] cpu_combined
uint8[4] gpu_cores
uint8[10] gpu_combined
int8 temperature_board
int8[8] temperature_core
int16[4] fan_speed
uint32 ram_usage
uint32 ram_total
uint32[4] storage_type
uint32[4] storage_usage
uint32[4] storage_total
uint32[6] link_type
uint32[6] link_tx_rate
uint32[6] link_rx_rate
uint32[6] link_tx_max
uint32[6] link_rx_max
