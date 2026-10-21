uint64 timestamp
uint64 boot_id
uint64 session_id
uint32 sequence
uint32 identity_revision
uint32 rid_boot_id
uint32 last_ack_age_ms
uint16 error_code
uint8 source_system
uint8 source_component
uint8 environment
uint8 state
uint8 flags
uint8[32] identity_hash
uint8 ENVIRONMENT_MOCK = 1
uint8 ENVIRONMENT_OFFICIAL = 2
uint8 STATE_FAILED = 6
uint8 STATE_READY = 3
uint8 STATE_REPORTING = 4
