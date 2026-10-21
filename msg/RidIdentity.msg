uint64 timestamp
uint64 boot_id
uint64 session_id
uint32 request_id
uint32 identity_revision
uint32 rid_boot_id
uint32 pending_revision
uint8 target_system
uint8 target_component
uint8 result
uint8 activation_state
uint8 rid_node_id
uint8 aircraft_class
uint8 activation_error
char[12] rid_uid
char[20] uas_id
char[64] registration_id
char[8] uin
uint8[32] identity_hash
char[48] error
uint8 RESULT_OK = 0
uint8 RESULT_NOT_FOUND = 1
uint8 RESULT_READING = 2
uint8 RESULT_INVALID = 3
uint8 RESULT_UNSUPPORTED = 4
uint8 RESULT_TIMEOUT = 5
uint8 RESULT_CONFLICT = 6
uint8 ACTIVATION_ACTIVE = 1
