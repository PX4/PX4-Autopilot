uint64 timestamp		# time since system start (microseconds)

uint8 primary_instance

uint8 instances_available

uint32 instance_changed_count
uint64 last_instance_change

float32[6] combined_test_ratio
float32[6] relative_test_ratio
bool[6] healthy

float32[4] accumulated_gyro_error
bool gyro_fault_detected
