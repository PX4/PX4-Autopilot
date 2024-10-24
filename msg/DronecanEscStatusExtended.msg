uint64 timestamp						# time since system start (microseconds)

uint8 CONNECTED_ESC_MAX = 8					# The number of ESCs supported. To be consistent with ESC Status, limit it to 8.

DronecanEscStatusExtendedData[8] extended_esc_status_data	# An array of up to CONNECTED_ESC_MAX DronecanEscStatusExtendedData instances
