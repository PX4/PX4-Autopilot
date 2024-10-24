uint64 timestamp						# time since system start (microseconds)

uint8 MAX_NODE_STATUSES_LOGGED = 8				# Hold up to 8 nodes worth of Node Status data in the logs. Limited to 8 to avoid needlessly large log files.

DronecanNodeStatusData[8] node_status_id			# An array of up to MAX_NODE_STATUSES_LOGGED DronecanNodeStatusData instances
