uint64 timestamp		# time since system start (microseconds)

uint16 mission_id   		# Id for the mission for which the result was generated
uint16 geofence_id  		# Id for the corresponding geofence for which the result was generated (used for mission feasibility)
uint64 home_position_counter  	# Counter of the home position for which the result was generated (used for mission feasibility)

int32 seq_reached		# Sequence of the mission item which has been reached, default -1
uint16 seq_current		# Sequence of the current mission item
uint16 seq_total		# Total number of mission items

bool valid			# true if mission is valid
bool warning			# true if mission is valid, but has potentially problematic items leading to safety warnings
bool finished			# true if mission has been completed
bool failure			# true if the mission cannot continue or be completed for some reason

bool item_do_jump_changed	# true if the number of do jumps remaining has changed
uint16 item_changed_index	# indicate which item has changed
uint16 item_do_jump_remaining	# set to the number of do jumps remaining for that item

uint8 execution_mode	# indicates the mode in which the mission is executed
