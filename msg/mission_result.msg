uint32 instance_count # Instance count of this mission. Increments monotonically whenever the mission is modified
uint32 seq_reached		# Sequence of the mission item which has been reached
uint32 seq_current		# Sequence of the current mission item
bool valid			# true if mission is valid
bool warning			# true if mission is valid, but has potentially problematic items leading to safety warnings
bool reached			# true if mission has been reached
bool finished			# true if mission has been completed
bool stay_in_failsafe		# true if the commander should not switch out of the failsafe mode
bool flight_termination		# true if the navigator demands a flight termination from the commander app
bool item_do_jump_changed	# true if the number of do jumps remaining has changed
uint32 item_changed_index	# indicate which item has changed
uint32 item_do_jump_remaining	# set to the number of do jumps remaining for that item
bool mission_failure		# true if the mission cannot continue or be completed for some reason
