uint32 instance_count		# Instance count of this mission. Increments monotonically whenever the mission is modified

int32 seq_reached		# Sequence of the mission item which has been reached, default -1
uint16 seq_current		# Sequence of the current mission item
uint16 seq_total		# Total number of mission items

bool valid			# true if mission is valid
bool warning			# true if mission is valid, but has potentially problematic items leading to safety warnings
bool finished			# true if mission has been completed
bool failure			# true if the mission cannot continue or be completed for some reason

bool stay_in_failsafe		# true if the commander should not switch out of the failsafe mode
bool flight_termination		# true if the navigator demands a flight termination from the commander app

bool item_do_jump_changed	# true if the number of do jumps remaining has changed
uint16 item_changed_index	# indicate which item has changed
uint16 item_do_jump_remaining	# set to the number of do jumps remaining for that item
