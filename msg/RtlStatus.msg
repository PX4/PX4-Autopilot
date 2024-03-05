uint64 timestamp                      # time since system start (microseconds)

uint32 safe_points_id 		      # unique ID of active set of safe_point_items
bool is_evaluation_pending 	      # flag if the RTL point needs reevaluation (e.g. new safe points available, but need loading).

bool has_vtol_approach 		      # flag if approaches are defined for current RTL_TYPE parameter setting

uint8 rtl_type	      		      # Type of RTL chosen
uint8 safe_point_index 		      # index of the chosen safe point, if in RTL_STATUS_TYPE_DIRECT_SAFE_POINT mode

uint8 RTL_STATUS_TYPE_NONE=0       # RTL type is pending if evaluation can't pe performed currently e.g. when it is still loading the safe points
uint8 RTL_STATUS_TYPE_DIRECT_SAFE_POINT=1 # RTL type is chosen to directly go to a safe point or home position
uint8 RTL_STATUS_TYPE_DIRECT_MISSION_LAND=2 # RTL type is going straight to the beginning of the mission landing
uint8 RTL_STATUS_TYPE_FOLLOW_MISSION=3 	# RTL type is following the mission from closest point to mission landing
uint8 RTL_STATUS_TYPE_FOLLOW_MISSION_REVERSE=4 # RTL type is following the mission in reverse to the start position
