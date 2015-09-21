int32 dataman_id	# default 0, there are two offboard storage places in the dataman: 0 or 1
uint32 count		# count of the missions stored in the dataman
int32 current_seq	# default -1, start at the one changed latest

# fixme: there is no mission definition in objects_common.cpp
# but it's required for systemcmds/topic_listener/topic_listener
# TOPICS mission offboard_mission onboard_mission
