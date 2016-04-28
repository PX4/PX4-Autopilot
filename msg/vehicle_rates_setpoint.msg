###############################################################################################
# The vehicle_rates_setpoint.msg needs to be in sync with the virtual setpoint messages
#
# Please keep the following messages identical;
#	vehicle_rates_setpoint.msg
#	mc_virtual_rates_setpoint.msg
#	fw_virtual_rates_setpoint.msg
#
###############################################################################################

uint64 timestamp    # in microseconds since system start

float32 roll	    # body angular rates in NED frame
float32 pitch	    # body angular rates in NED frame
float32 yaw			# body angular rates in NED frame
float32 thrust	    # thrust normalized to 0..1
