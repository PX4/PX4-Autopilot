# this file is only used in the position_setpoint triple as a dependency

#See mavlink for enumerations of mixer type and data type

uint8 mixer_group	    # Specific mixer (onboard, failsafe, bus connected etc..)
uint8 mixer_index	    # Index of mixer in the group
uint8 mixer_sub_index	# Index of sub mixer in a specific mixer
uint8 parameter_index	# Parameter index in a mixer or sub mixer. Reused as connection index.
uint8 connection_type	# Connection type for a mixer or sub mixer
uint8 connection_group	# Connection group that connection is attached to
uint8 mixer_data_type	# The group/mixer/submixer/parameter data requested
float32 real_value      # Real data or parameter value
uint32 int_value        # Integer data or parameter value
uint8 param_type	    # Parameter type
