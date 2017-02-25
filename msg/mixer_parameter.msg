#See mavlink for enumerations of data type

uint8 mixer_group	    # Specific mixer (onboard, failsafe, bus connected etc..)
uint8 mixer_index	    # Index of mixer in the group
uint8 mixer_sub_index	# Index of sub mixer in a specific mixer
uint8 parameter_index	# Parameter index in a mixer or sub mixer
float32 real_value      # Floating point data or parameter value
uint32 int_value        # Integer data or parameter value
uint8 param_type	    # Parameter type
