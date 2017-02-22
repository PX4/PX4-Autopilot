#See mavlink for enumerations of mixer type and data type

uint8 mixer_group	    # Specific mixer (onboard, failsafe, bus connected etc..)
uint8 mixer_index	    # Index of mixer in the group
uint8 mixer_sub_index	# Index of sub mixer in a specific mixer
uint8 parameter_index	# Parameter index in a mixer or sub mixer. Reused as connection index.
uint8 connection_type   # Mixer connection type
uint8 mixer_data_type	# The group/mixer/submixer/parameter data requested
uint64 dataref          # Data reference
