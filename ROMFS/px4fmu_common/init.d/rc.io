#
# Start PX4IO interface (depends on orb, commander)
#
if px4io start
then
	#
	# Allow PX4IO to recover from midair restarts.
	# this is very unlikely, but quite safe and robust.
	px4io recovery

	#
	# Disable px4io topic limiting
	#
	if [ $BOARD == fmuv1 ]
	then
		px4io limit 200
	else
		px4io limit 400
	fi
else
	# SOS
	tone_alarm error
fi
