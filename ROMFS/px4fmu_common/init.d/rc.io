#
# Init PX4IO interface
#

#
# Allow PX4IO to recover from midair restarts.
# this is very unlikely, but quite safe and robust.
#
px4io recovery

#
# Adjust px4io topic limiting
#
if hw_ver compare PX4FMU_V1
then
	px4io limit 200
else
	px4io limit 400
fi
