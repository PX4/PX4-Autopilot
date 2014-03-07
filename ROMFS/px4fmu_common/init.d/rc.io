#
# Init PX4IO interface
#

#
# Allow PX4IO to recover from midair restarts.
#
px4io recovery

#
# Adjust PX4IO update rate limit
#
set PX4IO_LIMIT 400
if hw_ver compare PX4FMU_V1
then
	set PX4IO_LIMIT 200
fi

echo "[init] Set PX4IO update rate limit: $PX4IO_LIMIT Hz"
px4io limit $PX4IO_LIMIT
