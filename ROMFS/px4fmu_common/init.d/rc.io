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
if ver hwcmp PX4FMU_V1
then
	set PX4IO_LIMIT 200
fi

if px4io limit $PX4IO_LIMIT
then
else
	echo "[i] Set PX4IO update rate to $PX4IO_LIMIT Hz failed!"
fi
