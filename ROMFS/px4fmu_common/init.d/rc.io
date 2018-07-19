#!nsh
#
# PX4IO interface init script.
#

if px4io start
then
	# Allow PX4IO to recover from midair restarts.
	px4io recovery

	# Adjust PX4IO update rate limit.
	px4io limit 400
else
	echo "PX4IO start failed" >> $LOG_FILE
	tune_control play -m ${TUNE_ERR}
fi
