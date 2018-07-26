#!nsh
#
# PX4IO interface init script.
#

#
# Check if PX4IO present and update firmware if needed.
#
if [ -f /etc/extras/px4io-v2.bin ]
then
	set IO_FILE /etc/extras/px4io-v2.bin

	if px4io checkcrc ${IO_FILE}
	then
		set IO_PRESENT yes
	else
		tune_control play -m MLL32CP8MB

		if px4io start
		then
			# Try to safety px4 io so motor outputs dont go crazy.
			if px4io safety_on
			then
				# success! no-op
			else
				# px4io did not respond to the safety command.
				px4io stop
			fi
		fi

		if px4io forceupdate 14662 ${IO_FILE}
		then
			usleep 10000
			if px4io checkcrc ${IO_FILE}
			then
				echo "PX4IO CRC OK after updating" >> $LOG_FILE
				tune_control play -m MLL8CDE

				set IO_PRESENT yes
			else
				echo "PX4IO update failed" >> $LOG_FILE
				# Error tune.
				tune_control play -t 2
			fi
		else
			echo "PX4IO update failed" >> $LOG_FILE
			# Error tune.
			tune_control play -t 2
		fi
	fi
fi

if [ $USE_IO == yes -a $IO_PRESENT == no ]
then
	echo "PX4IO not found" >> $LOG_FILE
	# Error tune.
	tune_control play -t 2
fi

if px4io start
then
	# Allow PX4IO to recover from midair restarts.
	px4io recovery

	# Adjust PX4IO update rate limit.
	px4io limit 400
else
	echo "PX4IO start failed" >> $LOG_FILE
	# Error tune.
	tune_control play -t 2
fi
