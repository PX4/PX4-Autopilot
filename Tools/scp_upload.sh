#!/bin/bash

if [[ "$#" < 2 ]]; then
	echo "usage: scp_upload.sh SRC1 [SRC2 ...] DEST"
	exit
fi

if [ -z ${AUTOPILOT_HOST+x} ]; then
	host=px4autopilot
	echo "\$AUTOPILOT_HOST is not set (use default: $host)"
else
	host=$AUTOPILOT_HOST
	echo "\$AUTOPILOT_HOST is set to $host"
fi

echo "Uploading..."

# Get last argument
for last; do true; done

# Go through source files and push them one by one.
i=0
for arg
do
	if [[ $((i+1)) == "$#" ]]; then
		break
	fi
	# echo "Pushing $arg to $last"
	#adb push $arg $last
	scp $arg pi@$host:$last
	((i+=1))
done
