#!/bin/bash
# upload script for network-connected devices via scp (eg Raspberry Pi)

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

user=pi
if [ -n "${AUTOPILOT_USER}" ]; then
	user=${AUTOPILOT_USER}
fi

# Get last argument
for last; do true; done

# All except last argument
length=$(($#-1))
src_files=${@:1:$length}

echo "Uploading $src_files..."

# Upload files
scp -r $src_files ${user}@${host}:$last
