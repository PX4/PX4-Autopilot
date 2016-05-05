#!/bin/bash

if [[ "$#" < 2 ]]; then
	echo "usage: adb_upload.sh SRC1 [SRC2 ...] DEST"
	exit
fi

echo "Wait for device..."
adb wait-for-device
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
	adb push $arg $last
	((i+=1))
done
