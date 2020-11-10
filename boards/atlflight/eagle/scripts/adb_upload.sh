#!/bin/bash

if [[ "$#" < 2 ]]; then
	echo "usage: adb_upload.sh SRC1 [SRC2 ...] DEST"
	exit
fi

# Get last argument
for last; do true; done

echo "Wait for device..."
adb wait-for-device

echo "Creating folder structure..."
adb shell mkdir -p $last

echo "Uploading..."
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

# Make sure they are synced to the file system
echo "Syncing FS..."
adb shell sync
