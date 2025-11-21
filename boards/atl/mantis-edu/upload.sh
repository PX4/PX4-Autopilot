#!/usr/bin/env bash

set -e

PX4_BINARY_FILE="$1"

echo "uploading: $PX4_BINARY_FILE"

PX4_BINARY_FILE_SIZE=$(stat -c%s "$PX4_BINARY_FILE")
curl -v -F "image=@$PX4_BINARY_FILE" -H "Expect:" -H "File-Size:$PX4_BINARY_FILE_SIZE" http://192.168.42.1/cgi-bin/upload
