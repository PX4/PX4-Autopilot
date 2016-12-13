#!/bin/bash
#
# Upload SITL CI logs to Flight Review
#
# License: according to LICENSE.md in the root directory of the PX4 Firmware repository

if [ -z "$WORKSPACE" ] || [ -z "${ghprbActualCommitAuthorEmail}" ] || [ -z "${ghprbPullDescription}" ]; then
    echo "Environment not set. Needs to be called from within Jenkins."
    exit 1
fi

echo "Uploading test logs to Flight Review"

CMD="$WORKSPACE/Firmware/Tools/upload_log.py"
find "$WORKSPACE/test_results" -name \*.ulg -exec "$CMD" -q \
	--description "${ghprbPullDescription}" --source CI "{}" \;

# XXX: move up if we want email notifications
#	--email "${ghprbActualCommitAuthorEmail}" \
