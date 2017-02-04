#!/bin/bash

filename=${1}

# Requires these ENV variables
# AWS_ACCESS_KEY_ID
# AWS_SECRET_ACCESS_KEY
# AWS_S3_BUCKET

[ -z "$AWS_ACCESS_KEY_ID" ] && { echo "ERROR: Need to set AWS_ACCESS_KEY_ID"; exit 1; }
[ -z "$AWS_SECRET_ACCESS_KEY" ] && { echo "ERROR: Need to set AWS_SECRET_ACCESS_KEY"; exit 1; }
[ -z "$AWS_S3_BUCKET" ] && { echo "ERROR: Need to set AWS_S3_BUCKET"; exit 1; }

if [ -f ${filename} ]; then
	base_file_name=`basename $filename`
	s3cmd --access_key=${AWS_ACCESS_KEY_ID} --secret_key=${AWS_SECRET_ACCESS_KEY} put ${filename} s3://${AWS_S3_BUCKET}/${base_file_name}
else
	echo "ERROR: ${file} doesn't exist"
	exit 1
fi
