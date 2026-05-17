#!/bin/sh
# Rewrite the container's apt sources to point at the AWS regional Ubuntu
# mirror that is local to the runs-on instance.
#
# The default archive.ubuntu.com round-robin sometimes serves out-of-sync
# index files mid-sync, breaking apt-get update with errors like:
#   File has unexpected size (25378 != 25381). Mirror sync in progress?
# The Canonical-operated EC2 mirrors are region-local and sync aggressively,
# eliminating that failure mode.
#
# This script is a no-op outside runs-on, so it is safe to call from any CI
# job (forks, self-hosted runners, local docker runs, etc.) without changing
# behavior there.
#
# Usage (from a workflow step running inside the container):
#   ./Tools/ci/use_aws_apt_mirror.sh

set -e

if [ -z "$RUNS_ON_AWS_REGION" ]; then
    echo "use_aws_apt_mirror: not running on runs-on (RUNS_ON_AWS_REGION unset), skipping"
    exit 0
fi

MIRROR="http://${RUNS_ON_AWS_REGION}.ec2.archive.ubuntu.com/ubuntu"
echo "use_aws_apt_mirror: rewriting apt sources to ${MIRROR}"

# Noble (24.04+) uses the deb822 format at /etc/apt/sources.list.d/ubuntu.sources
if [ -f /etc/apt/sources.list.d/ubuntu.sources ]; then
    sed -i \
        -e "s|http://archive.ubuntu.com/ubuntu|${MIRROR}|g" \
        -e "s|http://security.ubuntu.com/ubuntu|${MIRROR}|g" \
        /etc/apt/sources.list.d/ubuntu.sources
fi

# Jammy (22.04) and earlier use the legacy /etc/apt/sources.list
if [ -f /etc/apt/sources.list ]; then
    sed -i \
        -e "s|http://archive.ubuntu.com/ubuntu|${MIRROR}|g" \
        -e "s|http://security.ubuntu.com/ubuntu|${MIRROR}|g" \
        /etc/apt/sources.list
fi
