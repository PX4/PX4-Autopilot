#!/bin/sh
# Docker entrypoint for PX4 SITL containers.
#
# On Docker Desktop (macOS/Windows), host.docker.internal resolves to the
# host machine. We detect this and configure MAVLink + DDS to send to the
# host IP instead of localhost (which stays inside the container VM).
#
# On Linux with --network host, host.docker.internal does not resolve and
# PX4 defaults work without modification.

set -e

. /usr/local/lib/px4/network.sh

if [ -n "$DOCKER_HOST_IP" ]; then
    # Runtime-only images expect the DDS Agent on the host.
    sed -i -E "s/(uxrce_dds_client start -t udp)( -h [^ ]+)?/\1 -h $DOCKER_HOST_IP/g" \
        "$PX4_PREFIX/etc/init.d-posix/rcS"
fi

exec "$PX4_PREFIX/bin/px4" "$@"
