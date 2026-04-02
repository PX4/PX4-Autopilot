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

# Detect install prefix (SIH uses /opt/px4, Gazebo uses /opt/px4-gazebo)
if [ -d /opt/px4-gazebo ]; then
    PX4_PREFIX=/opt/px4-gazebo
else
    PX4_PREFIX=/opt/px4
fi

if getent hosts host.docker.internal >/dev/null 2>&1; then
    DOCKER_HOST_IP=$(getent hosts host.docker.internal | awk '{print $1}')

    # MAVLink: replace default target (127.0.0.1) with the Docker host IP
    sed -i "s/mavlink start -x -u/mavlink start -x -t $DOCKER_HOST_IP -u/g" \
        "$PX4_PREFIX/etc/init.d-posix/px4-rc.mavlink"

    # DDS: point uXRCE-DDS client at the host
    sed -i "s|uxrce_dds_client start -t udp|uxrce_dds_client start -t udp -h $DOCKER_HOST_IP|" \
        "$PX4_PREFIX/etc/init.d-posix/rcS"
fi

exec "$PX4_PREFIX/bin/px4" "$@"
