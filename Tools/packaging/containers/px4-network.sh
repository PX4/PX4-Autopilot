#!/bin/sh
# Shared host MAVLink routing; the calling entrypoint chooses the DDS destination.

if [ -d /opt/px4-gazebo ]; then
    PX4_PREFIX=/opt/px4-gazebo
else
    PX4_PREFIX=/opt/px4
fi

# Both PX4 clients require IPv4; Docker Desktop may also return an IPv6 address.
DOCKER_HOST_IP=$(getent ahostsv4 host.docker.internal 2>/dev/null | awk '/STREAM/ {print $1; exit}')

if [ -n "$DOCKER_HOST_IP" ]; then
    # Replace an existing target as well, so repeated entrypoint calls are safe.
    sed -i -E "s/(mavlink start -x)( -t [^ ]+)? -u/\1 -t $DOCKER_HOST_IP -u/g" \
        "$PX4_PREFIX/etc/init.d-posix/px4-rc.mavlink"
fi
