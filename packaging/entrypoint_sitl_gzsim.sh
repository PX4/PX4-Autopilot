#!/bin/bash
#

export PATH=/px4_sitl/bin:$PATH

case $PX4_VEHICLE_TYPE in
  mc)
    export PX4_SYS_AUTOSTART=4401
    export PX4_GZ_MODEL=holybro-x500
    ;;
  rover)
    export PX4_SYS_AUTOSTART=50005
    export PX4_GZ_MODEL=scout_mini
    ;;
  vtol)
    export PX4_SYS_AUTOSTART=4004
    export PX4_GZ_MODEL=standard_vtol
    ;;
  fw)
    export PX4_SYS_AUTOSTART=4003
    export PX4_GZ_MODEL=rc_cessna
    ;;
  *)
    echo "ERROR: unknown vehicle type: $PX4_VEHICLE_TYPE"
    exit 1
    ;;
esac

export PX4_GZ_MODEL_NAME=$DRONE_DEVICE_ID
export PX4_GZ_WORLD=${PX4_GZ_WORLD:-default}
export GZ_PARTITION=sim
export GZ_RELAY=$(dig +short gazebo-server)
export GZ_IP=${GZ_IP:-$(hostname -i)}

source /opt/ros/humble/setup.sh

/px4_sitl/bin/px4 -d -s /px4_sitl/etc/init.d-posix/rcS -w /px4_sitl
