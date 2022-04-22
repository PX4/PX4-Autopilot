#!/bin/bash

if [ "$1" == "-h" ]; then
   cat << USAGE

Script to test sitl simulation
  Give tag argument to run specific tii-px4-sitl container. Default tag is 'master'
  Usage: $0 [tag]

USAGE

   exit 0
fi


tii_px4_sitl=${1-master}

echo "Run simulation with ghcr.io/tiiuae/tii-px4-sitl:${tii_px4_sitl}"

if [ ! -d "/tmp/gazebo-data" ]; then
    echo "gazebo-data not found, download to '/tmp/gazebo-data'"
    docker create --name gdata_cont ghcr.io/tiiuae/tii-gazebo-data:tcp_test3
    docker cp gdata_cont:/gazebo-data /tmp
    docker rm gdata_cont
    echo "gazebo-data download done."
fi

if [ "$(docker ps | grep px4_sitl)" != "" ]; then
    echo "Old px4_sitl container found running, removing..."
    docker stop px4_sitl
fi

if [ "$(docker ps | grep gzserver)" != "" ]; then
    echo "Old gzserver container found running, removing..."
    docker stop gzserver
fi

echo "Start px4"
docker run -d --rm --name px4_sitl --network=host --env PX4_SIM_USE_TCP_SERVER=1 --env PX4_SIM_MODE=ssrc_fog_x ghcr.io/tiiuae/tii-px4-sitl:$tii_px4_sitl
echo "Start gzserver"
docker run -d --rm --name gzserver --network=host --env PX4_SIM_USE_TCP_SERVER=1 -v /tmp/gazebo-data:/data --env IP_ADDR=127.0.0.1 ghcr.io/tiiuae/tii-gzserver:tcp_test3 empty.world
echo "Wait 2 sec.."
sleep 2
echo "Spawn drone1 into simulation"
docker exec gzserver /gzserver-api/scripts/spawn-drone.sh "127.0.0.1" 12460 4560 5600 drone1 0 0 0 0 0 0
echo
cat << EOF

To get logs:
  $ docker logs -f px4_sitl
  $ docker logs -f gzserver

QGC connection:
  udp:14550

To stop simulation:
  $ docker stop px4_sitl; docker stop gzserver

EOF
