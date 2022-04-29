#!/bin/bash

set -e

###########################################################
# Argument parsing

usage() {
	echo "
Usage: $(basename "$0") [-h] [-d] [-n count] [-t tag]
 -- Run multiple containerized px4_sitl instances in gazebo simulation
Params:
    -h  Show help text.
    -d  Delete running simulation
    -n  Count of px4_sitl instances launched into simulation. Default: 1
    -t  Use specific tii-px4-sitl:<tag> image for sitl instances. Default: master
    -g  gzserver image name.    Default: ghcr.io/tiiuae/tii-gzserver:tcp
    -i  gazebo-data image name. Default: ghcr.io/tiiuae/tii-gazebo-data:tcp_test3
"
	exit 0
}

check_arg() {
	if [ "$(echo $1 | cut -c1)" = "-" ]; then
		return 1
	else
		return 0
	fi
}

error_arg() {
	echo "$0: option requires an argument -- $1"
	usage
}

delete_current_sim=0
instance_count=1
image_tag="master"
gzserver_image=ghcr.io/tiiuae/tii-gzserver:tcp
data_image=ghcr.io/tiiuae/tii-gazebo-data:tcp_test3

while getopts "hdn:t:g:i:" opt
do
	case $opt in
		h)
			usage
			;;
		d)
			delete_current_sim=1
			;;
		n)
			check_arg $OPTARG && instance_count=$OPTARG || error_arg $opt
			;;
		t)
			check_arg $OPTARG && image_tag=$OPTARG || error_arg $opt
			;;
		g)
			check_arg $OPTARG && image_tag=$OPTARG || error_arg $opt
			;;
		i)
			check_arg $OPTARG && image_tag=$OPTARG || error_arg $opt
			;;
		\?)
			usage
			;;
	esac
done


###########################################################
# Main

delete_simulation() {
  echo "Delete simulation:"
  pids=()
  dronestr=$(docker ps | grep px4_sitl_drone | awk '{ print $1 }')
  docker stop gzserver >/dev/null 2>&1 &
  pid=$!
  pids+=($pid)
  echo "  Removing gzserver (pid $pid)"

  if [ "$dronestr" != "" ]; then
    readarray -t drone_list <<<"$dronestr"
    for dr in ${drone_list[@]}
    do
      docker stop $dr >/dev/null 2>&1 &
      pid=$!
      pids+=($pid)
      echo "  Removing drone '$dr' (pid $pid)"
    done
  fi

  for p in ${pids[@]}
  do
    wait $p
    echo "  pid $p removed."
  done
  echo "Done."
}


if [ $delete_current_sim = 1 ]; then
  delete_simulation
  exit 0
fi


echo "Run simulation with px4_sitl image: ghcr.io/tiiuae/tii-px4-sitl:${image_tag}"

if [ ! -d "/tmp/gazebo-data" ]; then
    echo "gazebo-data not found, download to '/tmp/gazebo-data'"
    docker create --name gdata_cont ${data_image}
    docker cp gdata_cont:/gazebo-data /tmp
    docker rm gdata_cont
    echo "gazebo-data download done."
fi

if [ "$(docker ps | grep px4_sitl_drone)" != "" ] || [ "$(docker ps | grep gzserver)" != "" ]; then
    echo "Old simulation found running, removing..."
    delete_simulation
fi

echo "Start gzserver"
docker run -d --rm --name gzserver --env PX4_SIM_USE_TCP_SERVER=1 -v /tmp/gazebo-data:/data ${gzserver_image} empty.world

echo "Starting ${instance_count} PX4 instances"
echo

for i in $(seq 1 $instance_count)
do
  drone=px4_sitl_drone${i}
  echo "Start px4 instance #${i}/${instance_count}"
  docker run -d --rm --name $drone --env PX4_SIM_USE_TCP_SERVER=1 --env PX4_SIM_MODE=ssrc_fog_x ghcr.io/tiiuae/tii-px4-sitl:${image_tag}
  echo "  Wait 1 sec.."
  sleep 1
  drone_ip=$(docker inspect -f '{{ .NetworkSettings.IPAddress }}' ${drone})
  echo "  Spawn ${drone} into gazebo simulation instance"
  docker exec gzserver /gzserver-api/scripts/spawn-drone.sh ${drone_ip} 12460 4560 5600 ${drone} 0 0 0 0 0 0
  echo
done

cat << EOF

To get logs:
  $ docker logs -f gzserver
  $ docker logs -f px4_sitl_drone1
  $ docker logs -f px4_sitl_drone2
  ..
  $ docker logs -f px4_sitl_droneN

QGC connection:
  udp:14550

To stop simulation:
  $ $0 -d

EOF
