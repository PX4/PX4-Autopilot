#!/bin/bash
# run multiple instances of the 'px4' binary, but w/o starting the simulator.
# It assumes px4 is already built, with 'make px4_sitl_default'

# The simulator is expected to send to UDP port 14560+i for i in [0, N-1]
# For example jmavsim can be run like this:
#./Tools/jmavsim_run.sh -p 14561

sitl_num=2
[ -n "$1" ] && sitl_num="$1"

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="$SCRIPT_DIR/.."

build_path=${src_path}/build/px4_sitl_default

echo "killing running instances"

pkill -x px4 || true
pkill -x gazebo || true
pkill -x gzserver || true

sleep 1

export PX4_SIM_MODEL=iris

n=0
while [ $n -lt $sitl_num ]; do
    working_dir="$build_path/instance_$n"
    [ ! -d "$working_dir" ] && mkdir -p "$working_dir"
    
    pushd "$working_dir" &>/dev/null
    echo "starting instance $n in $(pwd)"
    ../bin/px4 -i $n -d "$src_path/ROMFS/px4fmu_common" -s etc/init.d-posix/rcS >out.log 2>err.log &
    popd &>/dev/null
    
    n=$(($n + 1))
done

model="iris"

source "$src_path/Tools/setup_gazebo.bash" "${src_path}" "${build_path}"

gzserver --verbose "${src_path}/Tools/sitl_gazebo/worlds/${model}.world" &
SIM_PID=`echo $!`

if [[ -n "$HEADLESS" ]]; then
    echo "not running gazebo gui"
else
    # gzserver needs to be running to avoid a race. Since the launch
    # is putting it into the background we need to avoid it by backing off
    sleep 3
    nice -n 20 gzclient --verbose &
    GUI_PID=`echo $!`
fi

