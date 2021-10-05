#!/bin/bash

jinja_gen_script="./Tools/sitl_gazebo/scripts/jinja_gen.py"
sdf_jinja_template="./Tools/sitl_gazebo/models/ssrc_fog_x/ssrc_fog_x.sdf.jinja"
workdir="./Tools/sitl_gazebo/"

echo "Generate ssrc_fog_x_hitl.sdf"
python3 $jinja_gen_script $sdf_jinja_template $workdir \
	--serial_enabled 1 \
	--serial_device /dev/ttyUSB0 \
	--serial_baudrate 2000000 \
	--hil_mode 1 \
	--lockstep 0 \
	--output-file /tmp/ssrc_fog_x_hitl.sdf

echo "Spawning ssrc_fog_x_hitl"

gz model --spawn-file=/tmp/ssrc_fog_x_hitl.sdf --model-name=sadci01 -x 0.0 -y 0.0 -z 0.0
