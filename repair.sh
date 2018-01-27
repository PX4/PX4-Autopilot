# $1 = find px4
# $2 = vehicle
# $3 = mavlink_udp_port
# $4 = ID
source /opt/ros/kinetic/setup.bash
rosrun xacro xacro $1/Tools/sitl_gazebo/models/rotors_description/urdf/$2_base.xacro rotors_description_dir:=$1/Tools/sitl_gazebo/models/rotors_description mavlink_udp_port:=$3 > $2_$4.urdf
