#
# PX4 ROS SITL
#

FROM px4ros/ros-base:no-drcsim
MAINTAINER Andreas Antener "andreas@antener.name"

# TODO
#USER px4

RUN . /opt/ros/indigo/setup.sh \
	&& mkdir -p /catkin_ws/src \
	&& cd /catkin_ws/src \
	&& catkin_init_workspace \
	&& cd /catkin_ws \
	&& catkin_make \
	&& echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

COPY get-sources.sh /catkin_ws/src/
RUN chmod +x /catkin_ws/src/get-sources.sh

CMD /bin/bash
