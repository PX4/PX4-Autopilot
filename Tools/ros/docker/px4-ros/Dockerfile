#
# Base ROS Indigo
#

FROM ubuntu:14.04.1
MAINTAINER Andreas Antener "andreas@antener.name"

# Install basics
## Use the "noninteractive" debconf frontend
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
	&& apt-get -y install wget git mercurial openssh-server

# Main ROS Setup
# Following http://wiki.ros.org/indigo/Installation/Ubuntu
# Also adding dependencies for gazebo http://gazebosim.org/tutorials?tut=drcsim_install

## add ROS repositories and keys
## install main ROS pacakges
RUN echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list \
	&& wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add - \
	&& apt-get update \
	&& apt-get -y install ros-indigo-desktop-full

RUN rosdep init \
	&& rosdep update

## setup environment variables
RUN echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc

## get rosinstall
RUN apt-get -y install python-rosinstall

## additional dependencies
RUN apt-get -y install ros-indigo-octomap-msgs


## install drcsim
#RUN echo "deb http://packages.osrfoundation.org/drc/ubuntu trusty main" > /etc/apt/sources.list.d/drc-latest.list \
#	&& wget http://packages.osrfoundation.org/drc.key -O - | apt-key add - \
#	&& apt-get update \
#	&& apt-get -y install drcsim

# some QT-Apps/Gazebo don't not show controls without this
ENV QT_X11_NO_MITSHM 1

#RUN apt-get install -y openssh-server
# Install x11-utils to get xdpyinfo, for X11 display debugging
# mesa-utils provides glxinfo, handy for understanding the 3D support.
#RUN apt-get -y install x11-utils
#RUN apt-get -y install mesa-utils

#CMD ["/bin/sbin/sshd", "-D"]
