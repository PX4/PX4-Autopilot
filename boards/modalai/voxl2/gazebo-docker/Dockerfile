FROM ros:noetic-perception-focal

USER root

RUN apt-get update && apt-get -y install sudo
RUN apt-get install --fix-missing
RUN apt-get install -y apt-utils
RUN apt-get install -y ca-certificates
RUN apt-get install -y git vim
RUN apt-get install dialog apt-utils -y
RUN apt-get install net-tools -y
RUN apt-get install inetutils-ping -y
RUN echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections
RUN sudo apt-get install -y -q
RUN apt-get install -y gcc g++ cmake git
RUN apt-get update
RUN apt-get install -y python3-pip
RUN apt-get install -y curl
RUN apt-get install -y software-properties-common
RUN apt-get install -y patch

WORKDIR /root

RUN git clone --branch v1.14.0 https://github.com/PX4/PX4-Autopilot.git --recursive
WORKDIR /root/PX4-Autopilot

COPY patch/mavlink_interface.patch .
COPY patch/iris_hitl.patch .
COPY patch/iris_vision.patch .

RUN mv mavlink_interface.patch Tools/simulation/gazebo-classic/sitl_gazebo-classic/src
RUN mv iris_hitl.patch Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_hitl
RUN mv iris_vision.patch Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_vision

WORKDIR /root/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/src
RUN patch < mavlink_interface.patch
RUN rm -rf mavlink_interface.patch

WORKDIR /root/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_hitl
RUN patch < iris_hitl.patch
RUN rm -rf iris_hitl.patch

WORKDIR /root/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_vision
RUN patch < iris_vision.patch
RUN rm -rf iris_vision.patch

RUN ./Tools/setup/ubuntu.sh
RUN DONT_RUN=1 make px4_sitl_default gazebo
