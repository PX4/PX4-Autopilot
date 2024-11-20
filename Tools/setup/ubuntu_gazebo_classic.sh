#! /usr/bin/env bash

set -e

## Bash script to setup Gazebo classic for PX4 simulation

UBUNTU_RELEASE="`lsb_release -rs`"

if [[ "${UBUNTU_RELEASE}" == "20.04" ]]; then

	echo "Installing Gazebo classic"

	sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
	wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

	sudo apt-get update -y --quiet
	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		gazebo11 \
		gstreamer1.0-libav \
		gstreamer1.0-plugins-bad \
		gstreamer1.0-plugins-base \
		gstreamer1.0-plugins-good \
		gstreamer1.0-plugins-ugly \
		libeigen3-dev \
		libgazebo11-dev \
		libgstreamer-plugins-base1.0-dev \
		libimage-exiftool-perl \
		libopencv-dev \
		libxml2-utils \
		pkg-config \
		protobuf-compiler \
		;

elif [[ "${UBUNTU_RELEASE}" == "22.04" ]]; then

	sudo add-apt-repository ppa:openrobotics/gazebo11-gz-cli
	sudo apt-get update -y --quiet
	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install gazebo11

elif [[ "${UBUNTU_RELEASE}" == "24.04" ]]; then

	sudo add-apt-repository ppa:openrobotics/gazebo11-gz-cli
	sudo apt-get update -y --quiet
	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install gazebo11

else
	# otherwise try the one-liner (no promises)

	# First install some necessary tools:
	sudo apt-get update -y --quiet
	sudo DEBIAN_FRONTEND=noninteractive apt-get -y --quiet --no-install-recommends install \
		lsb-release \
		curl \
		gnupg \
		;

	# Install
	curl -sSL http://get.gazebosim.org | sh
fi
