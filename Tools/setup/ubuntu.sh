#! /usr/bin/env bash


# detect if running in docker
if [ -f /.dockerenv ]; then
	echo "Running within docker, installing initial dependencies";
	apt-get --quiet -y update && apt-get --quiet -y install \
		ca-certificates \
		curl \
		gnupg \
		gosu \
		lsb-core \
		sudo \
		wget \
		;
fi

# script directory
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# check ubuntu version
# instructions for 16.04, 18.04
# otherwise warn and point to docker?
UBUNTU_RELEASE=`lsb_release -rs`

if [[ "${UBUNTU_RELEASE}" == "14.04" ]]
then
	echo "Ubuntu 14.04 unsupported, see docker px4io/px4-dev-base"
	exit 1
elif [[ "${UBUNTU_RELEASE}" == "16.04" ]]
then
	echo "Ubuntu 16.04"
elif [[ "${UBUNTU_RELEASE}" == "18.04" ]]
then
	echo "Ubuntu 18.04"
fi

export DEBIAN_FRONTEND=noninteractive

sudo apt-get update -yy --quiet
sudo apt-get -yy --quiet --no-install-recommends install \
	astyle \
	bzip2 \
	ccache \
	cmake \
	cppcheck \
	doxygen \
	file \
	g++ \
	gcc \
	gdb \
	git \
	lcov \
	make \
	ninja-build \
	python-pip \
	python-pygments \
	python-setuptools \
	rsync \
	shellcheck \
	unzip \
	wget \
	xsltproc \
	zip

# python dependencies
if [ -f /.dockerenv ]; then
	# in docker install requirements system wide
	sudo python -m pip install --upgrade pip setuptools wheel
	sudo python -m pip install -r ${DIR}/requirements.txt
else
	# otherwise only install for the user
	python -m pip install --user --upgrade pip setuptools wheel
	python -m pip install --user -r ${DIR}/requirements.txt
fi

# java (jmavsim or fastrtps)
# TODO: only install when necessary
sudo apt-get -y --quiet --no-install-recommends install \
	default-jre-headless \
	default-jdk-headless

# TODO: nuttx, raspberrypi, armhf generic

# TODO: gazebo or ROS optional
