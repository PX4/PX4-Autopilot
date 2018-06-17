#! /usr/bin/env bash

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
	echo "WARNING, instructions only tested on Ubuntu 16.04"
fi

export DEBIAN_FRONTEND=noninteractive
sudo apt-get update -yy --quiet
sudo apt-get -yy --quiet --no-install-recommends install \
	bzip2 \
	ca-certificates \
	ccache \
	cmake \
	g++ \
	gcc \
	git \
	lcov \
	make \
	ninja-build \
	python-pip
	rsync \
	unzip \
	wget \
	wget \
	xsltproc \
	zip

# python dependencies
python -m pip install --user --upgrade pip setuptools wheel
python -m pip install --user -r ${DIR}/requirements.txt

# java (jmavsim or fastrtps)
# TODO: only install when necessary
sudo apt-get -y --quiet --no-install-recommends install \
	default-jre-headless \
	default-jdk-headless \
