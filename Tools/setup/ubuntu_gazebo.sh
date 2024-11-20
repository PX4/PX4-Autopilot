#! /usr/bin/env bash

set -e

## Bash script to setup Gazebo for PX4 simulation

if [ "$(id -u)" -ne 0 ]; then
        echo 'This script must be run by root' >&2
        exit 1
fi

UBUNTU_RELEASE="`lsb_release -rs`"

export DEBIAN_FRONTEND=noninteractive

# First install some necessary tools:
apt-get update
apt-get -y --quiet --no-install-recommends install \
	lsb-release \
	curl \
	gnupg \
	;

if [[ "${UBUNTU_RELEASE}" == "24.04" ]]; then

	apt-get -y --quiet --no-install-recommends install \
		cppzmq-dev \
		libunwind-dev \
		;
fi

echo "Installing Gazebo (Harmonic)"

curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

apt-get update
apt-get -y --quiet --no-install-recommends install \
	gz-harmonic \
	;
