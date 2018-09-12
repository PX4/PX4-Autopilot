#! /usr/bin/env bash

# script directory
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

brew tap PX4/px4
brew install px4-dev

# python dependencies
sudo easy_install pip
sudo -H pip install -r ${DIR}/requirements.txt

# Optional, but recommended additional simulation tools:
brew install px4-sim
