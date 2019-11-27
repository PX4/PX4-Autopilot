#! /usr/bin/env bash

# script directory
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# Reinstall if --reinstall set
REINSTALL_FORMULAS=""
# Install simulation tools?
INSTALL_SIM=""

# Parse arguments
for arg in "$@"
do
  if [[ $arg == "--reinstall" ]]; then
    REINSTALL_FORMULAS=$arg
  elif [[ $arg == "--sim-tools" ]]; then
    INSTALL_SIM=$arg
  fi
done

# Install px4-dev formula
brew tap PX4/px4
if [ brew ls --versions px4-dev > /dev/null ]; then
  brew install px4-dev
elif [[ $REINSTALL_FORMULAS == "--reinstall" ]]; then
  brew reinstall px4-dev
fi

# Python dependencies
sudo easy_install pip
sudo -H python3 -m pip install --upgrade --force-reinstall pip
sudo -H python3 -m pip install -I -r ${DIR}/requirements.txt

# Optional, but recommended additional simulation tools:
if [[ $INSTALL_SIM == "--sim-tools" ]]; then
  if [ brew ls --versions px4-sim > /dev/null ]; then
    brew install px4-sim
  elif [[ $REINSTALL_FORMULAS == "--reinstall" ]]; then
    brew reinstall px4-sim
  fi
fi

echo "All set! PX4 toolchain installed!"
