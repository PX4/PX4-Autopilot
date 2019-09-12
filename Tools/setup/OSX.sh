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

# install Homebrew
/usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)" < /dev/null

# confirm Homebrew installed correctly
brew doctor

# Install px4-dev formula
echo
echo "Installing PX4 general dependencies (homebrew px4-dev)"

if [[ $REINSTALL_FORMULAS == "--reinstall" ]]; then
  brew tap PX4/px4
  brew reinstall px4-dev
elif brew ls --versions px4-dev > /dev/null; then
  brew tap PX4/px4
  brew install px4-dev
fi

# Python3 dependencies
echo
echo "Installing PX4 Python3 dependencies"
brew install python3
sudo -H python3 -m pip install --upgrade pip
sudo -H python3 -m pip install -r ${DIR}/requirements.txt

# Optional, but recommended additional simulation tools:
if [[ $INSTALL_SIM == "--sim-tools" ]]; then
  if brew ls --versions px4-sim > /dev/null; then
    brew install px4-sim
  elif [[ $REINSTALL_FORMULAS == "--reinstall" ]]; then
    brew reinstall px4-sim
  fi
fi

echo "All set! PX4 toolchain installed!"
