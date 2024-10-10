#! /usr/bin/env bash

# script directory
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# store the project root directory path in a variable
PX4_DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )/../../" && pwd )

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

if ! command -v brew &> /dev/null
then
	# install Homebrew if not installed yet
	/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)"
fi

# Install px4-dev formula
if [[ $REINSTALL_FORMULAS == "--reinstall" ]]; then
	echo "Re-installing PX4 general dependencies (homebrew px4-dev)"

	# confirm Homebrew installed correctly
	brew doctor

	brew tap PX4/px4
	brew reinstall px4-dev
	brew install ncurses
	brew install python-tk
else
	if brew ls --versions px4-dev > /dev/null; then
		echo "px4-dev already installed"
	else
		echo "Installing PX4 general dependencies (homebrew px4-dev)"
		brew tap PX4/px4
		brew install px4-dev
		brew install ncurses
		brew install python-tk
	fi
fi

# Python dependencies
echo
echo "Installing PX4 Python3 dependencies"
if [ ! -d ".venv" ]; then
	echo "Python venv not found. Creating a new virtual vnvironment at .venv"
	python3 -m venv ${PX4_DIR}/.venv
fi
activate () {
  . $PX4_DIR/.venv/bin/activate
}
echo "Activating Python virtual environment"
activate
python -m pip install -r ${DIR}/requirements.txt


# Optional, but recommended additional simulation tools:
if [[ $INSTALL_SIM == "--sim-tools" ]]; then
	if brew ls --versions px4-sim > /dev/null; then
		brew install px4-sim
	elif [[ $REINSTALL_FORMULAS == "--reinstall" ]]; then
		brew reinstall px4-sim
	fi
fi

#deactivate venv
# deactivate

echo "All set! PX4 toolchain installed!"
