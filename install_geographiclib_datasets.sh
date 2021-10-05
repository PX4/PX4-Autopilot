#!/bin/bash
# Script to install the model datasets required
# to GeographicLib apply certain conversions

if [[ $UID != 0 ]]; then
	echo "This script require root privileges!" 1>&2
	exit 1
fi

# Install datasets
run_get() {
	local dir="$1"
	local tool="$2"
	local model="$3"

	files=$(shopt -s nullglob dotglob; echo /usr/share/GeographicLib/$dir/$model* /usr/local/share/GeographicLib/$dir/$model*)
	if (( ${#files} )); then
		echo "GeographicLib $tool dataset $model already exists, skipping"
		return
	fi

	echo "Installing GeographicLib $tool $model"
	geographiclib-get-$tool $model >/dev/null 2>&1
	
	files=$(shopt -s nullglob dotglob; echo /usr/share/GeographicLib/$dir/$model* /usr/local/share/GeographicLib/$dir/$model*)
	if (( ! ${#files} )); then
		echo "Error while installing GeographicLib $tool $model"
		return
	fi
}

# check which command script is available
if hash geographiclib-get-geoids; then
	run_get geoids geoids egm96-5
	run_get gravity gravity egm96
	run_get magnetic magnetic emm2015
elif hash geographiclib-datasets-download; then # only allows install the goid model dataset
	geographiclib-datasets-download egm96_5;
else
	echo "OS not supported! Check GeographicLib page for supported OS and lib versions." 1>&2
fi
