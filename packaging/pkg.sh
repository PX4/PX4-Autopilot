#!/bin/bash

set -eo pipefail

usage() {
	echo "
Usage: $(basename "$0") [-h] [-b nbr] [-d dist]
 -- Generate debian package from fog_sw module.
Params:
    -h  Show help text.
    -b  Build number. This will be tha last digit of version string (x.x.N).
    -d  Distribution string in debian changelog.
		-g  Git commit hash.
		-v  Git version string
"
	exit 0
}

check_arg() {
	if [ "$(echo $1 | cut -c1)" = "-" ]; then
		return 1
	else
		return 0
	fi
}

error_arg() {
	echo "$0: option requires an argument -- $1"
	usage
}

mod_dir="$(realpath $(dirname $0)/..)"
build_nbr=0
distr=""
version=""
git_commit_hash=""
git_version_string=""

while getopts "hb:d:g:v:" opt
do
	case $opt in
		h)
			usage
			;;
		b)
			check_arg $OPTARG && build_nbr=$OPTARG || error_arg $opt
			;;
		d)
			check_arg $OPTARG && distr=$OPTARG || error_arg $opt
			;;
		g)
			check_arg $OPTARG && git_commit_hash=$OPTARG || error_arg $opt
			;;
		v)
			check_arg $OPTARG && git_version_string=$OPTARG || error_arg $opt
			;;
		\?)
			usage
			;;
	esac
done

if [[ "$git_commit_hash" == "0" || -z "$git_commit_hash" ]]; then
	git_commit_hash="$(git rev-parse HEAD)"
fi
if [[ "$git_version_string" == "0" || -z "$git_version_string" ]]; then
	git_version_string="$(git log --date=format:%Y%m%d --pretty=~git%cd.%h -n 1)"
fi

## Remove trailing '/' mark in module dir, if exists
mod_dir=$(echo $mod_dir | sed 's/\/$//')

## Debug prints
echo 
echo "[INFO] mod_dir: ${mod_dir}."
echo "[INFO] build_nbr: ${build_nbr}."
echo "[INFO] distr: ${distr}."
echo "[INFO] git_commit_hash: ${git_commit_hash}."
echo "[INFO] git_version_string: ${git_version_string}."

cd $mod_dir

## Generate package
echo "[INFO] Creating deb package..."
### ROS2 Packaging

### Create version string
version=$(grep "<version>" package.xml | sed 's/[^>]*>\([^<"]*\).*/\1/')

echo "[INFO] Version: ${version}."

#title="$version ($(date +%Y-%m-%d))"
#cat << EOF_CHANGELOG > CHANGELOG.rst
#$title
#$(printf '%*s' "${#title}" | tr ' ' "-")
#* commit: ${git_commit_hash}
#EOF_CHANGELOG

if [ -e ${mod_dir}/ros2_ws ]; then
	# From fog-sw repo.
	source ${mod_dir}/ros2_ws/install/setup.bash
fi
if [ -e ${mod_dir}/../deps_ws ]; then
	source ${mod_dir}/../deps_ws/install/setup.bash
fi

if [ -e ${mod_dir}/debian ]; then
	cp -r debian debian_bak
fi

bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro ${ROS_DISTRO} --place-template-files \
    && sed -i "s/@(DebianInc)@(Distribution)/@(DebianInc)/" debian/changelog.em \
    && [ ! "$distr" = "" ] && sed -i "s/@(Distribution)/${distr}/" debian/changelog.em || : \
    && bloom-generate rosdebian --os-name ubuntu --os-version focal --ros-distro ${ROS_DISTRO} --process-template-files -i ${build_nbr}${git_version_string} \
    && sed -i 's/^\tdh_shlibdeps.*/& --dpkg-shlibdeps-params=--ignore-missing-info/g' debian/rules \
		&& sed -i "s/\=\([0-9]*\.[0-9]*\.[0-9]*\*\)//g" debian/control \
    && fakeroot debian/rules clean \
    && fakeroot debian/rules binary || exit 1

echo "[INFO] Clean up."

rm -rf obj-x86_64-linux-gnu debian

if [ -e ${mod_dir}/debian_bak ]; then
	cp -r debian_bak debian
	rm -rf debian_bak
fi


echo "[INFO] Move debian packages to volume."
mv ${mod_dir}/../*.deb ${mod_dir}/../*.ddeb ${mod_dir}

echo "[INFO] Done."
exit 0
