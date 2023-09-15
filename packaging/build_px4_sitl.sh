set -eo pipefail

usage() {
	echo "
Usage: $(basename "$0") [-h] [-v version]
 -- Generate tar packages from px4_sitl module.
Params:
    -h  Show help text.
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
VERSION=""

while getopts "hv:" opt
do
	case $opt in
		h)
			usage
			;;
		v)
			check_arg $OPTARG && VERSION=$OPTARG || error_arg $opt
			;;
		\?)
			usage
			;;
	esac
done

source /opt/ros/humble/setup.sh


# Remove old build output
rm -Rf build/px4_sitl_default

# Build
make px4_sitl_default

# tar artifacts
mkdir -p tmp_packing_dir
pushd tmp_packing_dir

mkdir -p px4_sitl/build/px4_sitl_default

cp -r ../build/px4_sitl_default/bin                                   px4_sitl/bin
cp -r ../build/px4_sitl_default/etc                                   px4_sitl/etc
tar czvf ../px4_sitl_build-v${VERSION}.tar.gz px4_sitl

popd
rm -Rf tmp_packing_dir
