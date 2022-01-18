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

source /opt/ros/galactic/setup.sh


# Remove old build output
rm -Rf build/px4_sitl_rtps

# Build
DONT_RUN=1 make px4_sitl_rtps gazebo_ssrc_fog_x

# tar artifacts
mkdir -p tmp_packing_dir
pushd tmp_packing_dir

mkdir -p px4_sitl/build/px4_sitl_rtps
mkdir -p px4_gazebo_data/plugins
mkdir -p px4_gazebo_data/models
mkdir -p px4_gazebo_data/worlds
mkdir -p px4_gazebo_data/scripts

find ../build/px4_sitl_rtps/build_gazebo/*.so -exec cp {} px4_gazebo_data/plugins \;

cp -r ../build/px4_sitl_rtps/bin                    px4_sitl/build/px4_sitl_rtps/bin
cp -r ../build/px4_sitl_rtps/etc                    px4_sitl/build/px4_sitl_rtps/etc
cp -r ../Tools/sitl_gazebo/models/asphalt_plane     px4_gazebo_data/models/asphalt_plane
cp -r ../Tools/sitl_gazebo/models/ground_plane      px4_gazebo_data/models/ground_plane
cp -r ../Tools/sitl_gazebo/models/sun               px4_gazebo_data/models/sun
cp -r ../Tools/sitl_gazebo/models/ssrc_fog_x        px4_gazebo_data/models/ssrc_fog_x
cp -r ../Tools/sitl_gazebo/worlds/empty.world       px4_gazebo_data/worlds/
cp -r ../Tools/sitl_gazebo/worlds/empty_ssrc.world  px4_gazebo_data/worlds/
cp -r ../Tools/sitl_gazebo/scripts/jinja_gen.py     px4_gazebo_data/scripts/
tar czvf ../px4_sitl_build-v${VERSION}.tar.gz px4_sitl
tar czvf ../px4_gazebo_data-v${VERSION}.tar.gz px4_gazebo_data

popd
rm -Rf tmp_packing_dir
