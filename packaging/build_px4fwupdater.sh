set -eo pipefail

usage() {
	echo "
Usage: $(basename "$0") [-h] [-v version]
 -- Generate debian package from px4fwupdater module.
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
indir=""

while getopts "hv:i:" opt
do
	case $opt in
		h)
			usage
			;;
		v)
			check_arg $OPTARG && VERSION=$OPTARG || error_arg $opt
			;;
		i)
			indir=$OPTARG
			;;
		\?)
			usage
			;;
	esac
done

set -x

if [ -e tmp_packaging_dir ]; then
	rm -Rf tmp_packaging_dir
fi

mkdir -p tmp_packaging_dir
pushd tmp_packaging_dir

mkdir -p ./opt/px4fwupdater/
if [ -n "${indir}" ]
then
	cp ${indir}/*.px4 ./opt/px4fwupdater/
else
	cp ${mod_dir}/build/ssrc_saluki-v1_default/*.px4 ./opt/px4fwupdater/
	cp ${mod_dir}/build/px4_fmu-v5_ssrc/*.px4 ./opt/px4fwupdater/
	cp ${mod_dir}/build/px4_fmu-v5x_ssrc/*.px4 ./opt/px4fwupdater/
fi

mkdir DEBIAN
cp -R ${mod_dir}/packaging/debian/* DEBIAN/
cp ${mod_dir}/packaging/px4_update_fw.sh  ./opt/px4fwupdater/px4_update_fw.sh
cp ${mod_dir}/packaging/detect_ttyS.sh    ./opt/px4fwupdater/detect_ttyS.sh
cp ${mod_dir}/Tools/px_uploader.py        ./opt/px4fwupdater/px_uploader.py

if [ -e ../px4fwupdater*.deb ]; then
	rm -f ../px4fwupdater*.deb
fi

sed -i "s/Version:.*/&${VERSION}/" DEBIAN/control
fakeroot dpkg-deb --build . "${indir}"/px4fwupdater_${VERSION}_amd64.deb

popd
rm -Rf tmp_packaging_dir
