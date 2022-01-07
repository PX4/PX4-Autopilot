#!/bin/bash

# usage: ./package.sh path_to_firmware dest_dir

set -e

if [ "$1" = "" ]; then
	echo "ERROR: PX4 firmware source directory not given"
	echo " usage: $0 <px4-fw-source-dir> <output-dir>"
	exit 1
elif [ "$2" = "" ]; then
	echo "ERROR: Package output directory not given"
	echo " usage: $0 <px4-fw-source-dir> <output-dir>"
	exit 1
fi

fw_dir=$(realpath $1)
script_dir=$(dirname $(realpath $0))
dest_dir=$(realpath $2)


build_px4() {
	pushd ${fw_dir}

	# Copy Dockerfile and tools
	cp -f $script_dir/Dockerfile .

	# Generate debian package
	iname=tii-px4-pixhawk-artifacts
	docker build -t ${iname} .
	container_id=$(docker create ${iname} "")
	mkdir -p tmp_
	pushd tmp_
	docker cp ${container_id}:/artifacts .
	docker rm ${container_id}
	mkdir -p ${packaging_dir}/opt/px4fwupdater/
	cp artifacts/* ${packaging_dir}/opt/px4fwupdater/
	popd
	rm -Rf tmp_
	popd
}

build_saluki() {
	pushd ${fw_dir}

	# Copy Dockerfile and tools
	cp -f $script_dir/Dockerfile.saluki .

	# Generate debian package
	iname=tii-px4-saluki-artifacts
	docker build -f Dockerfile.saluki -t ${iname} .
	container_id=$(docker create ${iname} "")
	mkdir -p tmp_
	pushd tmp_
	docker cp ${container_id}:/artifacts .
	docker rm ${container_id}
	mkdir -p ${packaging_dir}/opt/px4fwupdater/
	cp artifacts/* ${packaging_dir}/opt/px4fwupdater/
	popd
	rm -Rf tmp_
	popd
}


make_deb() {
	echo "Creating deb package..."
	mkdir ${packaging_dir}/DEBIAN/

	cp ${script_dir}/debian/control ${packaging_dir}/DEBIAN/
	cp ${script_dir}/debian/postinst ${packaging_dir}/DEBIAN/
	cp ${script_dir}/debian/prerm ${packaging_dir}/DEBIAN/
	cp ${script_dir}/px4_update_fw.sh ${packaging_dir}/opt/px4fwupdater/px4_update_fw.sh
	cp ${script_dir}/detect_ttyS.sh ${packaging_dir}/opt/px4fwupdater/detect_ttyS.sh

	pushd ${fw_dir}
	version=$(git describe --always --tags --dirty | sed 's/^v//')
	popd
	sed -i "s/Version:.*/&${version}/" ${packaging_dir}/DEBIAN/control

	# move elf files from deb
	for elf in ${packaging_dir}/opt/px4fwupdater/*.elf
	do
		elf_out_file=$(basename $elf | sed "s/\(.*\)\.elf/\1-${version}.elf/")
		mv $elf ${dest_dir}/${elf_out_file}
	done

	cat ${packaging_dir}/DEBIAN/control
	echo px4fwupdater_${version}_amd64.deb
	fakeroot dpkg-deb --build ${packaging_dir} ${dest_dir}/px4fwupdater_${version}_amd64.deb

	for px4 in ${packaging_dir}/opt/px4fwupdater/*.px4
	do
		px4_out_file=$(basename $px4 | sed "s/\(.*\)\.px4/\1-${version}.px4/")
		cp $px4 ${dest_dir}/${px4_out_file}
	done

}

mkdir -p ${dest_dir}
packaging_dir=$(mktemp -d)
build_px4
build_saluki
make_deb
rm -rf ${packaging_dir}
echo "Done"
