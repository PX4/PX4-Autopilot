#!/bin/bash

# toolchains available in voxl-cross 4
TOOLCHAIN_QRB5165_1_32="/opt/cross_toolchain/qrb5165_ubun1_18.04_arm32.toolchain.cmake"
TOOLCHAIN_QRB5165_1_64="/opt/cross_toolchain/qrb5165_ubun1_18.04_aarch64.toolchain.cmake"
TOOLCHAIN_QRB5165_2_64="/opt/cross_toolchain/qrb5165_ubun2_20.04_aarch64.toolchain.cmake"
TOOLCHAIN_QCS6490_64="/opt/cross_toolchain/qcs6490_ubun_24.04_aarch64.toolchain.cmake"

## this list is just for tab-completion
AVAILABLE_PLATFORMS="qrb5165 qrb5165-2 qcs6490 native arm32-only arm64-only"


print_usage(){
	echo ""
	echo " Build the current project based on platform target."
	echo ""
	echo " Usage:"
	echo ""
	echo "  ./build.sh qrb5165"
	echo "        Build 32 and 64-bit binaries for qrb5165 1.X system images"
	echo ""
	echo "  ./build.sh qrb5165-2"
	echo "        Build 64-bit binaries for qrb5165 2.X system images"
	echo ""
	echo "  ./build.sh qcs6490"
	echo "        Build 64-bit binaries for qcs6490 24.34 system images"
	echo ""
	echo "  ./build.sh native"
	echo "        Build with the native gcc/g++ compilers for testing code"
	echo "        locally on a desktop computer."
	echo ""
	echo ""
}


check_docker() {
	local MIN_VERSION="$1"
	local FILE="/etc/modalai/image.name"

	if [[ ! -f "$FILE" ]]; then
		echo "$FILE does not exist, are you running in the voxl-cross docker?"
		exit 1
	fi

	local IMAGE_STRING
	IMAGE_STRING=$(<"$FILE")

	if [[ "$IMAGE_STRING" =~ ^voxl-cross\(([0-9]+\.[0-9]+)\)$ ]]; then
		local VERSION="${BASH_REMATCH[1]}"
		echo "Found voxl-cross version: $VERSION"

		if [[ "$(printf '%s\n' "$VERSION" "$MIN_VERSION" | sort -V | head -n1)" == "$MIN_VERSION" ]]; then
			return 0
		else
			echo "voxl-cross $VERSION does not meet minimum required $MIN_VERSION."
			exit 1
		fi
	else
		echo "voxl-cross not found in $FILE"
		echo "are you running in the voxl-cross docker?"
		exit 1
	fi
}



case "$1" in

	qrb5165)
		## multi-arch library, build both 32 and 64
		check_docker "4.3"
		mkdir -p build32
		cd build32
		cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_QRB5165_1_32} -DBUILD_EXAMPLES=OFF ../
		make -j$(nproc)
		cd ../
		mkdir -p build64
		cd build64
		cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_QRB5165_1_64} ../
		make -j$(nproc)
		cd ../
		;;

	qrb5165-2)
		check_docker "4.3"
		mkdir -p build
		cd build
		cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_QRB5165_2_64} ../
		make -j$(nproc)
		cd ../
		;;

	qcs6490)
		check_docker "4.3"
		mkdir -p build
		cd build
		cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_QCS6490_64} ../
		make -j$(nproc)
		cd ../
		;;

	arm32-only)
		## 32-bit only for testing
		check_docker "4.3"
		mkdir -p build32
		cd build32
		cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_QRB5165_1_32} ../
		make -j$(nproc)
		cd ../
		;;

	arm64-only)
		## 64-bit only for testing
		check_docker "4.3"
		mkdir -p build64
		cd build64
		cmake -DCMAKE_TOOLCHAIN_FILE=${TOOLCHAIN_QRB5165_1_64} ../
		make -j$(nproc)
		cd ../
		;;

	native)
		mkdir -p build
		cd build
		cmake ${EXTRA_OPTS} ../
		make -j$(nproc)
		cd ../
		;;

	*)
		print_usage
		exit 1
		;;
esac



