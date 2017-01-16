#!/bin/bash

while getopts "b:t:" opt; do
  case "${opt}" in
    b)
      builddir=$OPTARG
      ;;
    t)
      tool=$OPTARG
      ;;
  esac
done

echo "builddir = ${builddir}, tool = ${tool}"

case "${builddir}" in
  "build_posix_rpi_cross")
    CXX_INC=$(cd ${RPI_TOOLCHAIN_DIR}/gcc-linaro-arm-linux-gnueabihf-raspbian/arm-linux-gnueabihf/include/c++/*; pwd)
    EXTRA_ARG1=-I${CXX_INC}
    EXTRA_ARG2=-I${CXX_INC}/arm-linux-gnueabihf
    EXTRA_ARG3=-I${CXX_INC}/backward
    extra_args="--extra-arg=-I${CXX_INC} --extra-arg=-I${CXX_INC}/arm-linux-gnueabihf --extra-arg=-I${CXX_INC}/backward"
    ;;
  "build_posix_sitl_default")
    ;;
  *)
    echo "unknown build dir: ${builddir}"
    ;;
esac

COMPILE_DB=$(/bin/pwd)/${builddir}
if [[ ! -f ${COMPILE_DB}/compile_commands.json ]]; then
  echo "compile_commands.json not found in ${COMPILE_DB}"
  exit 1
fi

case "${tool}" in
  "clang-check")
    command=clang-check;
    option=-analyze;
    ;;
  "clang-tidy")
    command=clang-tidy
    #option=-fix
    ;;
esac

failed=0
while read line; do
	file_line=$(echo $line | grep \"file\")
	if [ $? -eq 0 ]; then
		file_path=$(echo $file_line | awk '{ print $2; }' | sed 's/\"//g')

		echo ${file_path}
		${command} ${option} -p ${COMPILE_DB} ${extra_args} ${file_path}

		if [ $? -ne 0 ]; then
			failed=1
		fi
		echo
	fi
done <${COMPILE_DB}/compile_commands.json

if [ $failed -ne 0 ]; then
    exit 1
fi
