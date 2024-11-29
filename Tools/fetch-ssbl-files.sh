#!/bin/bash -eu

# this script fetches second stage bootloader files from a docker container
# ssbl files are not necessary available so it might not extract anything

# input parameters:
# 1: bootloader repo url
# 2: PRODUCT
# 3: output dir

if [ $# -ne 3 ]; then
  echo "Error: ${0} requires 3 parameters"
  echo "Usage: ${0} <repo url> <PRODUCT> <output dir>"
  exit 1
fi

BL_REPO_URL="${1:-}"
PRODUCT="${2:-}"
OUTPUT_DIR="${3:-}"
TMP_CONTAINER_NAME=tmp_bootloader_container_$(date +%s)
SALUKI_FILE_INFO="saluki_file_info.json"
FILEINFO_JSON_OUT=${OUTPUT_DIR}/${SALUKI_FILE_INFO}

function cleanup_and_exit {
    docker stop ${TMP_CONTAINER_NAME}
    docker rm ${TMP_CONTAINER_NAME}
    exit 0
}

# run cleanup always on exit
trap cleanup_and_exit EXIT

echo "fetching bootloader files from ${BL_REPO_URL}"

docker run -d --pull always --name ${TMP_CONTAINER_NAME} ${BL_REPO_URL}

# try to get the build info json
mkdir -p ${OUTPUT_DIR}
docker cp ${TMP_CONTAINER_NAME}:/${SALUKI_FILE_INFO} ${FILEINFO_JSON_OUT} || true

# if saluki build info doesnt exist in bootloader, it has to be not split version
if [ ! -f ${FILEINFO_JSON_OUT} ]; then
	echo "bootloader not split version"
    exit 0
else
	echo "using info from ${FILEINFO_JSON_OUT}"
	# find out if the 2 stage bootloader files are available
	# files should have stage defined: fileinfo_json files[] -> stage
	PRODUCT=$(echo ${PRODUCT}|sed 's/_/-/g')
	filename=$(jq '.files[] | select(.hw=="'${PRODUCT}'" and .stage=="ssbl").filename' ${FILEINFO_JSON_OUT}|sed 's/"//g')

    tmp_bl_reponame=$(jq '.reponame' ${FILEINFO_JSON_OUT})
    tmp_bl_sha=$(jq '.sha' ${FILEINFO_JSON_OUT})
    tmp_bl_build_url=$(jq '.build_url' ${FILEINFO_JSON_OUT})

    # filter files for this product and append $output_dir to .files[].filename
    # file path is appended because the files are copied to a different directory so it will match the fileinfo.json
    tmp_files=$(jq '.files[] | select(.hw=="'${PRODUCT}'" and .stage=="ssbl") | .filename = "/firmware/'${OUTPUT_DIR}'"+.filename' ${FILEINFO_JSON_OUT}|jq -s .)

    if [ -z "${tmp_files}" ]; then
        echo "no ssbl files found"
        exit 0
    fi

    tmp_json="{\"${PRODUCT}_bootloader_reponame\":${tmp_bl_reponame},\"${PRODUCT}_bl_sha\":${tmp_bl_sha},\"${PRODUCT}_bl_build_url\":${tmp_bl_build_url},\"bl-files\":${tmp_files}}"
    echo $(echo $tmp_json|jq .)
    echo $tmp_json|jq . > ${FILEINFO_JSON_OUT}

    # copy each file
    echo "make target dir (dirname of) ${OUTPUT_DIR}/${filename}"
    mkdir -p $(dirname ${OUTPUT_DIR}/${filename})
    echo "copying ${filename} to ${OUTPUT_DIR}/${filename}"
    docker cp ${TMP_CONTAINER_NAME}:${filename} ${OUTPUT_DIR}/${filename}
fi
