#!/bin/bash -eu

# input parameters:
# 1: repo url
# 2: product
# 3: output dir
if [ -z ${3+x} ]; then
  echo "Error: ${0} requires 3 parameters"
  echo "Usage: ${0} <repo url> <product> <output dir>"
  exit 1
fi

saluki_v2_fpga_container="${1:-}"
product="${2:-}"
output_dir="${3:-}"
fileinfo_json="saluki_file_info.json"
fileinfo_json_out=${output_dir}/${fileinfo_json}

echo "fetching fpga files from ${saluki_v2_fpga_container}"

mkdir -p ${output_dir}

# run temporary container to copy the files
container_name=fpga_tmp_container_$(date +%s)
docker run -d --pull always --name ${container_name} ${saluki_v2_fpga_container}

# try to get the build info json
docker cp ${container_name}:/${fileinfo_json} ${fileinfo_json_out} || true

# legacy: if saluki build info doesnt exist, copy the whole /firmware directory
if [ ! -f ${fileinfo_json_out} ]; then
	echo "Using legacy method to copy the whole /firmware directory"
	docker cp ${container_name}:/firmware ${output_dir}/
else
	echo "using info from ${fileinfo_json_out}"
	# find out info from json
	# files to be used from fileinfo_json files[] -> filename
	#product=$(echo ${product}|sed 's/_/-/g')
	filename=$(jq -r '.files[] | select(.hw=="'${product}'").filename' ${fileinfo_json_out})

  echo "filet: ${filename}"

  tmp_reponame=$(jq '.reponame' ${fileinfo_json_out})
  tmp_sha=$(jq '.sha' ${fileinfo_json_out})
  tmp_build_url=$(jq '.build_url' ${fileinfo_json_out})

  # filter files for this product and append $output_dir to .files[].filename
  # file path is appended because the files are copied to a different directory so it will match the fileinfo.json
  tmp_files=$(jq '.files[] | select(.hw=="'${product}'") | .filename = "/firmware/'${output_dir}'"+.filename' ${fileinfo_json_out}|jq -s .)

  tmp_json="{\"${product}_fpga_reponame\":${tmp_reponame},\"${product}_fpga_sha\":${tmp_sha},\"${product}_fpga_build_url\":${tmp_build_url},\"fpga-files\":${tmp_files}}"
  echo $tmp_json
  echo $(echo $tmp_json|jq .)
  echo $tmp_json|jq . > ${fileinfo_json_out}

  for file in ${filename[@]}; do
    # copy each file
    echo "make target dir (dirname of) ${output_dir}/${file}"
    mkdir -p $(dirname ${output_dir}/${file})
    echo "copying ${file} to ${output_dir}/${file}"
    docker cp ${container_name}:${file} ${output_dir}/${file}
  done
fi

docker stop ${container_name}
docker rm ${container_name}
