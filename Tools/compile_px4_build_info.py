# combine multiple json files to one file
# takes as an input
# 1: output json filename
# 2: px4 directory from there the px4 json files are collected
# 3: fpga json file compiled by collect_json_files.py
# 4: build info file compiled by generate_basic_build_info.sh

# This has to be ran after all px4 builds are done because they are ran in parallel on separate environments
# combine all px4 json files, fpga json file and basic build info to one file
# the result is the final saluki_build_info.json file which get packaged to px4 container

import sys
import os
import json
import glob


# Check if input is provided for output file
if len(sys.argv) < 5:
  print("usage: {} [output_file.json] [px4_json_directory] [fpga_json_file] [build_info_file]".format(sys.argv[0]))
  sys.exit(1)

output_file = sys.argv[1]
px4_json_directory = sys.argv[2]
fpga_json_file = sys.argv[3]
build_info_file = sys.argv[4]

# Collect all px4 JSON files
px4_json_files = glob.glob(os.path.join(px4_json_directory, "*.json"))
px4_json_data = []

# collect px4 file info to array
for file in px4_json_files:
  print("{}: processing file: {}".format(sys.argv[0], file))

  with open(file, 'r') as f:
    temp_data = json.load(f)
    # append "filename" with "/firmware" prefix
    temp_data["filename"] = "/firmware/" + temp_data["filename"]
    px4_json_data.append(temp_data)

print("{}: collected px4 json data".format(sys.argv[0], file))
print(px4_json_data)

# read fpga json file
with open(fpga_json_file, 'r') as f:
  fpga_json_data = json.load(f)
  # add fpga-files array to files array
  fpga_json_data['files'] = fpga_json_data.pop('fpga-files')
  # add bl-files array to files array
  fpga_json_data['files'].extend(fpga_json_data.pop('bl-files'))
  # add sha field to fpga_sha
  fpga_json_data['fpga_sha'] = fpga_json_data.pop('sha', '')
  # add build_url field to fpga_build_url
  fpga_json_data['fpga_build_url'] = fpga_json_data.pop('build_url', '')
  # add reponame field to fpga_reponame
  fpga_json_data['fpga_reponame'] = fpga_json_data.pop('reponame', '')

  print("{}: fpga json data".format(sys.argv[0], file))
  print(json.dumps(fpga_json_data, indent=4, sort_keys=True))

# fpga json files to px4 json data
px4_json_data.extend(fpga_json_data.pop('files'))

# combine all json data into one
json_data = {}
json_data = {**json_data, **fpga_json_data}

# add file array from combined json data
json_data["files"] = px4_json_data

# combine basic build info from build_info_file
with open(build_info_file, 'r') as f:
  build_info_json_data = json.load(f)
  json_data = {**json_data, **build_info_json_data}

print("{}: this is the combined json data".format(sys.argv[0], file))
print(json.dumps(json_data, indent=4, sort_keys=True))

# write the json_data to the output file sorted by keys
with open(output_file, 'w') as outfile:
  print("{}: writing to file: {}".format(sys.argv[0], output_file))
  json.dump(json_data, outfile, indent=4, sort_keys=True)
