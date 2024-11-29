# combine all fpga json files to one file
# this has to be done because each fpga target has their own json file and we need to
# save the info. Each fpga json file includes info about fpga and bootloader (sha,
# link to build, filename, hw, etc.) and this has to be passed to px4 json file.

# as an input takes
# 1: fpga directory which contains json files
# 2: an output filename

import sys
import os
import json

# Check if input is provided for directory and output file
if len(sys.argv) < 3:
    print("usage: {} [dir] [output_file.json]".format(sys.argv[0]))
    sys.exit(1)

# Collect all JSON files from the specified directory and process one by one
json_files = []
for root, dirs, files in os.walk(sys.argv[1]):
    for file in files:
        if file.endswith(".json"):
            json_files.append(os.path.join(root, file))

json_data = {}
for file in json_files:
    print("{}: found file: {}".format(sys.argv[0], file))

    with open(file, 'r') as f:
        data = json.load(f)

        # fpga-files array has to be merged separately
        fpga_files_tmp_data = json_data.get("fpga-files", []) + data.get("fpga-files", [])

        # replace all saluki_ with saluki- in fpga-files[].hw
        for fpga_file in fpga_files_tmp_data:
            if fpga_file.get("hw").startswith("saluki_"):
                fpga_file["hw"] = fpga_file["hw"].replace("saluki_", "saluki-")

        # bootloader-files array has to be merged separately
        bl_files_tmp_data = json_data.get("bl-files", []) + data.get("bl-files", [])

        # replace all saluki_ with saluki- in bl-files[].hw
        for bl_file in bl_files_tmp_data:
            if bl_file.get("hw").startswith("saluki_"):
                bl_file["hw"] = bl_file["hw"].replace("saluki_", "saluki-")

        # combine all json data into one
        json_data = {**json_data, **data}

        # add fpga-files from temporary merged array
        json_data["fpga-files"] = fpga_files_tmp_data

        # add fpga-files from temporary merged array
        json_data["bl-files"] = bl_files_tmp_data

print( "{}: this is the combined json data".format(sys.argv[0]))
print (json.dumps(json_data, indent=4, sort_keys=True))

# write the json_data to the output file sorted by keys
with open(sys.argv[2], 'w') as outfile:
    print("{}: writing to file: {}".format(sys.argv[0], sys.argv[2]))
    json.dump(json_data, outfile, indent=4, sort_keys=True)
