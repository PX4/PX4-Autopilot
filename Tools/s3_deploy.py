#!/usr/bin/env python

import glob
import zipfile
import os
import re
import shutil

S3_DIR = 's3deploy-branch'
S3_ARCHIVE_DIR = 's3deploy-archive'

if not os.path.isdir(S3_DIR):
    os.mkdir(S3_DIR)

if not os.path.isdir(S3_ARCHIVE_DIR):
    os.mkdir(S3_ARCHIVE_DIR)

shutil.copy("Firmware.zip", S3_ARCHIVE_DIR)

def extract_file_only(filename, dest):
    # extract firmware files without paths
    f_src = archive.open(filename, 'r')
    data = f_src.read()
    with open(os.path.join(dest,
              os.path.basename(filename)), 'w') as f_dst:
        f_dst.write(data)
    f_src.close()

# get all zip files in Packages directory
for zip_filename in glob.glob("Packages/*.zip"):

    # open zipfile
    with zipfile.ZipFile(zip_filename, 'r') as archive:

        # look for interesting names
        for filename in archive.namelist():

            # extract firmware files
            if os.path.splitext(filename)[1] == '.px4':
                extract_file_only(filename, S3_DIR)

            # copy px4fmu-v4_default xml files for qgroundcontrol
            if re.match(filename, r'.*px4fmu-v4_default.*\.xml') is not None:
                extract_file_only(filename, S3_DIR)
