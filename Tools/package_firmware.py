#!/usr/bin/env python

import glob
import zipfile
import os

FIRMWARE_FILE = 'Firmware.zip'


def extract_file_only(filename, dest):
    """
    Extract a file without keeping directories.
    """
    # extract firmware files without paths
    f_src = archive.open(filename, 'r')
    data = f_src.read()
    dst_name = os.path.join(dest, os.path.basename(filename))
    with open(dst_name, 'w') as f_dst:
        f_dst.write(data)
    f_src.close()

# open destination archive
with zipfile.ZipFile(FIRMWARE_FILE, 'w') as dest_archive:

    # get all zip files in Packages directory
    for zip_filename in glob.glob("Packages/*.zip"):

        # open zipfile
        with zipfile.ZipFile(zip_filename, 'r') as archive:

            # look for interesting names
            for src_filename in archive.namelist():

                # extract firmware files
                if os.path.splitext(src_filename)[1] == '.px4':
                    base_name = os.path.basename(src_filename)
                    dest_archive.writestr(base_name, archive.read(src_filename))
