#!/usr/bin/env python
'''
Use mavgen.py matrixpilot.xml definitions to generate
C and Python MAVLink routines for sending and parsing the protocol
This python script is soley for MatrixPilot MAVLink impoementation

Copyright Pete Hollands 2011
Released under GNU GPL version 3 or later
'''

import os, sys, glob, re
from shutil import copy
from mavgen import mavgen

# allow import from the parent directory, where mavutil.py is
# Under Windows, this script must be run from a DOS command window
sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))

class options:
    """ a class to simulate the options of mavgen OptionsParser"""
    def __init__(self, lang, output, wire_protocol):
        self.language = lang
        self.wire_protocol = wire_protocol
        self.output = output

def remove_include_files(target_directory):
    search_pattern = target_directory+'/*.h'
    print "search pattern is", search_pattern
    files_to_remove = glob.glob(search_pattern)
    for afile in files_to_remove :
        try:
            print "removing", afile
            os.remove(afile)
        except:
            print "error while trying to remove", afile

def copy_include_files(source_directory,target_directory):
    search_pattern = source_directory+'/*.h'
    files_to_copy = glob.glob(search_pattern)
    for afile in files_to_copy:
        basename = os.path.basename(afile)
        print "Copying ...", basename
        copy(afile, target_directory)

protocol = "1.0"

xml_directory = './message_definitions/v'+protocol
print "xml_directory is", xml_directory
xml_file_names = []
xml_file_names.append(xml_directory+"/"+"matrixpilot.xml")

for xml_file in xml_file_names:
    print "xml file is ", xml_file
    opts = options(lang = "C", output = "C/include_v"+protocol, \
                   wire_protocol=protocol)
    args = []
    args.append(xml_file)
    mavgen(opts, args)
    xml_file_base = os.path.basename(xml_file)
    xml_file_base = re.sub("\.xml","", xml_file_base)
    print "xml_file_base is", xml_file_base
    opts = options(lang = "python", \
                   output="python/mavlink_"+xml_file_base+"_v"+protocol+".py", \
                   wire_protocol=protocol)
    mavgen(opts,args)

mavlink_directory_list = ["common","matrixpilot"]
for mavlink_directory in mavlink_directory_list :
    # Look specifically for MatrixPilot directory structure
    target_directory = "../../../../MAVLink/include/"+mavlink_directory
    source_directory = "C/include_v"+protocol+"/"+mavlink_directory
    if os.access(source_directory, os.R_OK):
        if os.access(target_directory, os.W_OK):
            print "Preparing to copy over files..."
            print "About to remove all files in",target_directory
            print "OK to continue ?[Yes / No]: ",
            line = sys.stdin.readline()
            if line == "Yes\n" or line == "yes\n" \
               or line == "Y\n" or line == "y\n":
                print "passed"
                remove_include_files(target_directory)
                copy_include_files(source_directory,target_directory)
                print "Finished copying over include files"
            else :
                print "Your answer is No. Exiting Program"
                sys.exit()
        else :
           print "Cannot find " + target_directory + "in MatrixPilot"
           sys.exit()
    else:
        print "Could not find files to copy at", source_directory
        print "Exiting Program."
        sys.exit()
