'''
Use mavgen.py matrixpilot.xml definitions to generate
C and Python MAVLink routines for sending and parsing the protocol
This python script is soley for MatrixPilot MAVLink impoementations

Copyright Pete Hollands 2011, 2012
Released under GNU GPL version 3 or later
'''

import os, sys, glob, re
from shutil import copy
from mavgen import mavgen

# allow import from the parent directory, where mavutil.py is
# Under Windows, this script must be run from a DOS command window 
# sys.path.insert(0, os.path.join(os.path.dirname(os.path.realpath(__file__)), '..'))
sys.path.insert(0, os.path.join(os.getcwd(), '..'))

class options:
    """ a class to simulate the options of mavgen OptionsParser"""
    def __init__(self, lang, output, wire_protocol, error_limit):
        self.language = lang
        self.wire_protocol = wire_protocol
        self.output = output
        self.error_limit = error_limit

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

def remove_xml_files(target_directory):
    search_pattern = target_directory+'/*.xml'
    print "search pattern is", search_pattern
    files_to_remove = glob.glob(search_pattern)
    for afile in files_to_remove :
        try:
            print "removing", afile
            os.remove(afile)
        except:
            print "error while trying to remove", afile

def copy_xml_files(source_directory,target_directory):
    search_pattern = source_directory+'/*.xml'
    files_to_copy = glob.glob(search_pattern)
    for afile in files_to_copy:
        basename = os.path.basename(afile)
        print "Copying ...", basename
        copy(afile, target_directory)


########### Generate MAVlink files for C and Python from XML definitions
protocol = "1.0"
 
xml_directory = '../../message_definitions/v'+protocol
print "xml_directory is", xml_directory

xml_file_names = []
xml_file_names.append(xml_directory+"/"+"common.xml")
xml_file_names.append(xml_directory+"/"+"matrixpilot.xml")

#Check to see if python directory exists ...
directory = 'python'
if not os.path.isdir(directory):
    os.makedirs(directory)

for xml_file in xml_file_names:
    print "xml file is ", xml_file   
    
    xml_file_base = os.path.basename(xml_file)
    xml_file_base = re.sub("\.xml","", xml_file_base)
    print "xml_file_base is", xml_file_base
    target_directory = "../../../../../MAVLink/include/"+xml_file_base
    source_directory = "C/include_v"+protocol+"/"+xml_file_base

    print "About to remove all files in",source_directory
    print "OK to continue ?[Yes / No]: ",
    line = sys.stdin.readline()
    if line == "Yes\n" or line == "yes\n" \
       or line == "Y\n" or line == "y\n":
        print "passed"
        remove_include_files(source_directory)
        print "Finished removing C include files for", xml_file_base
    else :
        print "Your answer is No. Exiting Program"
        sys.exit()

    opts = options(lang = "C", output = "C/include_v"+protocol, \
                   wire_protocol=protocol, error_limit=200)
    args = []
    args.append(xml_file)
    print "About to generate C include files"
    mavgen(opts, args)
    opts = options(lang = "python", \
                   output="python/mavlink_"+xml_file_base+"_v"+protocol+".py", \
                   wire_protocol=protocol, error_limit=200)
    print "About to generate python parsers"
    mavgen(opts,args)
    
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
                print "Finished copying over xml derived include files"
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

# Copy newer versions of C header files 
header_files = ['checksum.h','mavlink_helpers.h', 'mavlink_protobuf_manager.hpp', \
                'mavlink_types.h', 'protocol.h' ]
target_directory = "../../../../../MAVLink/include/"
source_directory = "C/include_v"+protocol+"/"
print "Copying over upper level header files..."
for filename in header_files :
  print "Copying ... ", filename
  #print "About to copy source_file", source_directory+filename, "to",target_directory+filename
  if  os.access(source_directory+filename, os.R_OK):
    #print "Can read source file", source_directory+filename
    if  os.access(source_directory+filename, os.W_OK):
      copy(source_directory+filename, target_directory+filename)
      #print "Finished copying to", target_directory+filename
    else :
      print "Could not access", target_directory+filename, " for writing"
  else :
    print "Could not access file to copy called ", source_directory+filename


# Copy specific Mavlink wire protocol 1.0 python parsers for MatrixPilot
source_file =  "./python/mavlink_matrixpilot_v1.0.py"
target_files = "./mavlink.py" , "../mavlinkv10.py"
for target_name in target_files:
  print "About to copy source_file", source_file, "to",target_name
  if  os.access(source_file, os.R_OK):
    print "Can read source file", source_file
    if  os.access(source_file, os.W_OK):
      copy(source_file, target_name)
      print "Finished copying to", target_name
    else :
      print "Could not access", target_name, " for writing"
  else :
    print "Could not access file to copy called ", source_file

         
        
##### End of Main program to generate MAVLink C and Python files ####

##### Copy new XML message definitions to main trunk directories
source_directory = "../../message_definitions/V1.0"
target_directory = "../../../../../MAVLink/message_definitions"
if os.access(source_directory, os.R_OK):
    if os.access(target_directory, os.W_OK):
        print "Preparing to copy over xml files ..."
        print "About to remove files in ",target_directory
        print "OK to continue ?[Yes / No]: ",
        line = sys.stdin.readline()
        if line == "Yes\n" or line == "yes\n" \
           or line == "Y\n" or line == "y\n":
            print "passed"
            try:
                print "removing xml files in", target_directory
                remove_xml_files(target_directory)
            except:
                print "error while trying to remove files in ", target_directory
            print "Copying xml files from ", source_directory
            copy_xml_files(source_directory, target_directory) 
            print "Finished copying over python files"
        else :
            print "Your answer is No. Exiting Program"
            sys.exit()
    else :
       print "Cannot find " + target_directory 
       sys.exit() 
else:
    print "Could not find files to copy at", source_directory
    print "Exiting Program."
    sys.exit()
print "Program has finished, please press Return Key to exit"
line = sys.stdin.readline()


