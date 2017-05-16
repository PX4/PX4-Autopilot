#!/usr/bin/env python

import sys, os
import px_generate_uorb_topic_files

if len(sys.argv) > 1:
    msg_file = sys.argv[1]
else:
    print("A .msg file must be specified")
    exit(-1)

root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
msg_folder = root_path + "/msg"
px_generate_uorb_topic_files.append_to_include_path({msg_folder}, px_generate_uorb_topic_files.INCL_DEFAULT)

out_dir = root_path + "/msgenerated"
print("Files created in: " + out_dir)

uorb_templates_dir = root_path + "/msg/templates/uorb"
urtps_templates_dir = root_path + "/msg/templates/urtps"

px_generate_uorb_topic_files.generate_uRTPS_application_file(msg_file, out_dir, uorb_templates_dir,  px_generate_uorb_topic_files.INCL_DEFAULT)
px_generate_uorb_topic_files.generate_idl_file(              msg_file, out_dir, urtps_templates_dir, px_generate_uorb_topic_files.INCL_DEFAULT)
px_generate_uorb_topic_files.generate_uRTPS_receiver_header( msg_file, out_dir, urtps_templates_dir, px_generate_uorb_topic_files.INCL_DEFAULT)
px_generate_uorb_topic_files.generate_uRTPS_receiver_source( msg_file, out_dir, urtps_templates_dir, px_generate_uorb_topic_files.INCL_DEFAULT)
