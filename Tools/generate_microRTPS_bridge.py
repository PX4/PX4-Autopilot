#!/usr/bin/env python

################################################################################
# Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

import sys, os, argparse, shutil
import px_generate_uorb_topic_files

parser = argparse.ArgumentParser()
parser.add_argument("-s", "--send", dest='send', metavar='*.msg', type=str, nargs='+', help="Topics to be sended")
parser.add_argument("-r", "--receive", dest='receive', metavar='*.msg', type=str, nargs='+', help="Topics to be received")

if len(sys.argv) > 1:
    msg_files_send = parser.parse_args().send
    msg_files_receive = parser.parse_args().receive
else:
    parser.print_usage()
    exit(-1)

root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
msg_folder = root_path + "/msg"
px_generate_uorb_topic_files.append_to_include_path({msg_folder}, px_generate_uorb_topic_files.INCL_DEFAULT)

out_dir = root_path + "/msgenerated"
print("Files created in: " + out_dir)

if os.path.isdir(out_dir):
    shutil.rmtree(out_dir)

uorb_templates_dir = root_path + "/msg/templates/uorb"
urtps_templates_dir = root_path + "/msg/templates/urtps"

uRTPS_CLIENT_TEMPL_FILE = 'microRTPS_client.cpp.template'
uRTPS_CLIENT_CMAKELIST_TEMPL_FILE = 'microRTPS_client_CMakeLists.txt.template'
uRTPS_AGENT_TEMPL_FILE = 'microRTPS_agent.cxx.template'
uRTPS_AGENT_CMAKELIST_TEMPL_FILE = 'microRTPS_agent_CMakeLists.txt.template'
uRTPS_PUBLISHER_SRC_TEMPL_FILE = 'Publisher.cxx.template'
uRTPS_PUBLISHER_H_TEMPL_FILE = 'Publisher.h.template'
uRTPS_SUBSCRIBER_SRC_TEMPL_FILE = 'Subscriber.cxx.template'
uRTPS_SUBSCRIBER_H_TEMPL_FILE = 'Subscriber.h.template'

if msg_files_send:
    for msg_file in msg_files_send:
        px_generate_uorb_topic_files.generate_idl_file(msg_file, out_dir, urtps_templates_dir,
            px_generate_uorb_topic_files.INCL_DEFAULT)
        px_generate_uorb_topic_files.generate_topic_file(msg_file, out_dir, urtps_templates_dir,
            px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_PUBLISHER_SRC_TEMPL_FILE)
        px_generate_uorb_topic_files.generate_topic_file(msg_file, out_dir, urtps_templates_dir,
            px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_PUBLISHER_H_TEMPL_FILE)

if msg_files_receive:
    for msg_file in msg_files_receive:
        px_generate_uorb_topic_files.generate_idl_file(msg_file, out_dir, urtps_templates_dir,
            px_generate_uorb_topic_files.INCL_DEFAULT)
        px_generate_uorb_topic_files.generate_topic_file(msg_file, out_dir, urtps_templates_dir,
            px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_SUBSCRIBER_SRC_TEMPL_FILE)
        px_generate_uorb_topic_files.generate_topic_file(msg_file, out_dir, urtps_templates_dir,
            px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_SUBSCRIBER_H_TEMPL_FILE)


px_generate_uorb_topic_files.generate_uRTPS_general(msg_files_send, msg_files_receive, out_dir, uorb_templates_dir,
				px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_CLIENT_TEMPL_FILE)

px_generate_uorb_topic_files.generate_uRTPS_general(msg_files_send, msg_files_receive, out_dir, uorb_templates_dir,
				px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_CLIENT_CMAKELIST_TEMPL_FILE)

px_generate_uorb_topic_files.generate_uRTPS_general(msg_files_send, msg_files_receive, out_dir, urtps_templates_dir,
				px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_AGENT_TEMPL_FILE)

px_generate_uorb_topic_files.generate_uRTPS_general(msg_files_send, msg_files_receive, out_dir, urtps_templates_dir,
                px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_AGENT_CMAKELIST_TEMPL_FILE)
