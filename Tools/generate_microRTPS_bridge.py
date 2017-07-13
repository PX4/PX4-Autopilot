#!/usr/bin/env python

################################################################################
# Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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
import subprocess, glob

def get_absolute_path(arg_parse_dir):
    root_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    if isinstance(arg_parse_dir, list):
        dir = arg_parse_dir[0]
    else:
        dir = arg_parse_dir

    if dir[0] != '/':
        dir = root_path + "/" + dir

    return dir


parser = argparse.ArgumentParser()
parser.add_argument("-s", "--send", dest='send', metavar='*.msg', type=str, nargs='+', help="Topics to be sended")
parser.add_argument("-r", "--receive", dest='receive', metavar='*.msg', type=str, nargs='+', help="Topics to be received")
parser.add_argument("-a", "--agent", dest='agent', action="store_true", help="Flag for generate the agent, by default is true if -c is not specified")
parser.add_argument("-c", "--client", dest='client', action="store_true", help="Flag for generate the client, by default is true if -a is not specified")
parser.add_argument("-t", "--topic-msg-dir", dest='msgdir', type=str, nargs=1, help="Topics message dir, by default msg/", default="msg")
parser.add_argument("-o", "--agent-outdir", dest='agentdir', type=str, nargs=1, help="Agent output dir, by default src/modules/micrortps_bridge/micrortps_agent", default="src/modules/micrortps_bridge/micrortps_agent")
parser.add_argument("-u", "--client-outdir", dest='clientdir', type=str, nargs=1, help="Client output dir, by default src/modules/micrortps_bridge/micrortps_client", default="src/modules/micrortps_bridge/micrortps_client")
parser.add_argument("-f", "--fastrtpsgen-dir", dest='fastrtpsgen', type=str, nargs=1, help="fastrtpsgen installation dir, only needed if fastrtpsgen is not in PATH, by default empty", default="")
parser.add_argument("--no-delete", dest='no_del', action="store_true", help="Do not delete dir tree output dir(s)")

if len(sys.argv) <= 1:
    parser.print_usage()
    exit(-1)

# Parse arguments
args = parser.parse_args()
msg_folder = get_absolute_path(args.msgdir)
msg_files_send = args.send
msg_files_receive = args.receive
agent = args.agent
client = args.client
no_del = args.no_del
px_generate_uorb_topic_files.append_to_include_path({msg_folder}, px_generate_uorb_topic_files.INCL_DEFAULT)
agent_out_dir = get_absolute_path(args.agentdir)
client_out_dir = get_absolute_path(args.clientdir)

if args.fastrtpsgen != "":
    # Path to fastrtpsgen is explicitly specified
    fastrtpsgen_path = get_absolute_path(args.fastrtpsgen) + "/fastrtpsgen"
else:
    # Assume fastrtpsgen is in PATH
    fastrtpsgen_path = "fastrtpsgen"

# If nothing specified it's generated both
if agent == False and client == False:
    agent = True
    client = True

if not no_del:
    if agent:
        _continue = str(raw_input("\nFiles in " +  agent_out_dir + " will be erased, continue?[Y/n]\n"))
        if _continue == "N" or _continue == "n":
            print("Aborting execution...")
            exit(-1)
        else:
            if agent and os.path.isdir(agent_out_dir):
                shutil.rmtree(agent_out_dir)

    if client:
        _continue = str(raw_input("\nFiles in " +  client_out_dir + " will be erased, continue?[Y/n]\n"))
        if _continue == "N" or _continue == "n":
            print("Aborting execution...")
            exit(-1)
        else:
            if client and os.path.isdir(client_out_dir):
                shutil.rmtree(client_out_dir)

uorb_templates_dir = msg_folder + "/templates/uorb"
urtps_templates_dir = msg_folder + "/templates/urtps"

uRTPS_CLIENT_TEMPL_FILE = 'microRTPS_client.cpp.template'
uRTPS_AGENT_TOPICS_H_TEMPL_FILE = 'RtpsTopics.h.template'
uRTPS_AGENT_TOPICS_SRC_TEMPL_FILE = 'RtpsTopics.cxx.template'
uRTPS_AGENT_TEMPL_FILE = 'microRTPS_agent.cxx.template'
uRTPS_AGENT_CMAKELIST_TEMPL_FILE = 'microRTPS_agent_CMakeLists.txt.template'
uRTPS_PUBLISHER_SRC_TEMPL_FILE = 'Publisher.cxx.template'
uRTPS_PUBLISHER_H_TEMPL_FILE = 'Publisher.h.template'
uRTPS_SUBSCRIBER_SRC_TEMPL_FILE = 'Subscriber.cxx.template'
uRTPS_SUBSCRIBER_H_TEMPL_FILE = 'Subscriber.h.template'

def generate_agent(out_dir):

    if msg_files_send:
        for msg_file in msg_files_send:
            px_generate_uorb_topic_files.generate_idl_file(msg_file, out_dir + "/idl", urtps_templates_dir,
                px_generate_uorb_topic_files.INCL_DEFAULT)
            px_generate_uorb_topic_files.generate_topic_file(msg_file, out_dir, urtps_templates_dir,
                px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_PUBLISHER_SRC_TEMPL_FILE)
            px_generate_uorb_topic_files.generate_topic_file(msg_file, out_dir, urtps_templates_dir,
                px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_PUBLISHER_H_TEMPL_FILE)

    if msg_files_receive:
        for msg_file in msg_files_receive:
            px_generate_uorb_topic_files.generate_idl_file(msg_file, out_dir + "/idl", urtps_templates_dir,
                px_generate_uorb_topic_files.INCL_DEFAULT)
            px_generate_uorb_topic_files.generate_topic_file(msg_file, out_dir, urtps_templates_dir,
                px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_SUBSCRIBER_SRC_TEMPL_FILE)
            px_generate_uorb_topic_files.generate_topic_file(msg_file, out_dir, urtps_templates_dir,
                px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_SUBSCRIBER_H_TEMPL_FILE)

    px_generate_uorb_topic_files.generate_uRTPS_general(msg_files_send, msg_files_receive, out_dir, urtps_templates_dir,
                    px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_AGENT_TEMPL_FILE)
    px_generate_uorb_topic_files.generate_uRTPS_general(msg_files_send, msg_files_receive, out_dir, urtps_templates_dir,
                    px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_AGENT_TOPICS_H_TEMPL_FILE)
    px_generate_uorb_topic_files.generate_uRTPS_general(msg_files_send, msg_files_receive, out_dir, urtps_templates_dir,
                    px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_AGENT_TOPICS_SRC_TEMPL_FILE)
    px_generate_uorb_topic_files.generate_uRTPS_general(msg_files_send, msg_files_receive, out_dir, urtps_templates_dir,
                    px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_AGENT_CMAKELIST_TEMPL_FILE)

    # Final steps to install agent
    os.system("mkdir -p " + agent_out_dir + "/fastrtpsgen")
    for idl_file in glob.glob( agent_out_dir + "/idl/*.idl"):
        ret = os.system("cd " + agent_out_dir + "/fastrtpsgen && " + fastrtpsgen_path + " -example x64Linux2.6gcc " + idl_file)
        if ret:
            raise Exception("fastrtpsgen not found. Specify the location of fastrtpsgen with the -f flag")

    os.system("rm " + agent_out_dir + "/fastrtpsgen/*PubSubMain.cxx "
                    + agent_out_dir + "/fastrtpsgen/makefile* "
                    + agent_out_dir + "/fastrtpsgen/*Publisher* "
                    + agent_out_dir + "/fastrtpsgen/*Subscriber*")
    os.system("cp " + agent_out_dir + "/fastrtpsgen/* " + agent_out_dir)
    os.system("rm -rf " + agent_out_dir + "/fastrtpsgen/")
    os.system("cp " + urtps_templates_dir + "/microRTPS_transport.* " + agent_out_dir)
    os.system("mv " + agent_out_dir + "/microRTPS_agent_CMakeLists.txt " + agent_out_dir + "/CMakeLists.txt")
    os.system("mkdir -p " + agent_out_dir + "/build")

    return 0

def generate_client(out_dir):

    px_generate_uorb_topic_files.generate_uRTPS_general(msg_files_send, msg_files_receive, out_dir, uorb_templates_dir,
                    px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_CLIENT_TEMPL_FILE)

    # Final steps to install client
    os.system("cp " + urtps_templates_dir + "/microRTPS_transport.* " + client_out_dir)

    return 0

if agent:
    generate_agent(agent_out_dir)
    print("\nAgent created in: " + agent_out_dir)

if client:
    generate_client(client_out_dir)
    print("\nClient created in: " + client_out_dir)
