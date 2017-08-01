#!/usr/bin/env python

################################################################################
#
# Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
################################################################################

# This script can generate the client and agent code based on a set of topics
# to sent and set to receive. It uses fastrtpsgen to generate the code from the
# IDL for the topic messages. The PX4 msg definitions are used to create the IDL
# used by fastrtpsgen using templates.

import sys, os, argparse, shutil
import px_generate_uorb_topic_files
import subprocess, glob
import errno    

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
parser.add_argument("-f", "--fastrtpsgen-dir", dest='fastrtpsgen', type=str, nargs='?', help="fastrtpsgen installation dir, only needed if fastrtpsgen is not in PATH, by default empty", default="")
parser.add_argument("--delete-tree", dest='del_tree', action="store_true", help="Delete dir tree output dir(s)")

if len(sys.argv) <= 1:
    parser.print_usage()
    exit(-1)

# Parse arguments
args = parser.parse_args()
msg_folder = get_absolute_path(args.msgdir)
msg_files_send = [get_absolute_path(msg) for msg in args.send]
msg_files_receive = [get_absolute_path(msg) for msg in args.receive]
agent = args.agent
client = args.client
del_tree = args.del_tree
px_generate_uorb_topic_files.append_to_include_path({msg_folder}, px_generate_uorb_topic_files.INCL_DEFAULT)
agent_out_dir = get_absolute_path(args.agentdir)
client_out_dir = get_absolute_path(args.clientdir)

if args.fastrtpsgen is None or args.fastrtpsgen == "":
    # Assume fastrtpsgen is in PATH
    fastrtpsgen_path = "fastrtpsgen"
else:
    # Path to fastrtpsgen is explicitly specified
    fastrtpsgen_path = get_absolute_path(args.fastrtpsgen) + "/fastrtpsgen"

# If nothing specified it's generated both
if agent == False and client == False:
    agent = True
    client = True

if del_tree:
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

if agent and os.path.isdir(agent_out_dir + "/idl"):
    shutil.rmtree(agent_out_dir + "/idl")

uorb_templates_dir = msg_folder + "/templates/uorb"
urtps_templates_dir = msg_folder + "/templates/urtps"

uRTPS_CLIENT_TEMPL_FILE = 'microRTPS_client.cpp.template'
uRTPS_AGENT_TOPICS_H_TEMPL_FILE = 'RtpsTopics.h.template'
uRTPS_AGENT_TOPICS_SRC_TEMPL_FILE = 'RtpsTopics.cpp.template'
uRTPS_AGENT_TEMPL_FILE = 'microRTPS_agent.cpp.template'
uRTPS_AGENT_CMAKELIST_TEMPL_FILE = 'microRTPS_agent_CMakeLists.txt.template'
uRTPS_PUBLISHER_SRC_TEMPL_FILE = 'Publisher.cpp.template'
uRTPS_PUBLISHER_H_TEMPL_FILE = 'Publisher.h.template'
uRTPS_SUBSCRIBER_SRC_TEMPL_FILE = 'Subscriber.cpp.template'
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
    mkdir_p(agent_out_dir + "/fastrtpsgen")
    os.chdir(agent_out_dir + "/fastrtpsgen")
    for idl_file in glob.glob(agent_out_dir + "/idl/*.idl"):
        ret = subprocess.call(fastrtpsgen_path + " -d " + agent_out_dir + "/fastrtpsgen -example x64Linux2.6gcc " + idl_file, shell=True)
        if ret:
            raise Exception("fastrtpsgen not found. Specify the location of fastrtpsgen with the -f flag")
    rm_wildcard(agent_out_dir + "/fastrtpsgen/*PubSubMain*")
    rm_wildcard(agent_out_dir + "/fastrtpsgen/makefile*")
    rm_wildcard(agent_out_dir + "/fastrtpsgen/*Publisher*")
    rm_wildcard(agent_out_dir + "/fastrtpsgen/*Subscriber*")
    for f in glob.glob(agent_out_dir + "/fastrtpsgen/*.cxx"):
        os.rename(f, f.replace(".cxx", ".cpp"))
    cp_wildcard(agent_out_dir + "/fastrtpsgen/*", agent_out_dir)
    if os.path.isdir(agent_out_dir + "/fastrtpsgen"):
        shutil.rmtree(agent_out_dir + "/fastrtpsgen")
    cp_wildcard(urtps_templates_dir + "/microRTPS_transport.*", agent_out_dir)
    os.rename(agent_out_dir + "/microRTPS_agent_CMakeLists.txt", agent_out_dir + "/CMakeLists.txt")
    mkdir_p(agent_out_dir + "/build")
    return 0

def rm_wildcard(pattern):
    for f in glob.glob(pattern):
        os.remove(f)

def cp_wildcard(pattern, destdir):
    for f in glob.glob(pattern):
        shutil.copy(f, destdir)

def mkdir_p(dirpath):
    try:
        os.makedirs(dirpath)
    except OSError as e:
        if e.errno == errno.EEXIST and os.path.isdir(dirpath):
            pass
        else:
            raise

def generate_client(out_dir):

    px_generate_uorb_topic_files.generate_uRTPS_general(msg_files_send, msg_files_receive, out_dir, uorb_templates_dir,
                    px_generate_uorb_topic_files.INCL_DEFAULT, uRTPS_CLIENT_TEMPL_FILE)

    # Final steps to install client
    cp_wildcard(urtps_templates_dir + "/microRTPS_transport.*", client_out_dir)

    return 0

if agent:
    generate_agent(agent_out_dir)
    print("\nAgent created in: " + agent_out_dir)

if client:
    generate_client(client_out_dir)
    print("\nClient created in: " + client_out_dir)
