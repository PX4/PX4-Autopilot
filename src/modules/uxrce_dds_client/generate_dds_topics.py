#!/usr/bin/env python3
################################################################################
#
# Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
# Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
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

# This script can generate the client code based on a set of topics
# to sent and set to receive.

import sys
import os
import argparse
import re
import em
import yaml

parser = argparse.ArgumentParser()
parser.add_argument("-m", "--topic-msg-dir", dest='msgdir', type=str,
                    help="Topics message, by default using relative path 'msg/'", default="msg")

parser.add_argument("-y", "--dds-topics-file", dest='yaml_file', type=str,
                    help="Setup topics file path, by default using 'dds_topics.yaml'")

parser.add_argument("-t", "--template_file", dest='template_file', type=str,
                    help="DDS topics template file")

parser.add_argument("-u", "--client-outdir", dest='clientdir', type=str,
                    help="Client output dir, by default using relative path 'src/modules/uxrce_dds_client'", default=None)

if len(sys.argv) <= 1:
    parser.print_usage()
    exit(-1)

# Parse arguments
args = parser.parse_args()

# Client files output path
client_out_dir = os.path.abspath(args.clientdir)
template_file = os.path.join(args.template_file)

# Make sure output directory exists:
if not os.path.isdir(client_out_dir):
    os.makedirs(client_out_dir)

output_file = os.path.join(client_out_dir, os.path.basename(template_file).replace(".em", ""))
folder_name = os.path.dirname(output_file)

if not os.path.exists(folder_name):
    os.makedirs(folder_name)



# open yaml file to load dictionary of publications and subscriptions
with open(args.yaml_file, 'r') as file:
    msg_map = yaml.safe_load(file)

merged_em_globals = {}
all_type_includes = []

for p in msg_map['publications']:
    # eg TrajectoryWaypoint from px4_msgs::msg::TrajectoryWaypoint
    simple_base_type = p['type'].split('::')[-1]

    # eg TrajectoryWaypoint -> trajectory_waypoint
    base_type_name_snake_case = re.sub(r'(?<!^)(?=[A-Z])', '_', simple_base_type).lower()
    all_type_includes.append(base_type_name_snake_case)

    # simple_base_type: eg vehicle_status
    p['simple_base_type'] = base_type_name_snake_case

    # dds_type: eg px4_msgs::msg::dds_::VehicleStatus_
    p['dds_type'] = p['type'].replace("::msg::", "::msg::dds_::") + "_"

    # topic_simple: eg vehicle_status
    p['topic_simple'] = p['topic'].split('/')[-1]


merged_em_globals['publications'] = msg_map['publications']

for s in msg_map['subscriptions']:
    # eg TrajectoryWaypoint from px4_msgs::msg::TrajectoryWaypoint
    simple_base_type = s['type'].split('::')[-1]

    # eg TrajectoryWaypoint -> trajectory_waypoint
    base_type_name_snake_case = re.sub(r'(?<!^)(?=[A-Z])', '_', simple_base_type).lower()
    all_type_includes.append(base_type_name_snake_case)

    # simple_base_type: eg vehicle_status
    s['simple_base_type'] = base_type_name_snake_case

    # dds_type: eg px4_msgs::msg::dds_::VehicleStatus_
    s['dds_type'] = s['type'].replace("::msg::", "::msg::dds_::") + "_"

    # topic_simple: eg vehicle_status
    s['topic_simple'] = s['topic'].split('/')[-1]


merged_em_globals['subscriptions'] = msg_map['subscriptions']

merged_em_globals['type_includes'] = sorted(set(all_type_includes))


# run interpreter
ofile = open(output_file, 'w')
interpreter = em.Interpreter(output=ofile, globals=merged_em_globals, options={em.RAW_OPT: True, em.BUFFERED_OPT: True})

try:
    interpreter.file(open(template_file))
except OSError as e:
    ofile.close()
    os.remove(output_file)
    raise

interpreter.shutdown()
ofile.close()
