#!/usr/bin/env python3
"""
Script to parse uORB message format to ROS msg format
Adapted from https://github.com/eProsima/px4_to_ros/blob/master/px4_to_ros2_PoC/px4_msgs/scripts/copy_and_rename.py
"""

import errno
import argparse
import os
import sys
from shutil import copyfile
import yaml

__author__ = 'PX4 Development Team'
__copyright__ = \
    '''
     '
     '   Copyright (C) 2018-2019 PX4 Development Team. All rights reserved.
     '
     ' Redistribution and use in source and binary forms, or without
     ' modification, permitted provided that the following conditions
     ' are met:
     '
     ' 1. Redistributions of source code must retain the above copyright
     '    notice, list of conditions and the following disclaimer.
     ' 2. Redistributions in binary form must reproduce the above copyright
     '    notice, list of conditions and the following disclaimer in
     '    the documentation and/or other materials provided with the
     '    distribution.
     ' 3. Neither the name PX4 nor the names of its contributors may be
     '    used to endorse or promote products derived from self software
     '    without specific prior written permission.
     '
     ' THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     ' "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, NOT
     ' LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
     ' FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
     ' COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
     ' INCIDENTAL, SPECIAL, EXEMPLARY, CONSEQUENTIAL DAMAGES (INCLUDING,
     ' BUT NOT LIMITED TO, OF SUBSTITUTE GOODS OR SERVICES; LOSS
     ' OF USE, DATA, PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
     ' AND ON ANY THEORY OF LIABILITY, IN CONTRACT, STRICT
     ' LIABILITY, TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
     ' ANY WAY OUT OF THE USE OF THIS SOFTWARE, IF ADVISED OF THE
     ' POSSIBILITY OF SUCH DAMAGE.
     '
    '''
__credits__ = [
    'Nuno Marques <nuno.marques@dronesolution.io>',
    'Vicente Monge']
__license__ = 'BSD-3-Clause'
__version__ = '1.10.0'
__maintainer__ = 'Nuno Marques'
__email__ = 'nuno.marques@dronesolution.io'
__status__ = 'Development'


def load_yaml_file(file):
    """
    Open yaml file and parse the data into a list of dict

    :param file: the yaml file to load
    :returns: the list of dictionaries that represent the message and respective RTPS IDs
    :raises IOError: raises and error when the file is not found
    """
    try:
        with open(file, 'r') as f:
            return yaml.safe_load(f)
    except OSError as e:
        if e.errno == errno.ENOENT:
            raise IOError(errno.ENOENT, os.strerror(
                errno.ENOENT), file)
        else:
            raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Parse uORB message format to ROS msg format')
    optional = parser._action_groups.pop()
    required = parser.add_argument_group('Required')
    required.add_argument("-i", "--topic-msg-dir", dest='indir', type=str,
                          help="uORB messages dir", metavar="UORB_MSG_DIR")
    required.add_argument(
        "-o",
        "--ros-msg-dir",
        dest='outdir',
        type=str,
        help="Output ROS messages dir",
        metavar="ROS_MSG_DIR")

    # Parse arguments
    args = parser.parse_args()
    input_dir = os.path.abspath(args.indir)
    output_dir = os.path.abspath(args.outdir)

    if not os.path.exists(os.path.abspath(output_dir)):
        os.mkdir(os.path.abspath(output_dir))
    else:
        ros_msg_dir = os.path.abspath(output_dir)
        msg_files = os.listdir(ros_msg_dir)
        for msg in msg_files:
            if msg.endswith(".msg"):
                os.remove(os.path.join(ros_msg_dir, msg))

    msg_list = list()

    # copies the uORB msgs to the ROS msg dir, applying the CamelCase style to
    # the naming
    for filename in os.listdir(input_dir):
        if '.msg' in filename:
            msg_list.append(filename.rstrip('.msg'))
            input_file = os.path.join(input_dir, filename)

            output_file = os.path.join(output_dir, filename.partition(".")[
                                       0].title().replace('_', '') + ".msg")
            copyfile(input_file, output_file)

    # removes unnecessary content from the msgs and creates new messages from #TOPICS on
    # the messages that have them
    for filename in os.listdir(output_dir):
        if '.msg' in filename:
            input_file = os.path.join(output_dir, filename)

            fileUpdated = False

            with open(input_file, 'r') as f:
                lines = f.readlines()
                newlines = []
                alias_msgs = []
                alias_msg_files = []

                for line in lines:
                    for msg_type in msg_list:
                        if ('px4/' + msg_type + ' ') in line:
                            fileUpdated = True
                            line = line.replace(
                                ('px4/' + msg_type),
                                msg_type.partition(".")[0].title().replace(
                                    '_',
                                    ''))
                        if ('' +
                            msg_type +
                            '[') in line.partition('#')[0] or ('' +
                                                               msg_type +
                                                               ' ') in line.partition('#')[0]:
                            fileUpdated = True
                            line = line.replace(msg_type, msg_type.partition(".")[
                                                0].title().replace('_', ''))
                        if '# TOPICS' in line:
                            fileUpdated = True
                            alias_msgs += line.split()
                            alias_msgs.remove('#')
                            alias_msgs.remove('TOPICS')
                            line = line.replace(line, '')
                    newlines.append(line)

            for msg_file in alias_msgs:
                with open(os.path.join(output_dir, msg_file.partition(".")[0].title().replace('_', '') + ".msg"), 'w+') as f:
                    for line in newlines:
                        f.write(line)

            if fileUpdated:
                with open(input_file, 'w+') as f:
                    for line in newlines:
                        f.write(line)
