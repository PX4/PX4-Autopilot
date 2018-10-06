#!/usr/bin/env python

################################################################################
#
# Copyright 2018 PX4 Pro Development Team. All rights reserved.
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

import sys
import os
import argparse
import yaml


class Classifier():
    """
    Class to classify RTPS msgs as senders, receivers or to be ignored
    """

    def __init__(self, yaml_file, msg_folder):
        self.msg_id_map = self.parse_yaml_msg_id_file(yaml_file)
        self.msg_folder = msg_folder
        self.msgs_to_send = self.set_msgs_to_send()
        self.msgs_to_receive = self.set_msgs_to_receive()
        self.msgs_to_ignore = self.set_msgs_to_ignore()
        self.msg_files_send = self.set_msg_files_send()
        self.msg_files_receive = self.set_msg_files_receive()
        self.msg_files_ignore = self.set_msg_files_ignore()

    # getters
    @property
    def msg_id_map(self):
        return self.__msg_id_map

    @property
    def msg_folder(self):
        return self.__msg_folder

    @property
    def msgs_to_send(self):
        return self.__msgs_to_send

    @property
    def msgs_to_receive(self):
        return self.__msgs_to_receive

    @property
    def msgs_to_ignore(self):
        return self.__msgs_to_ignore

    @property
    def msg_files_send(self):
        return self.__msg_files_send

    @property
    def msg_files_receive(self):
        return self.__msg_files_receive

    @property
    def msg_files_ignore(self):
        return self.__msg_files_ignore

    # setters (for class init)
    def set_msgs_to_send(self):
        send = {}
        for dict in self.msg_id_map['rtps']:
            if 'send' in dict.keys():
                send.update({dict['msg']: dict['id']})
        return send

    def set_msgs_to_receive(self):
        receive = {}
        for dict in self.msg_id_map['rtps']:
            if 'receive' in dict.keys():
                receive.update({dict['msg']: dict['id']})
        return receive

    def set_msgs_to_ignore(self):
        ignore = {}
        for dict in self.msg_id_map['rtps']:
            if ('send' not in dict.keys()) and ('receive' not in dict.keys()):
                ignore.update({dict['msg']: dict['id']})
        return ignore

    def set_msg_files_send(self):
        return [os.path.join(self.msg_folder, msg + ".msg")
                for msg in self.msgs_to_send.keys()]

    def set_msg_files_receive(self):
        return [os.path.join(self.msg_folder, msg + ".msg")
                for msg in self.msgs_to_receive.keys()]

    def set_msg_files_ignore(self):
        return [os.path.join(self.msg_folder, msg + ".msg")
                for msg in self.msgs_to_ignore.keys()]

    @staticmethod
    def parse_yaml_msg_id_file(yaml_file):
        """
        Parses a yaml file into a dict
        """
        try:
            with open(yaml_file, 'r') as f:
                return yaml.load(f)
        except OSError as e:
            if e.errno == errno.ENOENT:
                raise IOError(errno.ENOENT, os.strerror(
                    errno.ENOENT), yaml_file)
            else:
                raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("-s", "--send", dest='send',
                        action="store_true", help="Get topics to be sended")
    parser.add_argument("-r", "--receive", dest='receive',
                        action="store_true", help="Get topics to be received")
    parser.add_argument("-i", "--ignore", dest='ignore',
                        action="store_true", help="Get topics to be ignored")
    parser.add_argument("-p", "--path", dest='path',
                        action="store_true", help="Get msgs and its paths")
    parser.add_argument("-m", "--topic-msg-dir", dest='msgdir', type=str,
                        help="Topics message dir, by default msg/", default="msg")
    parser.add_argument("-y", "--rtps-ids-file", dest='yaml_file', type=str,
                        help="RTPS msg IDs definition file, relative to the msg_dir, by default tools/uorb_rtps_message_ids.yaml",
                        default="tools/uorb_rtps_message_ids.yaml")

    # Parse arguments
    args = parser.parse_args()

    msg_folder = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    classifier = Classifier(os.path.join(
        msg_folder, args.yaml_file), msg_folder)

    if args.send:
        if args.path:
            print ('send files: ' + ', '.join(str(msg_file)
                                              for msg_file in classifier.msgs_files_send) + '\n')
        else:
            print (', '.join(str(msg)
                             for msg in classifier.msgs_to_send.keys()) + '\n')
    if args.receive:
        if args.path:
            print ('receive files: ' + ', '.join(str(msg_file)
                                                 for msg_file in classifier.msgs_files_receive) + '\n')
        else:
            print (', '.join(str(msg)
                             for msg in classifier.msgs_to_receive.keys()) + '\n')
    if args.ignore:
        if args.path:
            print ('ignore files: ' + ', '.join(str(msg_file)
                                                for msg_file in classifier.msgs_files_ignore) + '\n')
        else:
            print (', '.join(str(msg)
                             for msg in classifier.msgs_to_ignore.keys()) + '\n')
