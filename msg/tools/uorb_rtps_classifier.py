#!/usr/bin/env python3
################################################################################
#
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

import argparse
import difflib
import errno
import os
from typing import Dict, List, Tuple
import yaml


class Classifier():
    """Class to classify RTPS msgs as to send, receive and set their IDs."""

    def __init__(self, yaml_file, msg_folder) -> None:
        self.msg_folder = msg_folder
        self.msg_map = self.parse_yaml_msgs_file(yaml_file)

        # Check if base types are defined correctly
        self.check_base_type()

        # Get messages to send and to receive
        self.msgs_to_send: List[Tuple[str, bool, float]] = []
        self.msgs_to_receive: Dict[str, int] = dict()
        self.alias_msgs_to_send: List[Tuple[str, str, bool, float]] = []
        self.alias_msgs_to_receive: List[Tuple[str, str]] = []
        self.msg_list: List[str] = []

        # Create message map
        self.setup_msg_map()

        self.msg_files_send = self.set_msg_files_send()
        self.msg_files_receive = self.set_msg_files_receive()

    def setup_msg_map(self) -> None:
        """Setup dictionary with an ID map for the messages."""
        for topic in self.msg_map['rtps']:
            if 'send' in list(topic.keys()):
                poll = topic.get('poll', False)
                poll_interval = topic['poll_interval'] if 'poll_interval' in list(topic.keys()) else 0.0
                if 'base' in list(topic.keys()):
                    self.alias_msgs_to_send.append(
                        (topic['msg'], topic['base'], poll, poll_interval))
                else:
                    self.msgs_to_send.append((topic['msg'], poll, poll_interval))
            if 'receive' in list(topic.keys()):
                if 'base' in list(topic.keys()):
                    self.alias_msgs_to_receive.append(
                        (topic['msg'], topic['base']))
                else:
                    self.msgs_to_receive.update({topic['msg']: 0})
            self.msg_list.append(topic['msg'])

    def set_msg_files_send(self) -> list:
        """
        Append the path to the files which messages are marked to
        be sent.
        """
        return [os.path.join(self.msg_folder, msg[0] + ".msg")
                for msg in self.msgs_to_send]

    def set_msg_files_receive(self) -> list:
        """
        Append the path to the files which messages are marked to
        be received.
        """
        return [os.path.join(self.msg_folder, msg + ".msg")
                for msg in list(self.msgs_to_receive.keys())]

    def check_base_type(self) -> None:
        """Check if alias message has correct base type."""
        registered_alias_msgs = list(
            topic['base'] for topic in self.msg_map['rtps'] if 'base' in list(topic.keys()))

        base_types = []
        for topic in self.msg_map['rtps']:
            if 'base' not in list(topic.keys()):
                base_types.append(topic['msg'])

        incorrect_base_types = list(
            set(registered_alias_msgs) - set(base_types))

        base_types_suggestion = {}
        for incorrect in incorrect_base_types:
            base_types_suggestion.update({incorrect: difflib.get_close_matches(
                incorrect, base_types, n=1, cutoff=0.6)})

        if len(base_types_suggestion) > 0:
            raise AssertionError(
                ('\n' + '\n'.join('\t- The multi-topic message base type \'{}\' does not exist.{}'.format(k, (' Did you mean \'' + v[0] + '\'?' if v else '')) for k, v in list(base_types_suggestion.items()))))

    @staticmethod
    def parse_yaml_msgs_file(yaml_file) -> dict:
        """Parses a yaml file into a dict."""
        try:
            with open(yaml_file, 'r') as file:
                return yaml.safe_load(file)
        except OSError as err:
            if err.errno == errno.ENOENT:
                raise IOError(errno.ENOENT, os.strerror(
                    errno.ENOENT), yaml_file)
            raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("-s", "--send", dest='send',
                        action="store_true", help="Get topics to be sent")
    parser.add_argument("-a", "--alias", dest='alias',
                        action="store_true", help="Get alias topics")
    parser.add_argument("-r", "--receive", dest='receive',
                        action="store_true", help="Get topics to be received")
    parser.add_argument("-i", "--ignore", dest='ignore',
                        action="store_true", help="Get topics to be ignored")
    parser.add_argument("-p", "--path", dest='path',
                        action="store_true", help="Get msgs and its paths")
    parser.add_argument("-m", "--topic-msg-dir", dest='msgdir', type=str,
                        help="Topics message dir, by default msg/", default="msg")
    parser.add_argument("-y", "--rtps-ids-file", dest='yaml_file', type=str,
                        help="RTPS msg IDs definition file absolute path, by default use relative path to msg, tools/urtps_bridge_topics.yaml",
                        default='tools/urtps_bridge_topics.yaml')

    # Parse arguments
    args = parser.parse_args()

    msg_dir = args.msgdir
    if args.msgdir == 'msg':
        msg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    else:
        msg_dir = os.path.abspath(args.msgdir)
    classifier = (Classifier(os.path.abspath(args.yaml_file), msg_dir) if os.path.isabs(args.yaml_file)
                  else Classifier(os.path.join(msg_dir, args.yaml_file), msg_dir))

    if args.send:
        if args.path:
            print(('send files: ' + ', '.join(str(msg_file)
                                              for msg_file in classifier.msg_files_send) + '\n'))
        else:
            if args.alias:
                print((', '.join(str(msg[0])
                                 for msg in sorted(classifier.msgs_to_send)) + (' alias ' + ', '.join(msg[0]
                                                                                                      for msg in classifier.alias_msgs_to_send) if len(classifier.alias_msgs_to_send) > 0 else '') + '\n'))
            else:
                print((', '.join(str(msg[0])
                                 for msg in sorted(classifier.msgs_to_send))))
    if args.receive:
        if args.path:
            print(('receive files: ' + ', '.join(str(msg_file)
                                                 for msg_file in classifier.msg_files_receive) + '\n'))
        else:
            if args.alias:
                print((', '.join(str(msg)
                                 for msg in sorted(classifier.msgs_to_receive)) + (' alias ' + ', '.join(msg[0]
                                                                                                         for msg in classifier.alias_msgs_to_receive) if len(classifier.alias_msgs_to_receive) > 0 else '') + '\n'))
            else:
                print((', '.join(str(msg)
                                 for msg in sorted(classifier.msgs_to_receive))))
