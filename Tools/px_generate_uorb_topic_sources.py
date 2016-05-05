#!/usr/bin/env python
#############################################################################
#
#   Copyright (C) 2013-2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#############################################################################

"""
px_generate_uorb_topic_sources.py
Generates cpp source files for uorb topics from .msg (ROS syntax)
message files
"""
from __future__ import print_function
import os
import shutil
import filecmp
import argparse

import sys
px4_tools_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(px4_tools_dir + "/genmsg/src")
sys.path.append(px4_tools_dir + "/gencpp/src")

try:
        import em
        import genmsg.template_tools
except ImportError as e:
        print("python import error: ", e)
        print('''
Required python packages not installed.

On a Debian/Ubuntu system please run:

  sudo apt-get install python-empy
  sudo pip install catkin_pkg

On MacOS please run:
  sudo pip install empy catkin_pkg

On Windows please run:
  easy_install empy catkin_pkg
''')
        exit(1)

__author__ = "Sergey Belash, Thomas Gubler"
__copyright__ = "Copyright (C) 2013-2014 PX4 Development Team."
__license__ = "BSD"
__email__ = "againagainst@gmail.com, thomasgubler@gmail.com"

TOPIC_TEMPLATE_FILE = 'msg.cpp.template'
TOPICS_LIST_TEMPLATE_FILE = 'uORBTopics.cpp.template'
OUTPUT_FILE_EXT = '.cpp'
INCL_DEFAULT = ['std_msgs:./msg/std_msgs']
PACKAGE = 'px4'
TOPICS_TOKEN = '# TOPICS '


def get_multi_topics(filename):
        """
        Get TOPICS names from a "# TOPICS" line
        """
        ofile = open(filename, 'r')
        text = ofile.read()
        result = []
        for each_line in text.split('\n'):
                if each_line.startswith (TOPICS_TOKEN):
                        topic_names_str = each_line.strip()
                        topic_names_str = topic_names_str.replace(TOPICS_TOKEN, "")
                        result.extend(topic_names_str.split(" "))
        ofile.close()
        return result


def get_msgs_list(msgdir):
    """
    Makes list of msg files in the given directory
    """
    return [fn for fn in os.listdir(msgdir) if fn.endswith(".msg")]


def generate_source_from_file(filename, outputdir, template_file, includepath, quiet=False):
        """
        Converts a single .msg file to a uorb source file
        """
        # print("Generating sources from {0}".format(filename))
        msg_context = genmsg.msg_loader.MsgContext.create_default()
        full_type_name = genmsg.gentools.compute_full_type_name(PACKAGE, os.path.basename(filename))
        spec = genmsg.msg_loader.load_msg_from_file(msg_context, filename, full_type_name)
        topics = get_multi_topics(filename)
        if includepath:
                search_path = genmsg.command_line.includepath_to_dict(includepath)
        else:
                search_path = {}
        if len(topics) == 0:
            topics.append(spec.short_name)
        em_globals = {
            "file_name_in": filename,
            "search_path": search_path,
            "spec": spec,
            "topics": topics
        }

        output_file = os.path.join(outputdir, spec.short_name + OUTPUT_FILE_EXT)
        if os.path.isfile(output_file):
            return False
        generate_by_template(output_file, template_file, em_globals)
        if not quiet:
            print("{0}: new source file".format(output_file))
        return True


def generate_by_template(output_file, template_file, em_globals):
        """
        Invokes empy intepreter to geneate output_file by the
        given template_file and predefined em_globals dict
        """
        ofile = open(output_file, 'w')
        # todo, reuse interpreter
        interpreter = em.Interpreter(output=ofile, globals=em_globals, options={em.RAW_OPT:True,em.BUFFERED_OPT:True})
        if not os.path.isfile(template_file):
            ofile.close()
            os.remove(output_file)
            raise RuntimeError("Template file %s not found" % (template_file))
        interpreter.file(open(template_file)) #todo try
        interpreter.shutdown()
        ofile.close()


def convert_dir(msgdir, outputdir, templatedir, quiet=False):
        """
        Converts all .msg files in msgdir to uORB sources
        """
        # Make sure output directory exists:
        if not os.path.isdir(outputdir):
                os.makedirs(outputdir)
        template_file = os.path.join(templatedir, TOPIC_TEMPLATE_FILE)

        includepath = INCL_DEFAULT + [':'.join([PACKAGE, msgdir])]
        for f in os.listdir(msgdir):
                # Ignore hidden files
                if f.startswith("."):
                        continue
                fn = os.path.join(msgdir, f)
                # Only look at actual files
                if not os.path.isfile(fn):
                        continue
                generate_source_from_file(fn, outputdir, template_file, includepath, quiet)

        # generate cpp file with topics list
        tl_globals = {"msgs" : get_msgs_list(msgdir)}
        tl_template_file = os.path.join(templatedir, TOPICS_LIST_TEMPLATE_FILE)
        tl_out_file = os.path.join(outputdir, TOPICS_LIST_TEMPLATE_FILE.replace(".template", ""))
        generate_by_template(tl_out_file, tl_template_file, tl_globals)
        return True


if __name__ == "__main__":
        parser = argparse.ArgumentParser(description='Convert msg files to uorb sources')
        parser.add_argument('-d', dest='dir', help='directory with msg files')
        parser.add_argument('-f', dest='file', help="files to convert (use only without -d)", nargs="+")
        parser.add_argument('-e', dest='templatedir', help='directory with template files',)
        parser.add_argument('-o', dest='outputdir', help='output directory for source files')
        parser.add_argument('-q', dest='quiet', default=False, action='store_true',
                            help='string added as prefix to the output file '
                            ' name when converting directories')
        args = parser.parse_args()

        if args.file is not None:
                for f in args.file:
                        generate_source_from_file(f, args.outputdir, args.templatedir, args.quiet)
        elif args.dir is not None:
                convert_dir(args.dir, args.outputdir, args.templatedir, args.quiet)
