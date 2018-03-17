#!/usr/bin/env python
#############################################################################
#
#   Copyright (C) 2013-2016 PX4 Development Team. All rights reserved.
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
px_generate_uorb_topics.py
Generates c/cpp header/source files for uorb topics from .msg (ROS syntax)
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
px4_msg_dir = os.path.join(px4_tools_dir,"../msg")

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

__author__ = "Sergey Belash, Thomas Gubler, Beat Kueng"
__copyright__ = "Copyright (C) 2013-2016 PX4 Development Team."
__license__ = "BSD"
__email__ = "thomasgubler@gmail.com"


TEMPLATE_FILE = ['msg.h.template', 'msg.cpp.template']
TOPICS_LIST_TEMPLATE_FILE = 'uORBTopics.cpp.template'
OUTPUT_FILE_EXT = ['.h', '.cpp']
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


def generate_output_from_file(format_idx, filename, outputdir, templatedir, includepath):
        """
        Converts a single .msg file to an uorb header/source file
        """
        msg_context = genmsg.msg_loader.MsgContext.create_default()
        full_type_name = genmsg.gentools.compute_full_type_name(PACKAGE, os.path.basename(filename))
        spec = genmsg.msg_loader.load_msg_from_file(msg_context, filename, full_type_name)
        topics = get_multi_topics(filename)
        if includepath:
                search_path = genmsg.command_line.includepath_to_dict(includepath)
        else:
                search_path = {}
        genmsg.msg_loader.load_depends(msg_context, spec, search_path)
        md5sum = genmsg.gentools.compute_md5(msg_context, spec)
        if len(topics) == 0:
                topics.append(spec.short_name)
        em_globals = {
            "file_name_in": filename,
            "md5sum": md5sum,
            "search_path": search_path,
            "msg_context": msg_context,
            "spec": spec,
            "topics": topics
        }

        # Make sure output directory exists:
        if not os.path.isdir(outputdir):
                os.makedirs(outputdir)

        template_file = os.path.join(templatedir, TEMPLATE_FILE[format_idx])
        output_file = os.path.join(outputdir, spec.short_name +
                OUTPUT_FILE_EXT[format_idx])

        return generate_by_template(output_file, template_file, em_globals)


def generate_by_template(output_file, template_file, em_globals):
        """
        Invokes empy intepreter to geneate output_file by the
        given template_file and predefined em_globals dict
        """
        # check if folder exists:
        folder_name = os.path.dirname(output_file)
        if not os.path.exists(folder_name):
            os.makedirs(folder_name)
            
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
        return True


def convert_dir(format_idx, inputdir, outputdir, templatedir):
        """
        Converts all .msg files in inputdir to uORB header/source files
        """

        # Find the most recent modification time in input dir
        maxinputtime = 0
        for f in os.listdir(inputdir):
                fni = os.path.join(inputdir, f)
                if os.path.isfile(fni):
                    it = os.path.getmtime(fni)
                    if it > maxinputtime:
                        maxinputtime = it

        # Find the most recent modification time in output dir
        maxouttime = 0
        if os.path.isdir(outputdir):
            for f in os.listdir(outputdir):
                    fni = os.path.join(outputdir, f)
                    if os.path.isfile(fni):
                        it = os.path.getmtime(fni)
                        if it > maxouttime:
                            maxouttime = it

        # Do not generate if nothing changed on the input
        if (maxinputtime != 0 and maxouttime != 0 and maxinputtime < maxouttime):
            return False

        includepath = INCL_DEFAULT + [':'.join([PACKAGE, inputdir])]
        for f in os.listdir(inputdir):
                # Ignore hidden files
                if f.startswith("."):
                        continue

                fn = os.path.join(inputdir, f)
                # Only look at actual files
                if not os.path.isfile(fn):
                        continue

                if fn[-4:].lower() != '.msg':
                        continue

                generate_output_from_file(format_idx, fn, outputdir, templatedir, includepath)
        return True


def copy_changed(inputdir, outputdir, prefix='', quiet=False):
        """
        Copies files from inputdir to outputdir if they don't exist in
        ouputdir or if their content changed
        """

        # Make sure output directory exists:
        if not os.path.isdir(outputdir):
                os.makedirs(outputdir)

        for input_file in os.listdir(inputdir):
                fni = os.path.join(inputdir, input_file)
                if os.path.isfile(fni):
                        # Check if input_file exists in outpoutdir, copy the file if not
                        fno = os.path.join(outputdir, prefix + input_file)
                        if not os.path.isfile(fno):
                                shutil.copy(fni, fno)
                                if not quiet:
                                    print("{0}: new header file".format(fno))
                                continue

                        if os.path.getmtime(fni) > os.path.getmtime(fno):
                                # The file exists in inputdir and outputdir
                                # only copy if contents do not match
                                if not filecmp.cmp(fni, fno):
                                        shutil.copy(fni, fno)
                                        if not quiet:
                                            print("{0}: updated".format(input_file))
                                        continue

                        if not quiet:
                            print("{0}: unchanged".format(input_file))


def convert_dir_save(format_idx, inputdir, outputdir, templatedir, temporarydir, prefix, quiet=False):
        """
        Converts all .msg files in inputdir to uORB header files
        Unchanged existing files are not overwritten.
        """
        # Create new headers in temporary output directory
        convert_dir(format_idx, inputdir, temporarydir, templatedir)
        if generate_idx == 1:
            generate_topics_list_file(inputdir, temporarydir, templatedir)
        # Copy changed headers from temporary dir to output dir
        copy_changed(temporarydir, outputdir, prefix, quiet)

def generate_topics_list_file(msgdir, outputdir, templatedir):
        # generate cpp file with topics list
        msgs = get_msgs_list(msgdir)
        multi_topics = []
        for msg in msgs:
            msg_filename = os.path.join(msgdir, msg)
            multi_topics.extend(get_multi_topics(msg_filename))
        tl_globals = {"msgs" : msgs, "multi_topics" : multi_topics}
        tl_template_file = os.path.join(templatedir, TOPICS_LIST_TEMPLATE_FILE)
        tl_out_file = os.path.join(outputdir, TOPICS_LIST_TEMPLATE_FILE.replace(".template", ""))
        generate_by_template(tl_out_file, tl_template_file, tl_globals)
        
def generate_topics_list_file_from_files(files, outputdir, templatedir):
        # generate cpp file with topics list
        filenames = [os.path.basename(p) for p in files if os.path.basename(p).endswith(".msg")]
        multi_topics = []
        for msg_filename in files:
            multi_topics.extend(get_multi_topics(msg_filename))
        tl_globals = {"msgs" : filenames, "multi_topics" : multi_topics}
        tl_template_file = os.path.join(templatedir, TOPICS_LIST_TEMPLATE_FILE)
        tl_out_file = os.path.join(outputdir, TOPICS_LIST_TEMPLATE_FILE.replace(".template", ""))
        generate_by_template(tl_out_file, tl_template_file, tl_globals)

def append_to_include_path(path_to_append, curr_include):
    for p in path_to_append:
        curr_include.append("%s:%s" % (PACKAGE, p))

if __name__ == "__main__":
        parser = argparse.ArgumentParser(
            description='Convert msg files to uorb headers/sources')
        parser.add_argument('--headers', help='Generate header files',
            action='store_true')
        parser.add_argument('--sources', help='Generate source files',
            action='store_true')
        parser.add_argument('-d', dest='dir', help='directory with msg files')
        parser.add_argument('-f', dest='file',
                            help="files to convert (use only without -d)",
                            nargs="+")
        parser.add_argument('-i', dest="include_paths",
                            help='Additional Include Paths', nargs="*",
                            default=None)
        parser.add_argument('-e', dest='templatedir',
                            help='directory with template files',)
        parser.add_argument('-o', dest='outputdir',
                            help='output directory for header files')
        parser.add_argument('-t', dest='temporarydir',
                            help='temporary directory')
        parser.add_argument('-p', dest='prefix', default='',
                            help='string added as prefix to the output file '
                            ' name when converting directories')
        parser.add_argument('-q', dest='quiet', default=False, action='store_true',
                            help='string added as prefix to the output file '
                            ' name when converting directories')
        args = parser.parse_args()

        if args.include_paths:
            append_to_include_path(args.include_paths, INCL_DEFAULT)

        if args.headers:
            generate_idx = 0
        elif args.sources:
            generate_idx = 1
        else:
            print('Error: either --headers or --sources must be specified')
            exit(-1)
        if args.file is not None:
            for f in args.file:
                generate_output_from_file(generate_idx, f, args.temporarydir, args.templatedir, INCL_DEFAULT)
            if generate_idx == 1:
                generate_topics_list_file_from_files(args.file, args.outputdir, args.templatedir)
            copy_changed(args.temporarydir, args.outputdir, args.prefix, args.quiet)
        elif args.dir is not None:
            convert_dir_save(
                    generate_idx,
                    args.dir,
                    args.outputdir,
                    args.templatedir,
                    args.temporarydir,
                    args.prefix,
                    args.quiet)
