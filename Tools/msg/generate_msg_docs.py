#!/usr/bin/env python3

"""
Generate docs from .msg files
"""

import os
import argparse
import sys


def get_msgs_list(msgdir):
    """
    Makes list of msg files in the given directory
    """
    return [fn for fn in os.listdir(msgdir) if fn.endswith(".msg")]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate docs from .msg files')
    parser.add_argument('-d', dest='dir', help='output directory', required=True)
    args = parser.parse_args()

    output_dir = args.dir
    if not os.path.isdir(output_dir):
        os.mkdir(output_dir)

    msg_path = os.path.join(os.path.dirname(os.path.realpath(__file__)),"../../msg")
    msg_files = get_msgs_list(msg_path)
    msg_files.sort()

    filelist_in_markdown=''

    for msg_file in msg_files:
        msg_name = os.path.splitext(msg_file)[0]
        output_file = os.path.join(output_dir, msg_name+'.md')
        msg_filename = os.path.join(msg_path, msg_file)
        print("{:} -> {:}".format(msg_filename, output_file))

        #Format msg url
        msg_url="[source file](https://github.com/PX4/PX4-Autopilot/blob/main/msg/%s)" % msg_file

        msg_description = ""
        summary_description = ""

        #Get msg description (first non-empty comment line from top of msg)
        with open(msg_filename, 'r') as lineparser:
            line = lineparser.readline()
            while line.startswith('#') or (line.strip() == ''):
                print('DEBUG: line: %s' % line)
                line=line[1:].strip()+'\n'
                stripped_line=line.strip()
                if msg_description and not summary_description and stripped_line=='':
                    summary_description = msg_description.strip()

                msg_description+=line
                line = lineparser.readline()
            msg_description=msg_description.strip()
            if not summary_description and msg_description:
                summary_description = msg_description
            print('msg_description: Z%sZ' % msg_description)
            print('summary_description: Z%sZ' % summary_description)
            summary_description
        msg_contents = ""
        #Get msg contents (read the file)
        with open(msg_filename, 'r') as source_file:
            msg_contents = source_file.read()

        #Format markdown using msg name, comment, url, contents.
        markdown_output="""# %s (UORB message)

%s

%s

```c
%s
```
""" % (msg_name, msg_description, msg_url, msg_contents)

        with open(output_file, 'w') as content_file:
            content_file.write(markdown_output)

        index_markdown_file_link='- [%s](%s.md)' % (msg_name,msg_name)
        if summary_description:
            index_markdown_file_link+=" — %s" % summary_description
        filelist_in_markdown+=index_markdown_file_link+"\n"

    # Write out the index.md file
    index_text="""# uORB Message Reference

::: info
This list is [auto-generated](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/msg/generate_msg_docs.py) from the source code.
:::

This topic lists the UORB messages available in PX4 (some of which may be may be shared by the [PX4-ROS 2 Bridge](../ros/ros2_comm.md)).
Graphs showing how these are used [can be found here](../middleware/uorb_graph.md).

%s
    """ % (filelist_in_markdown)
    index_file = os.path.join(output_dir, 'index.md')
    with open(index_file, 'w') as content_file:
            content_file.write(index_text)
