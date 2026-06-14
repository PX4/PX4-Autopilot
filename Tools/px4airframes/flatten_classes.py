#!/usr/bin/env python3
"""Flatten ROMFS airframe class subdirectories.

Airframes are organized in the source tree by vehicle class
(init.d/airframes/<class>/<file>) so contributors classify an airframe by where
they file it. The runtime, however, loads airframes flat from
/etc/init.d/airframes/<file>. Once the airframe metadata has been generated
(which reads the class from the directory), move every airframe up into the
airframes/ root and remove the now-empty class directories so the on-target
layout is unchanged.

Files already sitting flat in the airframes/ root (e.g. board-claimed frames)
are left untouched.
"""
import os
import shutil
import sys


def flatten(airframes_dir):
    for entry in sorted(os.listdir(airframes_dir)):
        class_dir = os.path.join(airframes_dir, entry)
        if not os.path.isdir(class_dir):
            continue
        for name in os.listdir(class_dir):
            dst = os.path.join(airframes_dir, name)
            if os.path.exists(dst):
                sys.exit("flatten_classes: refusing to overwrite '%s'" % dst)
            shutil.move(os.path.join(class_dir, name), dst)
        os.rmdir(class_dir)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: flatten_classes.py <airframes_dir>")
    flatten(sys.argv[1])
