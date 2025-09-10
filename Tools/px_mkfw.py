# vim: set noexpandtab tabstop=4 shiftwidth=4:
#!/usr/bin/env python3
############################################################################
#
#   Copyright (C) 2012, 2013 PX4 Development Team. All rights reserved.
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
############################################################################

#
# PX4 firmware image generator
#
# The PX4 firmware file is a JSON-encoded Python object, containing
# metadata fields and a zlib-compressed base64-encoded firmware image.
#

import argparse
import json
import base64
import zlib
import time
import subprocess
import hashlib

#
# Construct a basic firmware description
#
def mkdesc():
	proto = {}
	proto['magic']		= "PX4FWv1"
	proto['board_id']	= 0
	proto['board_revision']	= 0
	proto['version']	= ""
	proto['summary']	= ""
	proto['description']	= ""
	proto['git_identity']	= "" # git tag
	proto['git_hash']	= "" # git commit hash
	proto['build_time']	= 0
	proto['image']		= bytes()
	proto['image_size']	= 0
	proto['sha256sum'] = ""
	return proto

def _merge_manifest(dst, src):
	if not isinstance(src, dict):
		return
	for k, v in src.items():
		if k == "hardware":
			dst.setdefault("hardware", {})
		if isinstance(v, dict):
			dst["hardware"].update(v)
		else:
			dst[k] = v

# Parse commandline
parser = argparse.ArgumentParser(description="Firmware generator for the PX autopilot system.")
parser.add_argument("--prototype",	action="store", help="read a prototype description from a file")
parser.add_argument("--board_id",	action="store", help="set the board ID required")
parser.add_argument("--board_revision",	action="store", help="set the board revision required")
parser.add_argument("--version",	action="store", help="set a version string")
parser.add_argument("--summary",	action="store", help="set a brief description")
parser.add_argument("--description",	action="store", help="set a longer description")
parser.add_argument("--git_identity",	action="store", help="the working directory to check for git identity")
parser.add_argument("--parameter_xml",	action="store", help="the parameters.xml file")
parser.add_argument("--airframe_xml",	action="store", help="the airframes.xml file")
parser.add_argument("--image",		action="store", help="the firmware image")
parser.add_argument("--manifest_json",	action="append", help="path to manifest JSON fragment to merge")
args = parser.parse_args()

# Fetch the firmware descriptor prototype if specified
if args.prototype != None:
	f = open(args.prototype,"r")
	desc = json.load(f)
	f.close()
else:
	desc = mkdesc()

desc.setdefault("manifest_version", 1)
desc.setdefault("manifest", {})

desc['build_time'] 		= int(time.time())

if args.board_id != None:
	desc['board_id']	= int(args.board_id)
if args.board_revision != None:
	desc['board_revision']	= int(args.board_revision)
if args.version != None:
	desc['version']		= str(args.version)
if args.summary != None:
	desc['summary']		= str(args.summary)
if args.description != None:
	desc['description']	= str(args.description)
if args.git_identity != None:
	cmd = "git --git-dir '{:}/.git' describe --exclude ext/* --always --tags".format(args.git_identity)
	p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).stdout
	desc['git_identity'] = p.read().strip().decode('utf-8')
	p.close()
	cmd = "git --git-dir '{:}/.git' rev-parse --verify HEAD".format(args.git_identity)
	p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).stdout
	desc['git_hash'] = p.read().strip().decode('utf-8')
	p.close()
if args.parameter_xml != None:
	f = open(args.parameter_xml, "rb")
	bytes = f.read()
	desc['parameter_xml_size'] = len(bytes)
	desc['parameter_xml'] = base64.b64encode(zlib.compress(bytes,9)).decode('utf-8')
	desc['mav_autopilot'] = 12 # 12 = MAV_AUTOPILOT_PX4
if args.airframe_xml != None:
	f = open(args.airframe_xml, "rb")
	bytes = f.read()
	desc['airframe_xml_size'] = len(bytes)
	desc['airframe_xml'] = base64.b64encode(zlib.compress(bytes,9)).decode('utf-8')
if args.image != None:
	f = open(args.image, "rb")
	raw_image = f.read()
	f.close()
	desc['image_size'] = len(raw_image)
	sha256sum = hashlib.sha256(raw_image).hexdigest()
	desc['sha256sum'] = sha256sum
	desc['image'] = base64.b64encode(zlib.compress(raw_image, 9)).decode('utf-8')

# merge manifest
manifest_inputs = args.manifest_json or []
if isinstance(manifest_inputs, str):
	manifest_inputs = [manifest_inputs]
for p in manifest_inputs:
	with open(p, "r", encoding="utf-8") as f:
		frag = json.load(f)
	_merge_manifest(desc["manifest"], frag)

print(json.dumps(desc, indent=4))
