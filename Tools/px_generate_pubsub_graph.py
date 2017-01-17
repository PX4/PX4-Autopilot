#!/usr/bin/env python

import os
import glob
import re
import codecs
import json

from graphviz import Digraph
from px4params import scope, cmakeparser

if len(os.sys.argv) < 2:
	print("Error in %s" % os.sys.argv[0])
	print("Usage: %s <cmake-file-scoping> " % os.sys.argv[0])
	raise SystemExit

cmake_scope = scope.Scope()
with codecs.open(os.sys.argv[1], 'r', 'utf-8') as f:
	try:
		contents = f.read()
		f.close()
		parser = cmakeparser.CMakeParser()
		parser.Parse(cmake_scope, contents)
	except:
		contents = ''
		print('Failed reading file: %s, skipping scoping.' % os.sys.argv[2])
		pass

orb_pub_dict = {}
orb_sub_dict = {}
cmake_scope.scope = sorted(cmake_scope.scope)
for item in cmake_scope.scope:
	folder = "src/" + item + "/"
	files = glob.glob(folder + "*.c*")
	for file in files:
		contents = ''
		with open(file, 'r') as f:
			contents = f.read()
			f.close()
		lines = contents.split('\n')
		basename = os.path.basename(file)
		for line in lines:
			if "orb_advertise" in line or "orb_publish_auto" in line:
				try:
					topic = re.search('ORB_ID\((.+?)\)', line).group(1)
					if topic in orb_pub_dict:
						if basename not in orb_pub_dict[topic]:
							orb_pub_dict[topic].append(basename)
					else:
						orb_pub_dict[topic] = [basename]
				except AttributeError:
					#print(file, line, "Publish doesn't directly use an ORB_ID!")
					pass
			elif "orb_subscribe" in line or "orb_subscription" in line:
				try:
					topic = re.search('ORB_ID\((.+?)\)', line).group(1)
					if topic in orb_sub_dict:
						if basename not in orb_sub_dict[topic]:
							orb_sub_dict[topic].append(basename)
					else:
						orb_sub_dict[topic] = [basename]
				except AttributeError:
					#print(file, line, "Subscribe doesn't directly use an ORB_ID!")
					pass

#for key in sorted(orb_sub_dict):
#	print(key + ": ")
#	for sub in orb_sub_dict[key]:
#		print("\t"+sub)

dot = Digraph(comment='Pub Sub Graph')
pub_sub_graph = {}
pub_sub_json = {}
for key in orb_pub_dict:
	if key in orb_sub_dict:
		pub_sub_json[key] = ((orb_pub_dict[key], orb_sub_dict[key]))
		#print(key + ": " + str(orb_pub_dict[key]))
		#print("\t"+str(orb_sub_dict[key]))
		# There is a pub sub link
		for pub in orb_pub_dict[key]:
			dot.node(pub)
			for sub in orb_sub_dict[key]:
				dot.edge(pub, sub, label=key)

print(json.dumps(pub_sub_json, sort_keys=True, indent=4, separators=(',', ': ')))
#dot.render('test.gv', view=True)
#print(dot.source)
#plt.show()
