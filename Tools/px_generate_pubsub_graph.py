#!/usr/bin/env python

import os
import glob
import re
import codecs

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

exclude_list = ['mavlink', 'logger', 'sdlog2', 'trone', 'px4flow', 'sf0x',
		'iridium', 'bst', 'pwm_out_sim', 'ulanding', 'load_mon',
		'uavcan', 'led', 'nshterm']
orb_pub_dict = {}
orb_sub_dict = {}
cmake_scope.scope = sorted(cmake_scope.scope)
for item in cmake_scope.scope:
	if [scope for scope in exclude_list if scope in item]:
		continue
	folder = "src/" + item + "/"
	files = glob.glob(folder + "*.c*")
	for file in files:
		contents = ''
		with open(file, 'r') as f:
			contents = f.read()
			f.close()
		lines = contents.split('\n')
		basename = item
		for line in lines:
			if 'orb_advertise' in line or 'orb_publish_auto' in line:
				try:
					topic = re.search('ORB_ID\((.+?)\)', line).group(1)
					if topic in orb_pub_dict:
						if basename not in orb_pub_dict[topic]:
							orb_pub_dict[topic].append(basename)
					else:
						orb_pub_dict[topic] = [basename]
				except AttributeError:
					# print(file, line, "Publish doesn't directly use an ORB_ID!")
					pass
			elif 'orb_subscribe' in line or 'orb_subscription' in line or 'orb_copy' in line:
				try:
					topic = re.search('ORB_ID\((.+?)\)', line).group(1)
					if topic in orb_sub_dict:
						if basename not in orb_sub_dict[topic]:
							orb_sub_dict[topic].append(basename)
					else:
						orb_sub_dict[topic] = [basename]
				except AttributeError:
					# print(file, line, "Subscribe doesn't directly use an ORB_ID!")
					pass


def graph(excludes, title, filename):
	graph = Digraph(comment=title, graph_attr={'splines': 'true'})
	edge_dict = {}
	for key in orb_pub_dict:
		if key in orb_sub_dict:
			pubs = orb_pub_dict[key]
			subs = orb_sub_dict[key]
			for pub in pubs:
				if [topic for topic in excludes if topic in pub]:
					continue
				for sub in subs:
					if [topic for topic in excludes if topic in sub]:
						continue
					if ((pub, sub)) in edge_dict:
						if key not in edge_dict[((pub, sub))]:
							edge_dict[((pub, sub))] += '\n'+key
					else:
						edge_dict[((pub, sub))] = key

	for ((pub, sub)) in edge_dict:
		graph.node(pub)
		graph.node(sub)
		graph.edge(pub, sub, label=edge_dict[((pub, sub))])

	graph.render(filename, view=False)

mc_topics = ['mc', 'Multi']
fw_topics = ['fw', 'Fixed', 'airspeed']
vtol_topics = ['vtol', 'Vtol']

graph(fw_topics+vtol_topics, 'MC Pub Sub Graph', 'mc_pub_sub.gv')
graph(mc_topics+vtol_topics, 'FW Pub Sub Graph', 'fw_pub_sub.gv')
graph(mc_topics+fw_topics, 'VTOL Pub Sub Graph', 'vtol_pub_sub.gv')
