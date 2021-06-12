#!/usr/bin/env python

from svginstr import *
import sys

__author__ = "Melchior FRANZ < mfranz # aon : at >"
__url__ = "http://members.aon.at/mfranz/flightgear/"
__version__ = "$Id: test.py,v 1.1 2005/11/08 21:22:33 m Exp m $; GPL v2"
__doc__ = """
"""


try:
	a = Instrument("glass-pfd-horiz.svg", 2048, 2048, "glass horiz; " + __version__)

	# position and sizing constants

	ati_x = 00
	ati_y = 00
	ati_size = 100
	ati_fontsize = 1.5/100.0*ati_size
	vertfact = 1

	# gradient definitions, applying color() methods separately ...
	sky = LinearGradient("0%", "0%", "0%", "100%")
	sky.stop("0%", 6, 15, 20)
	sky.stop("50%", 13, 30, 40)
	sky.stop("100%", 51, 132, 179)

	ground = LinearGradient("0%", "0%", "0%", "100%")
	ground.stop("0%", 216, 140, 30)
	ground.stop("50%", 78, 55, 24)
	ground.stop("100%", 39, 26, 12)


	### Attitude indicator

	a.push("translate(%f %f)" % (ati_x, ati_y))

	# sky
	a.at(0,-50*vertfact).gradient(sky).rectangle(200, 100*vertfact)

	# ground
	a.at(0,50*vertfact).gradient(ground).rectangle(200, 100*vertfact)

	# horizon
	a.at(0,0).rectangle(200, 0.25*vertfact, color="white")

	# vertical disks
	a.at(0,-90*vertfact).disc(0.8, color="white")
	a.at(0, 90*vertfact).disc(0.8, color="white")

	# pitch angle
	for i in frange(0, 90, 10):
		a.at(0,(-i-5)*vertfact).rectangle(5*vertfact, 0.25*vertfact, color="white")
		a.at(0,( i+5)*vertfact).rectangle(5*vertfact, 0.25*vertfact, color="white")

	for i in frange(10, 90, 10):
		a.at(0,(-i*vertfact)).rectangle(10*vertfact, 0.25*vertfact, color="white")
		a.at(0,( i*vertfact)).rectangle(10*vertfact, 0.25*vertfact, color="white")
		a.at(-6.5*vertfact,(-i*vertfact)+0.5*vertfact).text("%d" % (i), size = ati_fontsize,
				dic = {'font-weight': 'bold'})
		a.at( 6.5*vertfact,(-i*vertfact)+0.5*vertfact).text("%d" % (i), size = ati_fontsize,
				dic = {'font-weight': 'bold'})
		a.at(-6.5*vertfact, (i*vertfact)+0.4*vertfact).text("%d" % (-i), size = ati_fontsize,
				dic = {'font-weight': 'bold'})
		a.at( 6.5*vertfact, (i*vertfact)+0.4*vertfact).text("%d" % (-i), size = ati_fontsize,
				dic = {'font-weight': 'bold'})

	# speed tape test

	a.pop()


except Error, e:
	print >>sys.stderr, "\033[31;1m%s\033[m\n" % e

