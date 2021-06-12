#!/usr/bin/env python

from svginstr import *
import sys

__author__ = "Melchior FRANZ < mfranz # aon : at >"
__url__ = "http://members.aon.at/mfranz/flightgear/"
__version__ = "$Id: test.py,v 1.1 2005/11/08 21:22:33 m Exp m $; GPL v2"
__doc__ = """
"""


try:
	a = Instrument("glass-pfd-main.svg", 1024, 1024, "glass face; " + __version__)

	# position and sizing constants

	ati_x = 00
	ati_y = -50
	ati_size = 50

	vsi_x = 125
	vsi_y = -50
	vsi_size = 50
	vsi_spread = 10
	vsi_fontsize = 8/100.0*vsi_size


	### Attitude indicator

	a.push("translate(%f %f)" % (ati_x, ati_y))

	# temp background for alignment
	# a.disc(0.9*ati_size, color = 'orange')

	# center white dot
	# a.disc(0.01*ati_size)

	# top white triangle
	a.write('<path d="M0,%f l%f,%f l%f,0 z" fill="white"/>'
		% (-0.80*ati_size, -0.04*ati_size, -0.08*ati_size,
		    0.08*ati_size) )

	# roll angle indicator
	#a.write('<path d="M0,%f l%f,%f l%f,0 z" stroke="white" stroke-width="0.4" fill="white"/>'
	#	% (-0.80*ati_size, -0.02*ati_size, 0.04*ati_size,
	#	    0.04*ati_size) )
	#a.write('<path d="M0,%f l%f,%f l%f,0 z" stroke="white" stroke-width="0.4" fill="none"/>'
	#	% (-0.80*ati_size, -0.04*ati_size, 0.08*ati_size,
	#	    0.08*ati_size) )

	# 10 degree tics
	a.tick(-10, 0.80*ati_size, 0.85*ati_size, 0.01*ati_size,
	       color = "white")
	a.tick( 10, 0.80*ati_size, 0.85*ati_size, 0.01*ati_size,
	       color = "white")

	# 20 degree tics
	a.tick(-20, 0.80*ati_size, 0.85*ati_size, 0.01*ati_size,
	       color = "white")
	a.tick( 20, 0.80*ati_size, 0.85*ati_size, 0.01*ati_size,
	       color = "white")

	# 30 degree tics
	a.tick(-30, 0.80*ati_size, 0.88*ati_size, 0.01*ati_size,
	       color = "white")
	a.tick( 30, 0.80*ati_size, 0.88*ati_size, 0.01*ati_size,
	       color = "white")

	# 45 degree tics
	a.tick(-45, 0.80*ati_size, 0.85*ati_size, 0.01*ati_size,
	       color = "white")
	a.tick( 45, 0.80*ati_size, 0.85*ati_size, 0.01*ati_size,
	       color = "white")

	# 60 degree tics
	a.tick(-60, 0.80*ati_size, 0.85*ati_size, 0.01*ati_size,
	       color = "white")
	a.tick( 60, 0.80*ati_size, 0.85*ati_size, 0.01*ati_size,
	       color = "white")

	# bird

	bdw = 32*ati_size/100.0
	bdw1 = bdw * 0.75
	bdw2 = 8*ati_size/100.0
	bdw3 = 0.4*bdw2
	bdw4 = 2 * (bdw - bdw1)
	bdw5 = 0.6*ati_size/100.0

	a.write('<path d="M0,0 l%f,%f l%f,0 l0,%f l%f,0 l0,%f l%f,0 l%f,%f z" stroke="black" stroke-width="%f" fill="yellow"/>'
		% (-bdw, bdw2, bdw1, -bdw3, bdw4, bdw3, bdw1, -bdw, -bdw2,
		    0.004*ati_size) )

	# vbars (left)
	#a.write('<path d="M0,0 l%f,%f l%f,%f z" stroke="black" stroke-width="%f" fill="green"/>'
	#	% (-bdw, bdw2, -bdw2, -bdw3,
	#	    0.004*ati_size) )
	#a.write('<path d="M%f,%f l%f,%f l0,%f z" stroke="black" stroke-width="%f" fill="green"/>'
	#	% (-bdw, bdw2, -bdw2, -bdw3, bdw3,
	#	    0.004*ati_size) )

	# vbars (right)
	#a.write('<path d="M0,0 l%f,%f l%f,%f z" stroke="black" stroke-width="%f" fill="green"/>'
	#	% (bdw, bdw2, bdw2, -bdw3,
	#	    0.004*ati_size) )
	#a.write('<path d="M%f,%f l%f,%f l0,%f z" stroke="black" stroke-width="%f" fill="green"/>'
	#	% (bdw, bdw2, bdw2, -bdw3, bdw3,
	#	    0.004*ati_size) )

	# horizon markers
	a.write('<path d="M%f,%f l%f,0 l0,%f l%f,0 z" stroke="black" stroke-width="%f" fill="yellow"/>'
		% (-bdw, -bdw5, -bdw2, 2*bdw5, bdw2,
		    0.004*ati_size) )
	a.write('<path d="M%f,%f l%f,0 l0,%f l%f,0 z" stroke="black" stroke-width="%f" fill="yellow"/>'
		% (bdw, -bdw5, bdw2, 2*bdw5, -bdw2,
		    0.004*ati_size) )
	
	speed_tape_width = 25
	stw = speed_tape_width*ati_size/100.0
	stw1 = 5*ati_size/100.0
	stw2 = 2*stw1
	stw3 = stw - stw1

	# left speed tape
	a.write('<path d="M%f,%f l%f,0 l0,%f l%f,0 z" stroke="none" stroke-width="1" fill="black" opacity="0.10"/>'
		% (-0.85*ati_size, -0.85*ati_size, -stw, 1.70*ati_size, stw) )
	a.write('<path d="M%f,0 l%f,%f l%f,0 l0,%f l%f,0 l%f,%f z" stroke="white" stroke-width="%f" fill="black"/>'
		% (-0.85*ati_size, -stw1, stw1, -stw3, -stw2, stw3,
		    stw1, stw1, 0.005*ati_size) )

	# right altitude tape
	a.write('<path d="M%f,%f l%f,0 l0,%f l%f,0 z" stroke="none" stroke-width="1" fill="black" opacity="0.10"/>'
		% (0.85*ati_size, -0.85*ati_size, stw, 1.70*ati_size,
		   -stw) )
	a.write('<path d="M%f,0 l%f,%f l%f,0 l0,%f l%f,0 l%f,%f z" stroke="white" stroke-width="%f" fill="black"/>'
		% (0.85*ati_size, stw1, stw1, stw3, -stw2, -stw3,
		    -stw1, stw1, 0.005*ati_size) )

	# bug
	#a.write('<path d="M%f,0 l%f,%f l0,%f l%f,0 l0,%f l%f,0 l0,%f z" stroke="magenta" stroke-width="%f" fill="none"/>'
	#	% (0.85*ati_size, stw1, -stw1, -0.75*stw1, -stw1, stw1*3.5,
	#	    stw1, -0.75*stw1, 0.005*ati_size) )

	a.pop()


	### VSI

	a.push("translate(%f %f)" % (vsi_x, vsi_y))

	# temp background for alignment
	# a.disc(1.10*vsi_size, color = 'black')

	# frame arc
	a.arc(-95 - 3*vsi_spread, -85 + 3*vsi_spread, 1.08*vsi_size,
	      0.01*vsi_size, color = "white");

	" define mapping function: map angle value for convenience "
	a.angle = lambda x: x*vsi_spread - 180

	# major tics
	for i in frange(-3, 4, 1):
	        a.tick(i, 1.00*vsi_size, 1.08*vsi_size, 0.01*vsi_size)

	" define mapping function: map angle value for convenience "
	a.angle = lambda x: x*vsi_spread/5 - 180

	# minor tics
	for i in frange(-10, 11, 1):
		a.tick(i, 1.03*vsi_size, 1.08*vsi_size, 0.006*vsi_size)

	a.tick(-12.5, 1.03*vsi_size, 1.08*vsi_size, 0.006*vsi_size)
	a.tick(12.5, 1.03*vsi_size, 1.08*vsi_size, 0.006*vsi_size)

	# labels
	a.at(-1.12*vsi_size, 0.03*vsi_size).text("0", vsi_fontsize,
		dic = {'font-weight': 'bold'})
	a.at(-1.11*vsi_size, -0.17*vsi_size).text("5", vsi_fontsize,
		dic = {'font-weight': 'bold'})
	a.at(-1.08*vsi_size, -0.36*vsi_size).text("10", vsi_fontsize,
		dic = {'font-weight': 'bold'})
	a.at(-1.00*vsi_size, -0.54*vsi_size).text("20", vsi_fontsize,
		dic = {'font-weight': 'bold'})

	a.at(-1.12*vsi_size, 0.23*vsi_size).text("-5", vsi_fontsize,
		dic = {'font-weight': 'bold'})
	a.at(-1.09*vsi_size, 0.42*vsi_size).text("-10", vsi_fontsize,
		dic = {'font-weight': 'bold'})
	a.at(-1.01*vsi_size, 0.60*vsi_size).text("-20", vsi_fontsize,
		dic = {'font-weight': 'bold'})

	# vsi needle

	# tip
	#a.write('<path d="M%f,0 l%f,%f l0,%f z" fill="white"/>'
	#	% (-1.00*vsi_size, 0.02*vsi_size, -0.01*vsi_size,
	#	    0.02*vsi_size) )

	# body
	#a.at(-0.78*vsi_size,0).rectangle(0.40*vsi_size, 0.02*vsi_size, color="white")

	a.pop()


except Error, e:
	print >>sys.stderr, "\033[31;1m%s\033[m\n" % e

