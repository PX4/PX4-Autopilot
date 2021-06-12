#!/usr/bin/env python

from svginstr import *
import sys

__author__ = "Melchior FRANZ < mfranz # aon : at >"
__url__ = "http://members.aon.at/mfranz/flightgear/"
__version__ = "$Id: test.py,v 1.1 2005/11/08 21:22:33 m Exp m $; GPL v2"
__doc__ = """
"""


def make_htape(base, startval, endval, step, hscale):
	a.push("translate(%f %f)" % (0, base))

	a.at(0,0.5*hscale).rectangle(190, 0.5, color="white")
	for i in frange(startval, endval, step):
		percent = (i - startval) / (endval - startval)
		xpos = -95+(percent*190)
		a.at(xpos,1.5*hscale).rectangle(0.5, 1.6, color="white")
		a.at(xpos,3.0*hscale).text("%02d" % (i/10), size = ati_fontsize,
				dic = {'font-weight': 'bold'})
		#halfpercent = (i + (step/2) - startval) / (endval - startval)
		#xpos = 95-(halfpercent*190)
		#a.at(-1.5,xpos).rectangle(2, 0.5, color="white")

	for i in frange(startval, endval, step/2):
		percent = (i - startval) / (endval - startval)
		xpos = -95+(percent*190)
		a.at(xpos,1.5*hscale).rectangle(0.5, 1.4, color="white")

	#for i in frange(startval, endval, step/10):
	#	percent = (i - startval) / (endval - startval)
	#	xpos = -95+(percent*190)
	#	a.at(xpos,1.0*hscale).rectangle(0.5, 0.9, color="white")

	a.pop()


def make_hrule(x, y, length, hscale):
	a.push("translate(%f %f)" % (x, y))

	halflen = length * 0.5

	a.at(0,0).rectangle(length + 0.1, 0.5, color="white")
	for i in frange(0.0, 1.01, 0.5):
		percent = i
		xpos = -halflen + percent*length
		a.at(xpos,0.5*hscale).rectangle(0.5,1.1, color="white")

	for i in frange(0.0, 1.01, 0.1):
		percent = i
		xpos = -halflen + percent*length
		a.at(xpos,0.25*hscale).rectangle(0.5, 0.6, color="white")

	a.pop()


def make_vrule(x, y, length, vscale):
	a.push("translate(%f %f)" % (x, y))

	halflen = length * 0.5

	a.at(0,0).rectangle(0.5, length + 0.1, color="white")
	for i in frange(0.0, 1.01, 0.5):
		percent = i
		xpos = -halflen + percent*length
		a.at(0.5*vscale,xpos).rectangle(1.1, 0.5, color="white")

	for i in frange(0.0, 1.01, 0.1):
		percent = i
		xpos = -halflen + percent*length
		a.at(0.25*vscale,xpos).rectangle(0.6, 0.5, color="white")

	a.pop()


try:
	a = Instrument("glass-pfd-parts.svg", 1024, 1024, "glass parts; " + __version__)

	# position and sizing constants

	ati_x = 00
	ati_y = -50
	ati_size = 100
	ati_fontsize = 3/100.0*ati_size

	vsi_x = 125
	vsi_y = -50
	vsi_size = 50
	vsi_spread = 10
	vsi_fontsize = 8/100.0*vsi_size

	# roll angle indicator
	# works out to be centered @ 32,32 in a 64x64 box (pixels)

	a.push("translate(%f %f)"
	       % (-100+6.25, 100-6.25))

	a.write('<path d="M0,%f l%f,%f l%f,0 z" stroke="white" stroke-width="0.4" fill="white"/>'
		% (-0.04*ati_size, -0.02*ati_size, 0.04*ati_size,
		    0.04*ati_size) )
	a.write('<path d="M0,%f l%f,%f l%f,0 z" stroke="white" stroke-width="0.4" fill="none"/>'
		% (-0.04*ati_size, -0.04*ati_size, 0.08*ati_size,
		    0.08*ati_size) )

	a.pop()

	# ground track indicator
	# works out to be (about) :-) centered @ 32,96 in a 64x64 box (pixels)

	a.push("translate(%f %f)"
	       % (-100+6.25, 100-15.25))

	# 30 degree tics
	a.tick(-30, 0.00*ati_size, 0.07*ati_size, 0.006*ati_size,
	       color = "white")
	a.tick( 30, 0.00*ati_size, 0.07*ati_size, 0.006*ati_size,
	       color = "white")

	a.pop()

	# vbars
	# works out to be centered @ 320,32 in a 512x64 box (pixels)

	a.push("translate(%f %f)"
	       % (-100+62.5, 100-10.25))

	bdw = 32*ati_size/100.0
	bdw1 = bdw * 0.75
	bdw2 = 8*ati_size/100.0
	bdw3 = 0.4*bdw2
	bdw4 = 2 * (bdw - bdw1)
	bdw5 = 0.5*ati_size/100.0

	# vbars (left)
	a.write('<path d="M0,0 l%f,%f l%f,%f z" stroke="black" stroke-width="%f" fill="green"/>'
		% (-bdw, bdw2, -bdw2, -bdw3,
		    0.004*ati_size) )
	a.write('<path d="M%f,%f l%f,%f l0,%f z" stroke="black" stroke-width="%f" fill="green"/>'
		% (-bdw, bdw2, -bdw2, -bdw3, bdw3,
		    0.004*ati_size) )

	# vbars (right)
	a.write('<path d="M0,0 l%f,%f l%f,%f z" stroke="black" stroke-width="%f" fill="green"/>'
		% (bdw, bdw2, bdw2, -bdw3,
		    0.004*ati_size) )
	a.write('<path d="M%f,%f l%f,%f l0,%f z" stroke="black" stroke-width="%f" fill="green"/>'
		% (bdw, bdw2, bdw2, -bdw3, bdw3,
		    0.004*ati_size) )

	a.pop()

	# bug
	# works out to be centered @ 296,48 in a 32x96 box (pixels)

	a.push("translate(%f %f)"
	       % (-100+115.625, 100-9.375))

	stw1 = 5*ati_size/100.0
	a.write('<path d="M%f,0 l%f,%f l0,%f l%f,0 l0,%f l%f,0 l0,%f z" stroke="magenta" stroke-width="%f" fill="none"/>'
		% (-0.5*stw1, stw1, -stw1, -0.75*stw1, -stw1, stw1*3.5,
		    stw1, -0.75*stw1, 0.01*ati_size) )

	a.pop();


	# vsi needle
	# works out to be centered @ 736,8 in a 256x16 box (pixels)

	a.push("translate(%f %f)"
	       % (-100+143.75, 100-1.5625))

	# tip
	a.write('<path d="M%f,0 l%f,%f l0,%f z" fill="white"/>'
		% (-0.40*vsi_size, 0.04*vsi_size, -0.02*vsi_size,
		    0.04*vsi_size) )
	# body
	a.at(-0.08*vsi_size,0).rectangle(0.56*vsi_size, 0.04*vsi_size, color="white")

	a.pop();

	# heading scale
	make_htape(70, 0, 90, 10, -1)
	make_htape(60, 90, 180, 10, -1)
	make_htape(50, 180, 270, 10, -1)
	make_htape(40, 270, 360, 10, -1)

	# control surface scales
	# works out to be centered @ 102.4,537.6 in a 204.8x204.8 box (pixels)
	make_vrule(-65, -5, 30,  1)
	make_vrule(-95, -5, 30, -1)
	make_hrule(-80, 10, 30,  1)
	make_hrule(-80,-20, 30, -1)
	a.at(-70,-4).text("ELEV", size = ati_fontsize,
			  dic = {'font-weight': 'bold'})
	a.at(-91,-4).text("THR", size = ati_fontsize,
			  dic = {'font-weight': 'bold'})
	a.at(-80,-17).text("AILERON", size = ati_fontsize,
			  dic = {'font-weight': 'bold'})
	a.at(-80,9).text("RUDDER", size = ati_fontsize,
			  dic = {'font-weight': 'bold'})

	# pilot command indicator
	# works out to be centered @ 96,96 in a 64x64 box (pixels)
	a.push("translate(%f %f)"
	       % (-100+18.75, 100-18.75))
	a.write('<path d="M0,%f l%f,%f l%f,0 z" stroke="white" stroke-width="0.4" fill="red"/>'
		% (0.04*ati_size, -0.04*ati_size, -0.08*ati_size,
		    0.08*ati_size) )
	a.pop()

	# autopilot command indicator
	# works out to be centered @ 160,96 in a 64x64 box (pixels)
	a.push("translate(%f %f)"
	       % (-100+31.25, 100-18.75))
	a.write('<path d="M0,%f l%f,%f l%f,0 z" stroke="white" stroke-width="0.4" fill="blue"/>'
		% (0.04*ati_size, -0.04*ati_size, -0.08*ati_size,
		    0.08*ati_size) )
	a.pop()



except Error, e:
	print >>sys.stderr, "\033[31;1m%s\033[m\n" % e

