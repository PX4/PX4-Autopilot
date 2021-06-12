#!/usr/bin/env python

from svginstr import *
import sys

__author__ = "Melchior FRANZ < mfranz # aon : at >"
__url__ = "http://members.aon.at/mfranz/flightgear/"
__version__ = "$Id: test.py,v 1.1 2005/11/08 21:22:33 m Exp m $; GPL v2"
__doc__ = """
"""


def make_tape(base, startval, endval, step, hscale):
	a.push("translate(%f %f)" % (base, 0))

	a.at(0.5*hscale,0).rectangle(0.5, 380, color="white")
	for i in frange(startval, endval, step):
		percent = (i - startval) / (endval - startval)
		xpos = 190-(percent*380)
		a.at(2.0*hscale,xpos).rectangle(3, 0.5, color="white")
		nudge = 0
		if i > -100000 and i < 100000:
			nudge = 0.8*hscale
		if i > -10000 and i < 10000:
			nudge = 0.1*hscale
		if i > -1000 and i < 1000:
			nudge = -0.6*hscale
		if i > -100 and i < 100:
			nudge = -1.3*hscale
		if i > -10 and i < 10:
			nudge = -2*hscale
		a.at(7.0*hscale + nudge, xpos+1).text("%d" % (i), size = ati_fontsize,
				dic = {'font-weight': 'bold'})
		#halfpercent = (i + (step/2) - startval) / (endval - startval)
		#xpos = 190-(halfpercent*380)
		#a.at(-1.5,xpos).rectangle(2, 0.5, color="white")

	for i in frange(startval, endval, step/2):
		percent = (i - startval) / (endval - startval)
		xpos = 190-(percent*380)
		a.at(1.5*hscale,xpos).rectangle(2, 0.5, color="white")

	for i in frange(startval, endval, step/10):
		percent = (i - startval) / (endval - startval)
		xpos = 190-(percent*380)
		a.at(1.0*hscale,xpos).rectangle(1, 0.5, color="white")

	a.pop()


try:
	a = Instrument("glass-pfd-tapes.svg", 1024, 2048, "glass tapes; " + __version__)

	# position and sizing constants

	ati_x = 00
	ati_y = 00
	ati_size = 100
	ati_fontsize = 3/100.0*ati_size

	a.push("translate(%f %f)" % (ati_x, ati_y))

	# speed tape
	make_tape(-85, 0, 200, 10, -1)

	#base = -90
	#a.at(base-0.5,0).rectangle(0.5, 380, color="white")
	#for i in frange(0, 190, 10):
	#	a.at(base-2.0,190-(i*2)).rectangle(3, 0.5, color="white")
	#	a.at(base-1.5,180-(i*2)).rectangle(2, 0.5, color="white")
	#	a.at(base-7.0,190-(i*2)+1).text("%d" % (i), size = ati_fontsize,
	#			dic = {'font-weight': 'bold'})

	#for i in frange(0, 190, 1):
	#	a.at(base-1.0,190-(i*2)).rectangle(1, 0.5, color="white")

	# altitude tapes
	make_tape(-80, 0, 2000, 100, 1)
	make_tape(-65, 2000, 4000, 100, 1)
	make_tape(-50, 4000, 6000, 100, 1)
	make_tape(-35, 6000, 8000, 100, 1)
	make_tape(-20, 8000, 10000, 100, 1)
	make_tape( -5, 10000, 12000, 100, 1)
	make_tape( 10, 12000, 14000, 100, 1)
	make_tape( 25, 14000, 16000, 100, 1)
	make_tape( 40, 16000, 18000, 100, 1)
	make_tape( 55, 18000, 20000, 100, 1)
	make_tape( 70, 22000, 24000, 100, 1)
	make_tape( 85, 26000, 28000, 100, 1)

	a.pop()


except Error, e:
	print >>sys.stderr, "\033[31;1m%s\033[m\n" % e


