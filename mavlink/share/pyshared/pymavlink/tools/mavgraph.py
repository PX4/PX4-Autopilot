#!/usr/bin/env python
'''
graph a MAVLink log file
Andrew Tridgell August 2011
'''

import sys, struct, time, os, datetime
import math, re
import pylab, matplotlib
from math import *

from pymavlink.mavextra import *

locator = None
formatter = None

colourmap = {
    'apm' : {
        'MANUAL'    : (1.0,   0,   0),
        'AUTO'      : (  0, 1.0,   0),
        'LOITER'    : (  0,   0, 1.0),
        'FBWA'      : (1.0, 0.5,   0),
        'RTL'       : (  1,   0, 0.5),
        'STABILIZE' : (0.5, 1.0,   0),
        'LAND'      : (  0, 1.0, 0.5),
        'STEERING'  : (0.5,   0, 1.0),
        'HOLD'      : (  0, 0.5, 1.0),
        'ALT_HOLD'  : (1.0, 0.5, 0.5),
        'CIRCLE'    : (0.5, 1.0, 0.5),
        'POSITION'  : (1.0, 0.0, 1.0),
        'GUIDED'    : (0.5, 0.5, 1.0),
        'ACRO'      : (1.0, 1.0,   0),
        'CRUISE'    : (  0, 1.0, 1.0)
        }
    }

edge_colour = (0.1, 0.1, 0.1)

def plotit(x, y, fields, colors=[]):
    '''plot a set of graphs using date for x axis'''
    global locator, formatter
    pylab.ion()
    fig = pylab.figure(num=1, figsize=(12,6))
    ax1 = fig.gca()
    ax2 = None
    xrange = 0.0
    for i in range(0, len(fields)):
        if len(x[i]) == 0: continue
        if x[i][-1] - x[i][0] > xrange:
            xrange = x[i][-1] - x[i][0]
    xrange *= 24 * 60 * 60
    if formatter is None:
        formatter = matplotlib.dates.DateFormatter('%H:%M:%S')
        interval = 1
        intervals = [ 1, 2, 5, 10, 15, 30, 60, 120, 240, 300, 600,
                      900, 1800, 3600, 7200, 5*3600, 10*3600, 24*3600 ]
        for interval in intervals:
            if xrange / interval < 15:
                break
        locator = matplotlib.dates.SecondLocator(interval=interval)
    if not opts.xaxis:
        ax1.xaxis.set_major_locator(locator)
        ax1.xaxis.set_major_formatter(formatter)
    empty = True
    ax1_labels = []
    ax2_labels = []
    for i in range(0, len(fields)):
        if len(x[i]) == 0:
            print("Failed to find any values for field %s" % fields[i])
            continue
        if i < len(colors):
            color = colors[i]
        else:
            color = 'red'
        (tz, tzdst) = time.tzname
        if axes[i] == 2:
            if ax2 == None:
                ax2 = ax1.twinx()
            ax = ax2
            if not opts.xaxis:
                ax2.xaxis.set_major_locator(locator)
                ax2.xaxis.set_major_formatter(formatter)
            label = fields[i]
            if label.endswith(":2"):
                label = label[:-2]
            ax2_labels.append(label)
        else:
            ax1_labels.append(fields[i])
            ax = ax1
        if opts.xaxis:
            if opts.marker is not None:
                marker = opts.marker
            else:
                marker = '+'
            if opts.linestyle is not None:
                linestyle = opts.linestyle
            else:
                linestyle = 'None'
            ax.plot(x[i], y[i], color=color, label=fields[i],
                    linestyle=linestyle, marker=marker)
        else:
            if opts.marker is not None:
                marker = opts.marker
            else:
                marker = 'None'
            if opts.linestyle is not None:
                linestyle = opts.linestyle
            else:
                linestyle = '-'
            ax.plot_date(x[i], y[i], color=color, label=fields[i],
                         linestyle=linestyle, marker=marker, tz=None)
        pylab.draw()
        empty = False
    if opts.flightmode is not None:
        for i in range(len(modes)-1):
            c = colourmap[opts.flightmode].get(modes[i][1], edge_colour) 
            ax1.axvspan(modes[i][0], modes[i+1][0], fc=c, ec=edge_colour, alpha=0.1)
        c = colourmap[opts.flightmode].get(modes[-1][1], edge_colour)
        ax1.axvspan(modes[-1][0], ax1.get_xlim()[1], fc=c, ec=edge_colour, alpha=0.1)
    if ax1_labels != []:
        ax1.legend(ax1_labels,loc=opts.legend)
    if ax2_labels != []:
        ax2.legend(ax2_labels,loc=opts.legend2)
    if empty:
        print("No data to graph")
        return


from optparse import OptionParser
parser = OptionParser("mavgraph.py [options] <filename> <fields>")

parser.add_option("--no-timestamps",dest="notimestamps", action='store_true', help="Log doesn't have timestamps")
parser.add_option("--planner",dest="planner", action='store_true', help="use planner file format")
parser.add_option("--condition",dest="condition", default=None, help="select packets by a condition")
parser.add_option("--labels",dest="labels", default=None, help="comma separated field labels")
parser.add_option("--legend",  default='upper left', help="default legend position")
parser.add_option("--legend2",  default='upper right', help="default legend2 position")
parser.add_option("--marker",  default=None, help="point marker")
parser.add_option("--linestyle",  default=None, help="line style")
parser.add_option("--xaxis",  default=None, help="X axis expression")
parser.add_option("--zero-time-base",  action='store_true', help="use Z time base for DF logs")
parser.add_option("--flightmode", default=None,
                    help="Choose the plot background according to the active flight mode of the specified type, e.g. --flightmode=apm for ArduPilot logs.  Cannot be specified with --xaxis.")
(opts, args) = parser.parse_args()

from pymavlink import mavutil

if len(args) < 2:
    print("Usage: mavlogdump.py [options] <LOGFILES...> <fields...>")
    sys.exit(1)

if opts.flightmode is not None and opts.xaxis:
    print("Cannot request flightmode backgrounds with an x-axis expression")
    sys.exit(1)

if opts.flightmode is not None and opts.flightmode not in colourmap:
    print("Unknown flight controller '%s' in specification of --flightmode" % opts.flightmode)
    sys.exit(1)

filenames = []
fields = []
for f in args:
    if os.path.exists(f):
        filenames.append(f)
    else:
        fields.append(f)
msg_types = set()
multiplier = []
field_types = []

colors = [ 'red', 'green', 'blue', 'orange', 'olive', 'black', 'grey', 'yellow' ]

# work out msg types we are interested in
x = []
y = []
modes = []
axes = []
first_only = []
re_caps = re.compile('[A-Z_][A-Z0-9_]+')
for f in fields:
    caps = set(re.findall(re_caps, f))
    msg_types = msg_types.union(caps)
    field_types.append(caps)
    y.append([])
    x.append([])
    axes.append(1)
    first_only.append(False)

def add_data(t, msg, vars, flightmode):
    '''add some data'''
    mtype = msg.get_type()
    if opts.flightmode is not None and (len(modes) == 0 or modes[-1][1] != flightmode):
        modes.append((t, flightmode))
    if mtype not in msg_types:
        return
    for i in range(0, len(fields)):
        if mtype not in field_types[i]:
            continue
        f = fields[i]
        if f.endswith(":2"):
            axes[i] = 2
            f = f[:-2]
        if f.endswith(":1"):
            first_only[i] = True
            f = f[:-2]
        v = mavutil.evaluate_expression(f, vars)
        if v is None:
            continue
        if opts.xaxis is None:
            xv = t
        else:
            xv = mavutil.evaluate_expression(opts.xaxis, vars)
            if xv is None:
                continue
        y[i].append(v)            
        x[i].append(xv)

def process_file(filename):
    '''process one file'''
    print("Processing %s" % filename)
    mlog = mavutil.mavlink_connection(filename, notimestamps=opts.notimestamps, zero_time_base=opts.zero_time_base)
    vars = {}
    
    while True:
        msg = mlog.recv_match(opts.condition)
        if msg is None: break
        tdays = matplotlib.dates.date2num(datetime.datetime.fromtimestamp(msg._timestamp))
        add_data(tdays, msg, mlog.messages, mlog.flightmode)

if len(filenames) == 0:
    print("No files to process")
    sys.exit(1)

if opts.labels is not None:
    labels = opts.labels.split(',')
    if len(labels) != len(fields)*len(filenames):
        print("Number of labels (%u) must match number of fields (%u)" % (
            len(labels), len(fields)*len(filenames)))
        sys.exit(1)
else:
    labels = None

for fi in range(0, len(filenames)):
    f = filenames[fi]
    process_file(f)
    for i in range(0, len(x)):
        if first_only[i] and fi != 0:
            x[i] = []
            y[i] = []
    if labels:
        lab = labels[fi*len(fields):(fi+1)*len(fields)]
    else:
        lab = fields[:]
    plotit(x, y, lab, colors=colors[fi*len(fields):])
    for i in range(0, len(x)):
        x[i] = []
        y[i] = []
pylab.show()
raw_input('press enter to exit....')
