# Plots a simple line plot given two array inputs and x,y labels and title

import matplotlib 
import numpy as np
import matplotlib.cm as cm
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
import pickle

def forceAspect(ax,aspect=1):
    im = ax.get_images()
    extent =  im[0].get_extent()
    ax.set_aspect(abs((extent[1]-extent[0])/(extent[3]-extent[2]))/aspect)

def serializePlotStructs(plotStructs, fileName):
    print fileName
    output = open(fileName, 'wb')
    pickle.dump(plotStructs, output)
    output.close()

def generatePlotStructs(results, attacks, saveDir):
    
    resultKeys = ['flightFail','missionFail']
    plotData = dict.fromkeys(resultKeys)

    attackSize = len(attacks)
    xdata = attacks[0].attack_values
    xlabel = attacks[0].label

    ydata = dict.fromkeys(resultKeys)
    zdata = dict.fromkeys(resultKeys)
    titles = dict.fromkeys(resultKeys)

    titles['flightFail'] = 'Flight Envelope Failure Time'
    titles['missionFail'] = 'Mission Envelope Failure Time'

    # Generate size-dependent values
    if attackSize == 1:
        ylabel= 'Failure Time (s)'
        for key in resultKeys:
            ydata[key] = results[key]
            zdata[key] = 0
    elif attackSize == 2:
        ylabel = attacks[1].label
        for key in resultKeys:
            ydata[key] = attacks[1].attack_values
            zdata[key] = results[key]

    # Create the plotting dictionary
    for key in resultKeys:
        plotData[key] = dict(
                size = attackSize,
                fileName = saveDir + key,
                xdata = xdata,
                xlabel = xlabel,
                ydata = ydata[key],
                ylabel = ylabel,
                zdata = zdata[key],
                title = titles[key]
                )

    return plotData

def generatePlots(plotStructs):

    # Define a dictionary of function pointers to select which plotting function to call
    # for a given numer of executed attacks
    functionDict = {
            1 : oneAttackPlot,
            2 : twoAttackPlot
            }

    # Call the plotting function corresponding to the number of attacks run
    for key in plotStructs:
        functionDict[plotStructs[key]['size']](plotStructs[key])

    plt.close('all')

def savePlots(fig, fileName, transparent=True):
    filetypes = ['.png','.eps']
    for ext in filetypes:
        fig.savefig(fileName + ext,bbox_inches='tight', transparent=transparent)

def oneAttackPlot(plotStruct):
    fig = plt.figure()
    plt.plot(plotStruct['xdata'],plotStruct['ydata'])
    plt.ylabel(plotStruct['ylabel'])
    plt.xlabel(plotStruct['xlabel'])
    plt.title(plotStruct['title']+'\n')
    savePlots(fig, plotStruct['fileName'])

    plt.clf()

def twoAttackPlot(plotStruct):
    x = plotStruct['xdata']
    y = plotStruct['ydata']
    X,Y = np.meshgrid(x,y)
    Z = plotStruct['zdata']

    minx = np.min(x)
    maxx = np.max(x)
    miny = np.min(y)
    maxy = np.max(y)

    #levels = np.arrange(np.min(Z), np.max(Z), 10)

    #fig = plt.figure()
    fig = plt.figure(figsize=(8,8))
    ax = fig.add_subplot(111)
    matplotlib.rcParams['contour.negative_linestyle'] = 'solid'
    im = plt.imshow(Z, interpolation='bilinear', origin='lower',cmap=cm.jet_r, extent=(minx,maxx,miny,maxy))

    # If all of the elements of Z are equal, pyplot will be angry. So we test for that condition
    # and reduce one element very slightly.
    if not np.any(Z != Z[0,0]):
        Z[0,0] -= 0.00001

    CS = plt.contour(X, Y, Z,5,colors='k')
    
    plt.clabel(CS, fontsize=9, inline=1)
    CBI = plt.colorbar(im, orientation='vertical', shrink=0.6)

    plt.ylabel(plotStruct['ylabel'])
    plt.xlabel(plotStruct['xlabel'])
    plt.title(plotStruct['title'])
    forceAspect(ax,1.2)
    savePlots(fig, plotStruct['fileName'], transparent=True)
    plt.clf()

