# Plots a simple line plot given two array inputs and x,y labels and title

import matplotlib 
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.cm as cm
import matplotlib.mlab as mlab
import matplotlib.pyplot as pyplot
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

    xdata = dict.fromkeys(resultKeys)
    ydata = dict.fromkeys(resultKeys)
    zdata = dict.fromkeys(resultKeys)
    titles = dict.fromkeys(resultKeys)

    titles['flightFail'] = 'Flight Envelope Failure Time (without Fault Detection)'
    titles['missionFail'] = 'Mission Envelope Failure Time (without Fault Detection)'

    # Generate size-dependent values
    attackSize = len(attacks)
    if attackSize == 1:
        xlabel = attacks[0].label
        ylabel= 'Failure Time (s)'
        for key in resultKeys:
            xdata[key] = attacks[0].attack_values
            ydata[key] = results[key]
            zdata[key] = 0
    elif attackSize == 2:
        # The first attack is the outer loop and the second is the inner loop. The data in this loop was saved as a row after each execution of the outer loop, making the first attack y and the second attack x. Make sure this is followed.
        xlabel = attacks[1].label
        ylabel = attacks[0].label
        for key in resultKeys:
            xdata[key] = attacks[1].attack_values
            ydata[key] = attacks[0].attack_values
            zdata[key] = results[key]

    # Create the plotting dictionary
    for key in resultKeys:
        plotData[key] = dict(
                size = attackSize,
                fileName = saveDir + key,
                xdata = xdata[key],
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

    pyplot.close('all')

def savePlots(fig, fileName, transparent=True):
    filetypes = ['.png', '.pdf']
    for ext in filetypes:
        fig.savefig(fileName + ext,bbox_inches='tight', transparent=transparent)

def oneAttackPlot(plotStruct):
    fig = pyplot.figure()
    pyplot.plot(plotStruct['xdata'],plotStruct['ydata'])
    pyplot.ylabel(plotStruct['ylabel'])
    pyplot.xlabel(plotStruct['xlabel'])
    pyplot.title(plotStruct['title']+'\n')
    savePlots(fig, plotStruct['fileName'])

    pyplot.clf()

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

    #fig = pyplot.figure()
    fig = pyplot.figure(figsize=(8,8))
    ax = fig.add_subplot(111)
    matplotlib.rcParams['contour.negative_linestyle'] = 'solid'
    im = pyplot.imshow(Z, interpolation='bilinear', origin='lower',cmap=cm.jet_r, extent=(minx,maxx,miny,maxy))

    # If all of the elements of Z are equal, pyplot will be angry. So we test for that condition
    # and reduce one element very slightly.
    contourCount = 5
    flat = False
    if not np.any(Z != Z[0,0]):
        Z[0,0] -= 0.00001
        contourCount = 1
        flat = True

    CS = pyplot.contour(X, Y, Z, contourCount, colors='k')
    
    CBI = pyplot.colorbar(im, orientation='vertical', shrink=0.6)
    CBI.set_label('Time (s)')
    if flat:
        #pass
        CBI.set_ticks([Z[0,1]])
    else:
        pyplot.clabel(CS, fontsize=9, inline=1)

    pyplot.show()
    pyplot.ylabel(plotStruct['ylabel'])
    pyplot.xlabel(plotStruct['xlabel'])
    pyplot.title(plotStruct['title'])
    forceAspect(ax,1.2)
    savePlots(fig, plotStruct['fileName'], transparent=True)
    pyplot.clf()

def checkIfInteresting(plotStructs):
    for key in plotStructs:
        if plotStructs[key]['size'] <= 1:
            return False
        Z = plotStructs[key]['zdata']
        if np.any(Z != Z[0,0]):
            return True
    return False

