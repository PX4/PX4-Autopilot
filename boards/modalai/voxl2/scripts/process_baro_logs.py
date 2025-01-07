from os import listdir
from os.path import isfile, join
import csv
import numpy as np

import plotly.graph_objects as go
from plotly.subplots import make_subplots

mypath = './baro_test0'
enable_reference = False
poly_fit_order = 1
baro_sensor_id = int(12018473)

nplots = 4
if enable_reference:
    nplots = 6
fig = make_subplots(rows=nplots, cols=1, start_cell="top-left")
#ref_temp = 40.0
log_cntr = 0

prefix = 'baro_log'
files = [f for f in listdir(mypath) if isfile(join(mypath, f)) and prefix in f]
files = sorted(files)

print(files)


def moving_average(x, w):
    out = np.convolve(x, np.ones(w), 'valid') / w
    out = np.append(out,out[-(w-1):])
    return out

for f in files:
    log_cntr += 1
    print(f)
    with open(join(mypath, f), 'r') as fp:
        nlines = len(fp.readlines())
    #print(nlines)

    data = np.zeros((4,nlines),dtype=float)
    data2 = np.zeros((4,nlines),dtype=float)

    ndata = 0
    ndata2 = 0
    with open(join(mypath, f)) as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            #print(', '.join(row))
            vals = [float(v.strip()) for v in row]
            nvals = np.asarray(vals)
            nvals = nvals[[0,1,3,4]]
            #print(nvals)

            if vals[2] == baro_sensor_id:
                data[:,ndata] = nvals
                ndata += 1
            else: # secondary barometer
                data2[:,ndata2] = nvals
                ndata2 += 1
                #enable_reference = True


    data   = data[:,0:ndata]
    

    ts_plot  = data[1,:]
    t_off    = ts_plot[0]

    ts_plot -= t_off
    ts_plot *= 0.000001 #convert from us to s
    t_plot   = data[3,:]
    p_plot   = data[2,:]
    p_comp   = p_plot.copy() #-p_plot[i_ref]
    
    #print(i_ref)

    if enable_reference:
        data2 = data2[:,0:ndata2]

        ts_plot2 = data2[1,:]
        ts_plot2 -= t_off  #same offset as main baro
        ts_plot2 *= 0.000001 #convert from us to s
        t_plot2   = data2[3,:]
        p_plot2  = data2[2,:]
        #p_plot2  = moving_average(p_plot2,5)
        p_comp2 = p_plot2 - np.mean(p_plot2)

        for pi in range(p_comp.size):
            #print(pi)
            try:
                ii = np.where(ts_plot2 > ts_plot[pi])[0][0]
            except:
                ii = p_comp2.size-1
            p_comp[pi] -= p_plot2[ii]
        

    #t_ref = np.mean(t_plot)
    t_min = np.min(t_plot)
    t_max = np.max(t_plot)
    t_ref = (t_max+t_min)/2
    #i_ref = np.where(t_plot > ref_temp)[0][0]
    i_ref = np.where(t_plot > t_ref)[0][0]
    
    p_comp = p_comp-p_comp[i_ref]

    #p_plot -= p_plot[i_ref]

    fig.add_trace(go.Scatter(x=ts_plot,  y=p_plot,  name=f'[{log_cntr}] Pressure Raw (Pa)'),  row=1, col=1)
    fig.add_trace(go.Scatter(x=t_plot,   y=p_plot,  name=f'[{log_cntr}] Pressure Raw (Pa)'),  row=2, col=1)
    fig.add_trace(go.Scatter(x=t_plot ,  y=p_comp,  name=f'[{log_cntr}] Pressure Comp (Pa)'), row=3, col=1)
    

    xx = t_plot -t_ref  #- 50.0
    yy = p_comp

    if log_cntr == 1:
        p = np.polyfit(xx, yy, poly_fit_order)

    zz = np.polyval(p,xx)
    fig.add_trace(go.Scatter(x=t_plot ,  y=zz,  name=f'[{log_cntr}] Pressure Fit (Pa)'), row=3, col=1)

    print('Poly Fit:')
    print(p)

    fig.add_trace(go.Scatter(x=t_plot ,  y=p_comp-zz,  name=f'[{log_cntr}] Fit Error (Pa)'), row=4, col=1)

    if enable_reference:
        fig.add_trace(go.Scatter(x=ts_plot2 , y=p_plot2, name=f'[{log_cntr}] Pressure Ref (Pa)'),  row=5, col=1)
        fig.add_trace(go.Scatter(x=ts_plot2 , y=t_plot2, name=f'[{log_cntr}] Temperature Ref (Pa)'),  row=6, col=1)

    # print the PX4 params
    print('param set TC_B_ENABLE 1')
    print('param set TC_B0_ID    %d' % baro_sensor_id)
    print('param set TC_B0_TREF  %.2f' % t_ref)
    print('param set TC_B0_TMIN  %.2f' % t_min)
    print('param set TC_B0_TMAX  %.2f' % t_max)
    for i in range(poly_fit_order+1):
        print('param set TC_B0_X%d    %.5f' % (i,p[poly_fit_order-i]))

    for i in range(poly_fit_order+1,6):
        print('param set TC_B0_X%d    %.5f' % (i,0.0))

    #print(data)

fig.update_xaxes(title_text='Time (s)',row=1, col=1)
fig.update_yaxes(title_text='Absolue Pressure (Pa)',row=1, col=1)

fig.update_xaxes(title_text='Temperature (deg C)',row=2, col=1)
fig.update_yaxes(title_text='Absolue Pressure (Pa)',row=2, col=1)

fig.update_xaxes(title_text='Temperature (deg C)',row=3, col=1)
fig.update_yaxes(title_text='Relative Pressure (Pa)',row=3, col=1)

fig.update_xaxes(title_text='Temperature (deg C)',row=4, col=1)
fig.update_yaxes(title_text='Pressure Fit Error (Pa)',row=4, col=1)

if enable_reference:
    fig.update_xaxes(title_text='Time (s)',row=5, col=1)
    fig.update_yaxes(title_text='Reference Pressure (Pa)',row=5, col=1)
    fig.update_xaxes(title_text='Time (s)',row=6, col=1)
    fig.update_yaxes(title_text='Reference Temperature (Pa)',row=6, col=1)



fig.show()