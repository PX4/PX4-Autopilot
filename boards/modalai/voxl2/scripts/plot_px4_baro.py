import time
import numpy as np
import argparse
import os
import sys
import subprocess
import psutil
import signal

from multiprocessing import Process
from multiprocessing import Event

# install plotly using    pip3 install plotly psutil --upgrade

parser = argparse.ArgumentParser(description='Barometer Test Script')
parser.add_argument('-n','--num-samples',   type=int, required=False, default=1000)
parser.add_argument('-p','--path',   type=str, required=False, default='.')
parser.add_argument('-c','--num-cpus',   type=int, required=False, default=4)
parser.add_argument('-i','--num-iter',   type=int, required=False, default=3)
parser.add_argument('-s','--sleep-interval',   type=int, required=False, default=300)
parser.add_argument('-t','--enable-plots',   type=int, required=False, default=1)
args = parser.parse_args()

max_sample_count = args.num_samples
keep_going = 1
log_path = args.path

num_iterations = args.num_iter
num_cpu_stress = args.num_cpus
sleep_between_iterations = args.sleep_interval #5*60 #seconds
enable_plots   = args.enable_plots == 1

def handler(signum, frame):
    global keep_going
    keep_going = 0
    print('\nCtrl+C was pressed. Exiting gracefully...')

def kill(proc_pid):
    process = psutil.Process(proc_pid)
    for proc in process.children(recursive=True):
        proc.kill()
    process.kill()

def get_px4_baro():
    cmd = 'px4-listener sensor_baro'
    #cmd = 'px4-listener vehicle_air_data'

    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    timestamp   = None
    pressure    = None
    temperature = None
    device_id   = None

    for line in p.stdout.readlines():
        l=str(line)
        #print(l)
        if 'timestamp' in l:
            ll = l.split(':')[1].split()[0]
            timestamp = float(ll)
        if 'pressure' in l:
            ll = l.split(':')[-1][:-3]
            pressure = float(ll)
        if 'temp' in l:
            ll = l.split(':')[-1][:-3]
            temperature = float(ll)
        if 'device_id' in l:
            ll = l.split(':')[1].split()[0]
            device_id = int(ll)

        retval = p.wait()

    return (timestamp,device_id,pressure,temperature)


signal.signal(signal.SIGINT, handler)

stress_cmd = 'stress -c %d' % num_cpu_stress
def stress_task(event):
    p = subprocess.Popen(stress_cmd, shell=True)

    while True:
        if event.is_set():
            #os.killpg(os.getpgid(p.pid), signal.SIGTERM)
            #os.kill(p.pid, signal.SIGTERM)
            #p.kill()
            #p.communicate()
            kill(p.pid)
            break

        #print('stress running')
        time.sleep(0.1)


if log_path != '.':
        try:
            os.mkdir(log_path)
        except Exception as e:
            print(f'Warning: Could not create log directory: {e}')
            #sys.exit(1)


for i in range(num_iterations):
    if keep_going==0:
        print('Exiting')
        sys.exit(0)
    print(f'Starting iteration #{i}')

    #print('stopping PX4..')
    #subprocess.run(["systemctl", "stop", "voxl-px4"])
    #time.sleep(3.0)

    #print('starting PX4..')
    #subprocess.run(["systemctl", "start", "voxl-px4"])
    #time.sleep(3.0)

    tlast = 0
    ts = []
    ps = []
    temps = []
    sample_count = 0
    time_start = time.time()

    fname = log_path + '/baro_log_%d_%d.txt' % (i,time_start)
    f     = open(fname, 'w')


    ev = Event()
    stress_process= Process(target=stress_task, args=(ev,))
    stress_process.start()

    while (sample_count < max_sample_count and keep_going==1):
        (timestamp, device_id, pressure, temperature) = get_px4_baro()

        #make sure we got new data
        if timestamp != tlast and timestamp is not None:
            if device_id == 12018473:  #icp10100
                sample_count += 1
                ts.append(timestamp)
                ps.append(pressure)
                temps.append(temperature)
            print(f'({sample_count})[{timestamp}] ID: {device_id} P: {pressure}, T: {temperature}')
            f.write(f'{sample_count}, {int(timestamp)}, {device_id}, {pressure}, {temperature}\n')

        tlast=timestamp
        time.sleep(0.01)

    print('Finished collecting data')

    print(f'Closing log file {fname}..')
    f.close()

    print('Stopping stress task')
    ev.set()
    stress_process.join()

    if enable_plots:
        print('Generating plots..')
            
        try:
            import plotly.graph_objects as go
            from plotly.subplots import make_subplots
        except:
            print('WARNING: In order to plot the results, install the Python "plotly" module: pip3 install plotly --upgrade')
            sys.exit(0)

        fig = make_subplots(rows=4, cols=1, start_cell="top-left")

        ts_plot  = np.array(ts)
        ts_plot -= ts_plot[0]
        ts_plot *= 0.000001 #convert from us to s

        # calculate approximate height from start
        dh       = np.array(ps)
        dh      -= dh[0] # subtract the first value
        dh      /= 12.0  # about 12 Pascals per meter at standard pressure and temperature
        dh      *= -1.0  # positive pressure increase means lower height

        t_plot = np.array(temps)
        p_plot = np.array(ps)

        fig.add_trace(go.Scatter(x=ts_plot, y=np.array(ps), name='Pressure (Pa)'), row=1, col=1)
        fig.add_trace(go.Scatter(x=ts_plot, y=t_plot, name='Temperature (deg C)'), row=2, col=1)
        fig.add_trace(go.Scatter(x=ts_plot, y=dh, name='Approx Height Change (m)'), row=3, col=1)
        fig.add_trace(go.Scatter(x=t_plot, y=p_plot, name='Pressure (Pa) vs Temperature'), row=4, col=1)

        fig.update_layout(title_text='Barometer Test Results')
        fig.update_xaxes(title_text='Time (s)',row=1, col=1)
        fig.update_yaxes(title_text='Pressure (Pa)',row=1, col=1)
        fig.update_xaxes(title_text='Time (s)',row=2, col=1)
        fig.update_yaxes(title_text='Temperature (deg C)',row=2, col=1)
        fig.update_xaxes(title_text='Time (s)',row=3, col=1)
        fig.update_yaxes(title_text='Approx Height Change (m)',row=3, col=1)
        fig.update_xaxes(title_text='Temperature (deg C)',row=4, col=1)
        fig.update_yaxes(title_text='Pressure (Pa)',row=4, col=1)

        #fig.update_xaxes(matches='x')

        fig.write_html(log_path + '/barometer_test_results_%d_%d.html' % (i,time_start),include_plotlyjs='cdn')
        #fig.show()  #the figure will not show on VOXL because there is no display / browser

    print('done')

    if (i < (num_iterations-1) ):
        print('sleeping..')
        nsleep = sleep_between_iterations
        for s in range(nsleep):
            time.sleep(1.0)
            if keep_going==0:
                print('Exiting')
                sys.exit(0)

        print('done sleeping')
