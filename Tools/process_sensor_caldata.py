#! /usr/bin/env python
"""
Reads in IMU data from a static thermal calibration test and performs a curve fit of gyro, accel and baro bias vs temperature
Data can be gathered using the following sequence:

1) Power up the board and set the TC_A_ENABLE, TC_B_ENABLE and TC_G_ENABLE parameters to 1
2) Set all CAL_GYR and CAL_ACC parameters to defaults
3) Set the SYS_LOGGER parameter to 1 to use the new system logger
4) Set the SDLOG_MODE parameter to 3 to enable logging of sensor data for calibration and power off
5) Cold soak the board for 30 minutes
6) Move to a warm dry, still air, constant pressure environment.
7) Apply power for 45 minutes, keeping the board still.
8) Remove power and extract the .ulog file
9) Open a terminal window in the Firmware/Tools directory and run the python calibration script script file: 'python process_sensor_caldata.py <full path name to .ulog file>
10) Power the board, connect QGC and load the parameter from the generated .params file onto the board using QGC. Due to the number of parameters, loading them may take some time.
11) TODO - we need a way for user to reliably tell when parameters have all been changed and saved.
12) After parameters have finished loading, set SDLOG_MODE to 1 to re-enable normal logging and remove power.
13) Power the board and perform a normal gyro and accelerometer sensor calibration using QGC. The board must be repowered after this step before flying due to large parameter changes and the thermal compensation parameters only being read on startup.

Outputs thermal compensation parameters in a file named <inputfilename>.params which can be loaded onto the board using QGroundControl
Outputs summary plots in a pdf file named <inputfilename>.pdf

"""


from __future__ import print_function

import argparse
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import os
#import json

import pandas
from numpy.polynomial import Polynomial
import numpy as np
import pyulog
from jinja2 import Environment, FileSystemLoader

# pylint: disable=invalid-name


def ulog2pandas(log):
    """
    Convert from ulog to pandas using dictionary
    """
    r = {}
    for msg in log.data_list:
        data = pandas.DataFrame.from_dict(msg.data)
        data.index = pandas.TimedeltaIndex(data.timestamp, 'us')
        if msg.name not in r.keys():
            r[msg.name] = {}
        r[msg.name][msg.multi_id] = data
    return r


def fitPlot(x, y, f_poly, name, field, config):
    """
    A temperature calibration fit plot.
    """
    # pylint: disable=too-many-arguments
    x = x.resample(config['plot_interval']).mean()
    y = y.resample(config['plot_interval']).mean()
    plt.plot(x, y, '.', label=field)
    x_resample = np.linspace(x.min(), x.max())
    plt.plot(x_resample, f_poly(x_resample), '-',  label='{:s} fit'.format(field))
    plt.title('{:s} temperature calibration'.format(name))
    plt.xlabel(config['xlabel'])
    plt.ylabel(config['ylabel'])
    plt.legend(loc='best', ncol=3)


def qgc_param(value, param_type):
    """
    Construct a dict representation of a qgroundcontrol parameter
    """
    type_code = {
        'int': 6,
        'float': 9,
    }
    return {'value': value, 'type': type_code[param_type]}


def temperature_calibration(ulog_filename, do_plot):
    """
    Do a temperature calibration
    @param ulog_filename : log filename
    @do_plot : create plots, save to pdf
    """
    log = pyulog.ULog(ulog_filename)
    r = ulog2pandas(log)
    coeffs = {}

    pdf_filename = "{:s}_temp_cal.pdf".format(
        ulog_filename.replace('.ulg',''))

    pdf = PdfPages(pdf_filename)

    for topic in r.keys():
        coeffs[topic] = {}
        for multi_id in r[topic].keys():

            name = '{:s}_{:d}'.format(topic, multi_id)
            print('processing', name)

            # default for config
            config = {
                'fields': [],
                'units': [],
                'poly_deg': 3,
                'xlabel': 'Temp, deg C',
                'ylabel': '',
                'min': lambda x: float(x.min()),
                'max': lambda x: float(x.max()),
                'ref': lambda x: float((x.min() + x.max())/2),
                'offset': lambda y: float(0),
                'save_plot': True,
                'plot_interval': '5 s',
            }

            if topic == 'sensor_baro':
                config['fields'] = ['pressure']
                config['ylabel'] = 'pressure, Pa'
                config['offset'] = lambda y: y.median()
                config['poly_deg'] = 5
            elif topic == 'sensor_gyro':
                config['fields'] = ['x', 'y', 'z']
                config['ylabel'] = 'gyro, rad/s'
            elif topic == 'sensor_accel':
                config['fields'] = ['x', 'y', 'z']
                config['ylabel'] = 'accel, m/s^2'
                config['offset'] = lambda y: y.median()
            else:
                continue

            # get data and fill in empty (NaN) values with forward fill, followed by
            # backward fill
            data = r[topic][multi_id].ffill().bfill()
            x = data.temperature

            try:
                device_id = int(np.median(r[topic][multi_id]['device_id']))
            except KeyError:
                print('device id not found')
                device_id = 0

            # default for coefficients
            coeffs[topic][multi_id] = {
                'poly': {},
                'T_min': config['min'](x),
                'T_max': config['max'](x),
                'T_ref': config['ref'](x),
                'device_id': device_id
            }
            name = '{:s}_{:d}'.format(topic, multi_id)

            plt.figure()

            for i, field in enumerate(config['fields']):
                y = data[field]
                y -= config['offset'](y)
                f_poly = Polynomial.fit(x, y, config['poly_deg'])
                coeffs[topic][multi_id]['poly'][field] = list(f_poly.coef)

                if do_plot:
                    fitPlot(
                        x=x, y=y, f_poly=f_poly, name=name, field=field,
                        config=config)
                    if config['save_plot'] and i == len(config['fields']) - 1:
                        pdf.savefig()
                        plt.close()

    pdf.close()

    # print(json.dumps(coeffs, indent=2, sort_keys=True))

    # create param dict from coeff dict
    params = {}
    for topic in coeffs.keys():
        for mult_id in coeffs[topic].keys():
            data = coeffs[topic][multi_id]

            # common params
            p = {
                'ID': qgc_param(data['device_id'], 'int'),
                'TMIN': qgc_param(data['T_min'], 'float'),
                'TMAX': qgc_param(data['T_max'], 'float'),
                'TREF': qgc_param(data['T_ref'], 'float'),
                'SCL_0': qgc_param(1.0, 'float'),
                'SCL_1': qgc_param(1.0, 'float'),
                'SCL_2': qgc_param(1.0, 'float'),
            }

            # poly coeffs
            for i_field, field in enumerate(data['poly'].keys()):
                for i_c, c in enumerate(data['poly'][field]):
                    p['X{:d}_{:d}'.format(i_c, i_field)] = \
                        qgc_param(data['poly'][field][i_c], 'float')

            # naming
            if topic == 'sensor_gyro':
                name = 'TC_G{:d}'.format(mult_id)
                params[name + '_ID'] = qgc_param(mult_id, 'int')
            elif topic == 'sensor_baro':
                name = 'TC_B{:d}'.format(mult_id)
            elif topic == 'sensor_accel':
                name = 'TC_A{:d}'.format(mult_id)

            # prepend name to params and save in params dict
            params.update({ '{:s}_{:s}'.format(name, key): val 
                for key, val in p.iteritems() })

    # print(json.dumps(params, indent=2, sort_keys=True))

    # for jinja docs see: http://jinja.pocoo.org/docs/2.9/api/
    script_path = os.path.dirname(os.path.realpath(__file__))
    env = Environment(
        loader=FileSystemLoader(os.path.join(script_path, 'templates')))

    template_file = 'temp_cal.params.jinja'
    template = env.get_template(template_file)
    with open(ulog_filename.replace('ulg', 'params'), 'w') as fid:
        fid.write(template.render(params=params))

if __name__ == "__main__":
    parser = argparse.ArgumentParser('Temperature calibration')
    parser.add_argument('ulog')
    parser.add_argument('--noplot', action='store_true', default=False)
    args = parser.parse_args()
    temperature_calibration(args.ulog, not args.noplot)

#  vim: set et fenc=utf-8 ff=unix sts=0 sw=4 ts=4 :
