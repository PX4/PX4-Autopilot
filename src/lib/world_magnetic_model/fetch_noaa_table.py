#!/usr/bin/env python3
############################################################################
#
#   Copyright (c) 2020-2024 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

import math
import numpy
import json
import statistics
import sys
import urllib.request

SAMPLING_RES = 10
SAMPLING_MIN_LAT = -90
SAMPLING_MAX_LAT = 90
SAMPLING_MIN_LON = -180
SAMPLING_MAX_LON = 180

def constrain(n, nmin, nmax):
    return max(min(nmin, n), nmax)

header = """/****************************************************************************
 *
 *   Copyright (c) 2020-2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
"""

key=sys.argv[1] # NOAA key (https://www.ngdc.noaa.gov/geomag/CalcSurvey.shtml)

print(header)

print('#include <stdint.h>\n')

LAT_DIM=int((SAMPLING_MAX_LAT-SAMPLING_MIN_LAT)/SAMPLING_RES)+1
LON_DIM=int((SAMPLING_MAX_LON-SAMPLING_MIN_LON)/SAMPLING_RES)+1

print('static constexpr float SAMPLING_RES = {}'.format(SAMPLING_RES) + ';')
print('static constexpr float SAMPLING_MIN_LAT = {}'.format(SAMPLING_MIN_LAT) + ';')
print('static constexpr float SAMPLING_MAX_LAT = {}'.format(SAMPLING_MAX_LAT) + ';')
print('static constexpr float SAMPLING_MIN_LON = {}'.format(SAMPLING_MIN_LON) + ';')
print('static constexpr float SAMPLING_MAX_LON = {}'.format(SAMPLING_MAX_LON) + ';')
print('')
print('static constexpr int LAT_DIM = {}'.format(LAT_DIM) + ';')
print('static constexpr int LON_DIM = {}'.format(LON_DIM) + ';')
print('\n')

print('// *INDENT-OFF*\n\n\n')



# build the world magnetic model dictionary
world_magnitude_model = {} # lat/lon dictionary with grid result

params = urllib.parse.urlencode({'lat1': 0, 'lon1': 0, 'resultFormat': 'json'})
f = urllib.request.urlopen("https://www.ngdc.noaa.gov/geomag-web/calculators/calculateIgrfwmm?key=%s&%s" % (key, params))
data = json.loads(f.read())

world_magnitude_model_units = data['units']

for latitude in range(SAMPLING_MIN_LAT, SAMPLING_MAX_LAT+1, SAMPLING_RES):
    world_magnitude_model[latitude] = {}

    for longitude in range(SAMPLING_MIN_LON, SAMPLING_MAX_LON+1, SAMPLING_RES):
        params = urllib.parse.urlencode({'lat1': latitude, 'lon1': longitude, 'lon2': SAMPLING_MAX_LON, 'resultFormat': 'json'})
        f = urllib.request.urlopen("https://www.ngdc.noaa.gov/geomag-web/calculators/calculateIgrfwmm?key=%s&%s" % (key, params))
        data = json.loads(f.read())
        #print(json.dumps(data, indent = 4)) # debugging

        world_magnitude_model[latitude][longitude] = data['result'][0]
        #print(world_magnitude_model[latitude][longitude])


def print_wmm_table(key_name):

    value_min = float('inf')
    value_min_lat_lon = ()

    value_max = float('-inf')
    value_max_lat_lon = ()

    for latitude, lat_row in world_magnitude_model.items():
        #print(latitude, lat_row)
        for longitude, result in lat_row.items():
            #print(result)

            value = float(result[key_name])

            if (value > value_max):
                value_max = value
                value_max_lat_lon = (latitude, longitude)

            if (value < value_min):
                value_min = value
                value_min_lat_lon = (latitude, longitude)

    # scale the values to fit into int16_t
    value_scale_max = abs(numpy.iinfo(numpy.int16).max) / abs(value_max)
    value_scale_min = abs(numpy.iinfo(numpy.int16).min) / abs(value_min)
    value_scale = min(value_scale_max, value_scale_min)

    units_str = world_magnitude_model_units[key_name].split(' ')[0]

    # print the table
    print('// Magnetic {} data in {:.4g} {}'.format(key_name, 1.0 / value_scale, units_str))
    print('// Model: {},'.format(data['model']))
    print('// Version: {},'.format(data['version']))
    print('// Date: {},'.format(data['result'][0]['date']))
    print('static constexpr const int16_t {}_table[{}][{}]'.format(key_name, LAT_DIM, LON_DIM) + " {")
    print('\t//    LONGITUDE: ', end='')
    for l in range(SAMPLING_MIN_LON, SAMPLING_MAX_LON+1, SAMPLING_RES):
        print('{0:6d},'.format(l), end='')
    print('')

    for latitude, lat_row in world_magnitude_model.items():
        print('\t/* LAT: {0:3d} */'.format(latitude) + ' { ', end='')
        latitude_blackout_zone = False

        for longitude, result in lat_row.items():

            value = float(result[key_name])

            # value scaled to fit into int16_t
            value_int = int(round(value * value_scale))
            print('{0:6d},'.format(value_int), end='')

            # blackout warning at this latitude
            try:
                if result['warning']:
                    latitude_blackout_zone = True

            except:
                pass

        if latitude_blackout_zone:
            print(' }, // WARNING! black out zone')
        else:
            print(' },')

    print("};")

    print('static constexpr float WMM_{}_SCALE_TO_{} = {:.9g}f;'.format(key_name.upper(), units_str.upper(), 1.0 / value_scale))
    print('static constexpr float WMM_{}_MIN_{} = {:.1f}f; // latitude: {:.0f}, longitude: {:.0f}'.format(key_name.upper(), units_str.upper(), value_min, value_min_lat_lon[0], value_min_lat_lon[1]))
    print('static constexpr float WMM_{}_MAX_{} = {:.1f}f; // latitude: {:.0f}, longitude: {:.0f}'.format(key_name.upper(), units_str.upper(), value_max, value_max_lat_lon[0], value_max_lat_lon[1]))
    print("\n")


print_wmm_table('declination')
print_wmm_table('inclination')
print_wmm_table('totalintensity')
