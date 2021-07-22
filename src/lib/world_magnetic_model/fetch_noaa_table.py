#!/usr/bin/env python3
############################################################################
#
#   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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
import json
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
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

print('// *INDENT-OFF*')

# Declination
params = urllib.parse.urlencode({'lat1': 0, 'lat2': 0, 'lon1': 0, 'lon2': 0, 'latStepSize': 1, 'lonStepSize': 1, 'magneticComponent': 'd', 'resultFormat': 'json'})
f = urllib.request.urlopen("https://www.ngdc.noaa.gov/geomag-web/calculators/calculateIgrfgrid?%s" % params)
data = json.loads(f.read())
print("// Magnetic declination data in radians * 10^-4")
print('// Model: {},'.format(data['model']))
print('// Version: {},'.format(data['version']))
print('// Date: {},'.format(data['result'][0]['date']))
print('static constexpr const int16_t declination_table[{}][{}]'.format(LAT_DIM, LON_DIM) + " {")
print('\t//    LONGITUDE: ', end='')
for l in range(SAMPLING_MIN_LON, SAMPLING_MAX_LON+1, SAMPLING_RES):
    print('{0:6d},'.format(l), end='')
print('')
for latitude in range(SAMPLING_MIN_LAT, SAMPLING_MAX_LAT+1, SAMPLING_RES):
    params = urllib.parse.urlencode({'lat1': latitude, 'lat2': latitude, 'lon1': SAMPLING_MIN_LON, 'lon2': SAMPLING_MAX_LON, 'latStepSize': 1, 'lonStepSize': SAMPLING_RES, 'magneticComponent': 'd', 'resultFormat': 'json'})
    f = urllib.request.urlopen("https://www.ngdc.noaa.gov/geomag-web/calculators/calculateIgrfgrid?%s" % params)
    data = json.loads(f.read())

    print('\t/* LAT: {0:3d} */'.format(latitude) + ' { ', end='')
    for p in data['result']:
        # declination in radians * 10^-4
        declination_int = constrain(int(round(math.radians(p['declination'] * 10000))), 32767, -32768)
        print('{0:6d},'.format(declination_int), end='')

    print(' },')
print("};\n")

# Inclination
params = urllib.parse.urlencode({'lat1': 0, 'lat2': 0, 'lon1': 0, 'lon2': 0, 'latStepSize': 1, 'lonStepSize': 1, 'magneticComponent': 'i', 'resultFormat': 'json'})
f = urllib.request.urlopen("https://www.ngdc.noaa.gov/geomag-web/calculators/calculateIgrfgrid?%s" % params)
data = json.loads(f.read())
print("// Magnetic inclination data in radians * 10^-4")
print('// Model: {},'.format(data['model']))
print('// Version: {},'.format(data['version']))
print('// Date: {},'.format(data['result'][0]['date']))
print('static constexpr const int16_t inclination_table[{}][{}]'.format(LAT_DIM, LON_DIM) + " {")
print('\t//    LONGITUDE: ', end='')
for l in range(SAMPLING_MIN_LON, SAMPLING_MAX_LON+1, SAMPLING_RES):
    print('{0:6d},'.format(l), end='')
print('')
for latitude in range(SAMPLING_MIN_LAT, SAMPLING_MAX_LAT+1, SAMPLING_RES):
    params = urllib.parse.urlencode({'lat1': latitude, 'lat2': latitude, 'lon1': SAMPLING_MIN_LON, 'lon2': SAMPLING_MAX_LON, 'latStepSize': 1, 'lonStepSize': SAMPLING_RES, 'magneticComponent': 'i', 'resultFormat': 'json'})
    f = urllib.request.urlopen("https://www.ngdc.noaa.gov/geomag-web/calculators/calculateIgrfgrid?%s" % params)
    data = json.loads(f.read())

    print('\t/* LAT: {0:3d} */'.format(latitude) + ' { ', end='')
    for p in data['result']:
        # inclination in radians * 10^-4
        inclination_int = constrain(int(round(math.radians(p['inclination'] * 10000))), 32767, -32768)
        print('{0:6d},'.format(inclination_int), end='')

    print(' },')
print("};\n")

# total intensity
params = urllib.parse.urlencode({'lat1': 0, 'lat2': 0, 'lon1': 0, 'lon2': 0, 'latStepSize': 1, 'lonStepSize': 1, 'magneticComponent': 'i', 'resultFormat': 'json'})
f = urllib.request.urlopen("https://www.ngdc.noaa.gov/geomag-web/calculators/calculateIgrfgrid?%s" % params)
data = json.loads(f.read())
print("// Magnetic strength data in milli-Gauss * 10")
print('// Model: {},'.format(data['model']))
print('// Version: {},'.format(data['version']))
print('// Date: {},'.format(data['result'][0]['date']))
print('static constexpr const int16_t strength_table[{}][{}]'.format(LAT_DIM, LON_DIM) + " {")
print('\t//    LONGITUDE: ', end='')
for l in range(SAMPLING_MIN_LON, SAMPLING_MAX_LON+1, SAMPLING_RES):
    print('{0:5d},'.format(l), end='')
print('')
for latitude in range(SAMPLING_MIN_LAT, SAMPLING_MAX_LAT+1, SAMPLING_RES):
    params = urllib.parse.urlencode({'lat1': latitude, 'lat2': latitude, 'lon1': SAMPLING_MIN_LON, 'lon2': SAMPLING_MAX_LON, 'latStepSize': 1, 'lonStepSize': SAMPLING_RES, 'magneticComponent': 'f', 'resultFormat': 'json'})
    f = urllib.request.urlopen("https://www.ngdc.noaa.gov/geomag-web/calculators/calculateIgrfgrid?%s" % params)
    data = json.loads(f.read())

    print('\t/* LAT: {0:3d} */'.format(latitude) + ' { ', end='')
    for p in data['result']:
        totalintensity_int = int(round(p['totalintensity']/10))
        print('{0:5d},'.format(totalintensity_int), end='')

    print(' },')
print("};")
