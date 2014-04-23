from distutils.core import setup, Extension
import glob, os, shutil

version = '1.1.29'

from generator import mavgen, mavparse

# path to message_definitions directory
mdef_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'message_definitions')
dialects_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'dialects')

v09_dialects = glob.glob(os.path.join(mdef_path, 'v0.9', '*.xml'))
v10_dialects = glob.glob(os.path.join(mdef_path, 'v1.0', '*.xml'))

v09_dialects

if not "NOGEN" in os.environ:
    for xml in v09_dialects:
        shutil.copy(xml, os.path.join(dialects_path, 'v09'))
    for xml in v10_dialects:
        shutil.copy(xml, os.path.join(dialects_path, 'v10'))

    for xml in v09_dialects:
        dialect = os.path.basename(xml)[:-4]
        print("Building %s" % xml)
        mavgen.mavgen_python_dialect(dialect, mavparse.PROTOCOL_0_9)

    for xml in v10_dialects:
        dialect = os.path.basename(xml)[:-4]
        print("Building %s" % xml)
        mavgen.mavgen_python_dialect(dialect, mavparse.PROTOCOL_1_0)

setup (name = 'pymavlink',
       version = version,
       description = 'Python MAVLink code',
       long_description = '''A Python library for handling MAVLink protocol streams and log files. This allows for the creation of simple scripts to analyse telemetry logs from autopilots such as ArduPilot which use the MAVLink protocol. See the scripts that come with the package for examples of small, useful scripts that use pymavlink. For more information about the MAVLink protocol see http://qgroundcontrol.org/mavlink/''',
       url = 'http://github.com/mavlink/mavlink',
       classifiers=['Development Status :: 4 - Beta',
                    'Environment :: Console',
                    'Intended Audience :: Science/Research',
                    'License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)',
                    'Operating System :: OS Independent',
                    'Programming Language :: Python :: 2.7',
                    'Topic :: Scientific/Engineering'
                    ],
       license='LGPLv3',
       package_dir = { 'pymavlink' : '.' },
       package_data = { 'pymavlink.dialects.v09' : ['*.xml'],
                        'pymavlink.dialects.v10' : ['*.xml'],
                        'pymavlink.generator'    : [ '*.xsd' ],
                        'pymavlink.generator'    : [ 'C/include_v0.9/*.h',
                                                     'C/include_v1.0/*.h',
                                                     'C/include_v1.0/*.hpp' ],
                        'pymavlink.generator.lib.minixsv': [ '*.xsd' ] },
       packages = ['pymavlink',
                   'pymavlink.generator',
                   'pymavlink.generator.lib',
                   'pymavlink.generator.lib.genxmlif',
                   'pymavlink.generator.lib.minixsv',
                   'pymavlink.dialects',
                   'pymavlink.dialects.v09',
                   'pymavlink.dialects.v10'],
       scripts = [ 'tools/magfit_delta.py', 'tools/mavextract.py',
                   'tools/mavgraph.py', 'tools/mavparmdiff.py',
                   'tools/mavtogpx.py', 'tools/magfit_gps.py',
                   'tools/mavflightmodes.py', 'tools/mavlogdump.py',
                   'tools/mavparms.py', 'tools/magfit_motors.py',
                   'tools/mavflighttime.py', 'tools/mavloss.py',
                   'tools/mavplayback.py', 'tools/magfit.py',
                   'tools/mavgpslock.py',
                   'tools/mavmission.py',
                   'tools/mavsigloss.py',
                   'tools/mavsearch.py',
                   'tools/mavtomfile.py',
                   'generator/mavgen.py', 'tools/mavkml.py']
       )
