#! /usr/bin/env python
"""
Convert a ULog file into a KML file (positioning information)
"""

from __future__ import print_function

import argparse
import simplekml # pylint: disable=import-error

from .core import ULog


#pylint: disable=too-many-locals, invalid-name, consider-using-enumerate, too-many-arguments
#pylint: disable=unused-variable


def main():
    """Command line interface"""

    parser = argparse.ArgumentParser(description='Convert ULog to KML')
    parser.add_argument('filename', metavar='file.ulg', help='ULog input file')

    parser.add_argument('-o', '--output', dest='output_filename',
                        help="output filename", default='track.kml')
    parser.add_argument('--topic', dest='topic_name',
                        help="topic name with position data (default=vehicle_gps_position)",
                        default='vehicle_gps_position')
    parser.add_argument('--camera-trigger', dest='camera_trigger',
                        help="Camera trigger topic name (e.g. camera_capture)",
                        default=None)
    parser.add_argument('-i', '--ignore', dest='ignore', action='store_true',
                        help='Ignore string parsing exceptions', default=False)

    args = parser.parse_args()

    convert_ulog2kml(args.filename, args.output_filename,
                     position_topic_name=args.topic_name,
                     camera_trigger_topic_name=args.camera_trigger,
                     disable_str_exceptions=args.ignore)

    # alternative example call:
#    convert_ulog2kml(args.filename, 'test.kml', ['vehicle_global_position',
#        'vehicle_gps_position'], [_kml_default_colors, lambda x: simplekml.Color.green])


def _kml_default_colors(x):
    """ flight mode to color conversion """
    x = max([x, 0])
    colors_arr = [simplekml.Color.red, simplekml.Color.green, simplekml.Color.blue,
                  simplekml.Color.violet, simplekml.Color.yellow, simplekml.Color.orange,
                  simplekml.Color.burlywood, simplekml.Color.azure, simplekml.Color.lightblue,
                  simplekml.Color.lawngreen, simplekml.Color.indianred, simplekml.Color.hotpink,
                  simplekml.Color.bisque, simplekml.Color.cyan, simplekml.Color.darksalmon,
                  simplekml.Color.deepskyblue, simplekml.Color.lime, simplekml.Color.orchid]
    return colors_arr[x]



def convert_ulog2kml(ulog_file_name, output_file_name, position_topic_name=
                     'vehicle_gps_position', colors=_kml_default_colors, altitude_offset=0,
                     minimum_interval_s=0.1, style=None, camera_trigger_topic_name=None,
                     disable_str_exceptions=False):
    """
    Coverts and ULog file to a CSV file.

    :param ulog_file_name: The ULog filename to open and read
    :param output_file_name: KML Output file name
    :param position_topic_name: either name of a topic (must have 'lon', 'lat' &
           'alt' fields), or a list of topic names
    :param colors: lambda function with flight mode (int) (or -1) as input and
           returns a color (eg 'fffff8f0') (or list of lambda functions if
           multiple position_topic_name's)
    :param altitude_offset: add this offset to the altitude [m]
    :param minimum_interval_s: minimum time difference between two datapoints
           (drop if more points)
    :param style: dictionary with rendering options:
                  'extrude': Bool
                  'line_width': int
    :param camera_trigger_topic_name: name of the camera trigger topic (must
                                      have 'lon', 'lat' & 'seq')

    :return: None
    """

    default_style = {
        'extrude': False,
        'line_width': 3
        }

    used_style = default_style
    if style is not None:
        for key in style:
            used_style[key] = style[key]


    if not isinstance(position_topic_name, list):
        position_topic_name = [position_topic_name]
        colors = [colors]

    kml = simplekml.Kml()
    load_topic_names = position_topic_name + ['vehicle_status']
    if camera_trigger_topic_name is not None:
        load_topic_names.append(camera_trigger_topic_name)
    ulog = ULog(ulog_file_name, load_topic_names, disable_str_exceptions)

    # get flight modes
    try:
        cur_dataset = ulog.get_dataset('vehicle_status')
        flight_mode_changes = cur_dataset.list_value_changes('nav_state')
        flight_mode_changes.append((ulog.last_timestamp, -1))
    except (KeyError, IndexError) as error:
        flight_mode_changes = []

    # add the graphs
    for topic, cur_colors in zip(position_topic_name, colors):
        _kml_add_position_data(kml, ulog, topic, cur_colors, used_style,
                               altitude_offset, minimum_interval_s, flight_mode_changes)

    # camera triggers
    _kml_add_camera_triggers(kml, ulog, camera_trigger_topic_name, altitude_offset)

    kml.save(output_file_name)


def _kml_add_camera_triggers(kml, ulog, camera_trigger_topic_name, altitude_offset):
    """
    Add camera trigger points to the map
    """

    data = ulog.data_list
    topic_instance = 0

    cur_dataset = [elem for elem in data
                   if elem.name == camera_trigger_topic_name and elem.multi_id == topic_instance]
    if len(cur_dataset) > 0:
        cur_dataset = cur_dataset[0]

        pos_lon = cur_dataset.data['lon']
        pos_lat = cur_dataset.data['lat']
        pos_alt = cur_dataset.data['alt']
        sequence = cur_dataset.data['seq']

        for i in range(len(pos_lon)):
            pnt = kml.newpoint(name='Camera Trigger '+str(sequence[i]))
            pnt.coords = [(pos_lon[i], pos_lat[i], pos_alt[i] + altitude_offset)]
            # Balloons instead of text does not work
            #pnt.style.balloonstyle.text = 'Camera Trigger '+str(sequence[i])


def _kml_add_position_data(kml, ulog, position_topic_name, colors, style,
                           altitude_offset=0, minimum_interval_s=0.1,
                           flight_mode_changes=None):

    data = ulog.data_list
    topic_instance = 0
    if flight_mode_changes is None:
        flight_mode_changes = []

    cur_dataset = [elem for elem in data
                   if elem.name == position_topic_name and elem.multi_id == topic_instance]
    if len(cur_dataset) == 0:
        raise Exception(position_topic_name+' not found in data')

    cur_dataset = cur_dataset[0]


    pos_lon = cur_dataset.data['lon']
    pos_lat = cur_dataset.data['lat']
    pos_alt = cur_dataset.data['alt']
    pos_t = cur_dataset.data['timestamp']

    if 'fix_type' in cur_dataset.data:
        indices = cur_dataset.data['fix_type'] > 2 # use only data with a fix
        pos_lon = pos_lon[indices]
        pos_lat = pos_lat[indices]
        pos_alt = pos_alt[indices]
        pos_t = pos_t[indices]

    # scale if it's an integer type
    lon_type = [f.type_str for f in cur_dataset.field_data if f.field_name == 'lon']
    if len(lon_type) > 0 and lon_type[0] == 'int32_t':
        pos_lon = pos_lon / 1e7 # to degrees
        pos_lat = pos_lat / 1e7
        pos_alt = pos_alt / 1e3 # to meters


    current_flight_mode = 0
    current_flight_mode_idx = 0
    if len(flight_mode_changes) > 0:
        current_flight_mode = flight_mode_changes[0][1]


    def create_linestring():
        """ create a new kml linestring and set rendering options """
        name = position_topic_name + ":" + str(current_flight_mode)
        new_linestring = kml.newlinestring(name=name, altitudemode='absolute')

        # set rendering options
        if style['extrude']:
            new_linestring.extrude = 1
        new_linestring.style.linestyle.color = colors(current_flight_mode)

        new_linestring.style.linestyle.width = style['line_width']
        return new_linestring

    current_kml_linestring = create_linestring()

    last_t = 0
    for i in range(len(pos_lon)):
        cur_t = pos_t[i]

        if (cur_t - last_t)/1e6 > minimum_interval_s: # assume timestamp is in [us]
            pos_data = [pos_lon[i], pos_lat[i], pos_alt[i] + altitude_offset]
            current_kml_linestring.coords.addcoordinates([pos_data])
            last_t = cur_t

            # flight mode change?
            while current_flight_mode_idx < len(flight_mode_changes)-1 and \
                flight_mode_changes[current_flight_mode_idx+1][0] <= cur_t:
                current_flight_mode_idx += 1
                current_flight_mode = flight_mode_changes[current_flight_mode_idx][1]
                current_kml_linestring = create_linestring()

                current_kml_linestring.coords.addcoordinates([pos_data])


