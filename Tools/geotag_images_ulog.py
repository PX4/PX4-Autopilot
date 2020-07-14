#!/usr/bin/env python
#######################################################################################
#
# DeltaTag: enhanced geo-referencing survey images
# @author: Sander Smeets (sander@droneslab.com)
# Copyright (c) 2018 Vertical Technologies
#
# DeltaTag provides an alternate method to geo-referencing images from survey missions.
# It uses the delta in seconds between the images and the delta between the triggers to 
# match geo information with pictures, allowing for missing images.
#
# It uses the last image and last trigger event as a calibration point as errors are
# more likely to arrise in the first images (test triggers, booting cameras, etc)
# 
# Note: DeltaTag does not make copies, it writes the Exif information directly to the images
#
# Install: pip install pyulog piexif Pillow numpy
# Run: python geotag_images_ulog.py [logfile] [image dir] (optional offset)
# eg: python geotag_images_ulog.py mylog.ulg ./images
#
# Parameters
# logfile: a ulog formatted logfile containing camera_capture events (survey missions)
# image_dir: the directory where the images are located
# offset (optional): skip [offset] triggers to reference the last image
#                    0 means the last trigger event equals last image (default)
#                    1 means the second last trigger event equals last image
#                    ...etc
#
# Parts included from https://gist.github.com/c060604/8a51f8999be12fc2be498e9ca56adc72
# Parts included from https://github.com/PX4/flight_review/


from __future__ import print_function
import os, sys, time, datetime, piexif
from pyulog import *
from pyulog.px4 import *
from PIL import Image
from fractions import Fraction


if(len(sys.argv)) < 3:
    print("Usage: python geotag_images_ulog.py [logfile] [image dir]")
    print("Example: python geotag_images_ulog.py mylog.ulg ./images")
    print(len(sys.argv))
    sys.exit()

logfile = sys.argv[1]
imageDir = sys.argv[2]
triggerOffset = 0

if(len(sys.argv) > 3):
    triggerOffset = int(sys.argv[3])

def to_deg(value, loc):
    """convert decimal coordinates into degrees, munutes and seconds tuple
    Keyword arguments: value is float gps-value, loc is direction list ["S", "N"] or ["W", "E"]
    return: tuple like (25, 13, 48.343 ,'N')
    """
    if value < 0:
        loc_value = loc[0]
    elif value > 0:
        loc_value = loc[1]
    else:
        loc_value = ""
    abs_value = abs(value)
    deg =  int(abs_value)
    t1 = (abs_value-deg)*60
    min = int(t1)
    sec = round((t1 - min)* 60, 5)
    return (deg, min, sec, loc_value)


def change_to_rational(number):
    """convert a number to rantional
    Keyword arguments: number
    return: tuple like (1, 2), (numerator, denominator)
    """
    f = Fraction(str(number))
    return (f.numerator, f.denominator)



class ULogException(Exception):
    """
    Exception to indicate an ULog parsing error. It is most likely a corrupt log
    file, but could also be a bug in the parser.
    """
    pass


def load_ulog_file(file_name):
    """ load an ULog file
    :return: ULog object
    """
    # The reason to put this method into helper is that the main module gets
    # (re)loaded on each page request. Thus the caching would not work there.

    # load only the messages we really need
    msg_filter = ['camera_capture']
    try:
        ulog = ULog(file_name, msg_filter)
    except FileNotFoundError:
        print("Error: file %s not found" % file_name)
        raise

    # catch all other exceptions and turn them into an ULogException
    except Exception as error:
        traceback.print_exception(*sys.exc_info())
        raise ULogException()

    # filter messages with timestamp = 0 (these are invalid).
    # The better way is not to publish such messages in the first place, and fix
    # the code instead (it goes against the monotonicity requirement of ulog).
    # So we display the values such that the problem becomes visible.
#    for d in ulog.data_list:
#        t = d.data['timestamp']
#        non_zero_indices = t != 0
#        if not np.all(non_zero_indices):
#            d.data = np.compress(non_zero_indices, d.data, axis=0)

    return ulog

class ULogException(Exception):
    """
    Exception to indicate an ULog parsing error. It is most likely a corrupt log
    file, but could also be a bug in the parser.
    """
    pass


def load_ulog_file(file_name):
    """ load an ULog file
    :return: ULog object
    """
    # The reason to put this method into helper is that the main module gets
    # (re)loaded on each page request. Thus the caching would not work there.

    # load only the messages we really need
    

    msg_filter = ['camera_capture']
    try:
        ulog = ULog(file_name)
    except FileNotFoundError:
        print("Error: file %s not found" % file_name)
        raise

    # catch all other exceptions and turn them into an ULogException
    except Exception as error:
        traceback.print_exception(*sys.exc_info())
        raise ULogException()

    # filter messages with timestamp = 0 (these are invalid).
    # The better way is not to publish such messages in the first place, and fix
    # the code instead (it goes against the monotonicity requirement of ulog).
    # So we display the values such that the problem becomes visible.
#    for d in ulog.data_list:
#        t = d.data['timestamp']
#        non_zero_indices = t != 0
#        if not np.all(non_zero_indices):
#            d.data = np.compress(non_zero_indices, d.data, axis=0)

    return ulog


ulog = load_ulog_file(logfile)

camera_capture = ulog.get_dataset('camera_capture')
count = len(camera_capture.data['timestamp_utc'])
init = round(camera_capture.data['timestamp_utc'][count-1-triggerOffset] / 1000000)
offsets = {}

for i in range(0, count):
  test = round(camera_capture.data['timestamp_utc'][i] / 1000000)
  offset = init - test
  offsets[offset] = i



files = os.listdir(imageDir)
os.chdir(imageDir)
#files.sort(key=lambda x: os.path.getctime(x))
files.sort()
first = 0

for f in reversed(files):
  img = Image.open(f)
  exif_dict = piexif.load(img.info['exif'])
  timestring = exif_dict['Exif'][piexif.ExifIFD.DateTimeOriginal]
  timestamp = time.mktime(datetime.datetime.strptime(timestring, "%Y:%m:%d %H:%M:%S").timetuple())
  if first == 0:
    first = timestamp
    print("Calibrating on",f,"as last image on",timestring)
    print("")
    print("[filename] [offset] [trigger seq] [lat] [lng] [alt]")

  offset = first - timestamp
  print((f), end=' ')
  print((offset), end=' ')
  if not offset in offsets:
    offset += 1

  if not offset in offsets:
    offset += 1

  if offset in offsets:
    print((offsets[offset]), end=' ')
    print((camera_capture.data['lat'][offsets[offset]]), end=' ')
    print((camera_capture.data['lon'][offsets[offset]]), end=' ')
    print(camera_capture.data['alt'][offsets[offset]])

    lat = camera_capture.data['lat'][offsets[offset]]
    lng = camera_capture.data['lon'][offsets[offset]]
    altitude = camera_capture.data['alt'][offsets[offset]]

    lat_deg = to_deg(lat, ["S", "N"])
    lng_deg = to_deg(lng, ["W", "E"])

    exiv_lat = (change_to_rational(lat_deg[0]), change_to_rational(lat_deg[1]), change_to_rational(lat_deg[2]))
    exiv_lng = (change_to_rational(lng_deg[0]), change_to_rational(lng_deg[1]), change_to_rational(lng_deg[2]))

    gps_ifd = {
        piexif.GPSIFD.GPSVersionID: (2, 0, 0, 0),
        piexif.GPSIFD.GPSAltitudeRef: 0,
        piexif.GPSIFD.GPSAltitude: change_to_rational(round(altitude)),
        piexif.GPSIFD.GPSLatitudeRef: lat_deg[3],
        piexif.GPSIFD.GPSLatitude: exiv_lat,
        piexif.GPSIFD.GPSLongitudeRef: lng_deg[3],
        piexif.GPSIFD.GPSLongitude: exiv_lng,
    }

    exif_dict["GPS"] = gps_ifd
    exif_bytes = piexif.dump(exif_dict)
    piexif.insert(exif_bytes, f)
  else:
    print("Could not georeference")


print("Done")
