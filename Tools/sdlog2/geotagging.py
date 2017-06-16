#!/usr/bin/env python
#
#       __geotagging__
#       Tag the images recorded during a flight with geo location extracted from
#       a PX4 log file.
#
#       This file accepts *.jpg format images and reads position information
#       from a *.px4log file
#
#       Example Syntax:
#       python geotagging.py --logfile=log001.px4log --input=images/
#                            --output=imagesWithTag/
#
#   Author: Andreas Bircher, Wingtra, http://wingtra.com, in 2016
#

import glob
import os
import pyexiv2
from shutil import copyfile
from optparse import OptionParser
import csv
from datetime import datetime, timedelta

class TriggerList(object):

    def __init__(self):
        self.CAMT_seq = []
        self.CAMT_timestamp = []
        self.GPOS_Lat = []
        self.GPOS_Lon = []
        self.GPOS_Alt = []
        self.GPS_GPSTime = []


class ImageList(object):

    def __init__(self):
        self.jpg = []
        self.raw = []


def to_degree(value, loc):
    if value < 0:
        loc_value = loc[0]
    elif value > 0:
        loc_value = loc[1]
    else:
        loc_value = ""
    absolute_value = abs(value)
    deg = int(absolute_value)
    t1 = (absolute_value - deg) * 60
    min = int(t1)
    sec = round((t1 - min) * 60, 5)
    return (deg, min, sec, loc_value)


def SetGpsLocation(file_name, gps_datetime, lat, lng, alt):
    """
        Adding GPS tag

    """
    lat_deg = to_degree(lat, ["S", "N"])
    lng_deg = to_degree(lng, ["W", "E"])

    exiv_lat = (pyexiv2.Rational(lat_deg[0] * 60 + lat_deg[1], 60), pyexiv2.Rational(
        lat_deg[2] * 100, 6000), pyexiv2.Rational(0, 1))
    exiv_lng = (pyexiv2.Rational(lng_deg[0] * 60 + lng_deg[1], 60), pyexiv2.Rational(
        lng_deg[2] * 100, 6000), pyexiv2.Rational(0, 1))

    exiv_image = pyexiv2.ImageMetadata(file_name)
    exiv_image.read()

    date_tag = exiv_image['Exif.Image.DateTime']

    date_max = max(date_tag.value, gps_datetime)
    date_min = min(date_tag.value, gps_datetime)
    time_diff = date_max - date_min
    if (time_diff > timedelta(seconds=5)):
        print(
            "WARNING, camera trigger and photo time different by {}".format(time_diff))
        print("  Photo tag time: {}".format(date_tag.value))
        print("  Camera trigger time: {}".format(gps_datetime))

    exiv_image["Exif.GPSInfo.GPSLatitude"] = exiv_lat
    exiv_image["Exif.GPSInfo.GPSLatitudeRef"] = lat_deg[3]
    exiv_image["Exif.GPSInfo.GPSLongitude"] = exiv_lng
    exiv_image["Exif.GPSInfo.GPSLongitudeRef"] = lng_deg[3]
    exiv_image["Exif.GPSInfo.GPSAltitude"] = pyexiv2.Rational(alt, 1)
    exiv_image["Exif.GPSInfo.GPSAltitudeRef"] = '0'
    exiv_image["Exif.Image.GPSTag"] = 654
    exiv_image["Exif.GPSInfo.GPSMapDatum"] = "WGS-84"
    exiv_image["Exif.GPSInfo.GPSVersionID"] = '2 0 0 0'

    exiv_image.write(True)


def LoadPX4log(px4_log_file):
    """
       load px4 log file and extract trigger locations
    """
    os.system('python sdlog2_dump.py ' + px4_log_file +
              ' -t time -m TIME -m CAMT -m GPOS -m GPS -f log.csv')
    f = open('log.csv', 'rb')
    reader = csv.reader(f)
    headers = reader.next()
    line = {}
    for h in headers:
        line[h] = []

    for row in reader:
        for h, v in zip(headers, row):
            line[h].append(v)

    trigger_list = TriggerList()
    for seq in range(0, len(line['CAMT_seq']) - 1):
        if line['CAMT_seq'][seq] != line['CAMT_seq'][seq + 1]:
            trigger_list.CAMT_seq.append(line['CAMT_seq'][seq + 1])
            trigger_list.CAMT_timestamp.append(line['CAMT_timestamp'][seq + 1])
            trigger_list.GPOS_Lat.append(line['GPOS_Lat'][seq + 1])
            trigger_list.GPOS_Lon.append(line['GPOS_Lon'][seq + 1])
            trigger_list.GPOS_Alt.append(line['GPOS_Alt'][seq + 1])
            trigger_list.GPS_GPSTime.append(line['GPS_GPSTime'][seq + 1])

    return trigger_list


def LoadImageList(input_folder):
    """
       load the image list
    """
    image_list = ImageList()
    for jpg_image in glob.glob(input_folder + "/*.jpg"):
        image_list.jpg.append(jpg_image)
    for jpg_image in glob.glob(input_folder + "/*.JPG"):
        image_list.jpg.append(jpg_image)
    for raw_image in glob.glob(input_folder + "/*.RC"):
        image_list.raw.append(raw_image)
    if len(image_list.jpg) != len(image_list.raw) and len(image_list.jpg) * len(image_list.raw) != 0:
        print("Unequal number of jpg and raw images")
    if len(image_list.jpg) == 0 and len(image_list.raw) == 0:
        print("No images found")
    image_list.jpg = sorted(image_list.jpg)
    image_list.raw = sorted(image_list.raw)
    return image_list


def FilterTrigger(trigger_list, image_list):
    """
       filter triggers to allow exact matching with recorded images
    """
    # filter trigger list to match the number of pics
    if len(image_list.jpg) != len(trigger_list.CAMT_seq):
        print('WARNING! differ number of jpg images ({}) and camera triggers ({})'.format(
            len(image_list.jpg), len(trigger_list.CAMT_seq)))

        n_overlap = min(len(image_list.jpg), len(trigger_list.CAMT_seq))
        del image_list.jpg[n_overlap:]

    if len(image_list.raw) != len(trigger_list.CAMT_seq):
        print('WARNING! differ number of raw images ({}) and camera triggers ({})'.format(
            len(image_list.raw), len(trigger_list.CAMT_seq)))

        n_overlap = min(len(image_list.raw), len(trigger_list.CAMT_seq))
        del image_list.raw[n_overlap:]

    return trigger_list


def TagImages(trigger_list, image_list, output_folder):
    """
       load px4 log file and extract trigger locations
    """
    for image in range(len(image_list.jpg)):

        print("############################################################")
        print('Photo {}: {}'.format(image, image_list.jpg[image]))

        cam_time = int(trigger_list.GPS_GPSTime[image]) / 1000000
        gps_datetime = datetime.fromtimestamp(cam_time)

        base_path, filename = os.path.split(image_list.jpg[image])
        copyfile(image_list.jpg[image], output_folder + "/" + filename)
        SetGpsLocation(output_folder + "/" + filename, gps_datetime, float(
            trigger_list.GPOS_Lat[image]), float(trigger_list.GPOS_Lon[image]), float(trigger_list.GPOS_Alt[image]))

    for image in range(len(image_list.raw)):

        print("############################################################")
        print('Photo {}: {}'.format(image, image_list.raw[image]))

        cam_time = int(trigger_list.GPS_GPSTime[image]) / 1000000
        gps_datetime = datetime.fromtimestamp(cam_time)

        base_path, filename = os.path.split(image_list.raw[image])
        copyfile(image_list.raw[image], output_folder + "/" + filename)
        SetGpsLocation(output_folder + "/" + filename, gps_datetime, float(
            trigger_list.GPOS_Lat[image]), float(trigger_list.GPOS_Lon[image]), float(trigger_list.GPOS_Alt[image]))


def main():
    """
        Main method
    """
    parser = OptionParser()
    parser.add_option("-l", "--logfile", dest="LogFile",
                      help="PX4 log file containing recorded positions",
                      metavar="string")
    parser.add_option("-i", "--input", dest="InputFolder",
                      help="Input folder containing untagged images in alphabetical order",
                      type="string")
    parser.add_option("-o", "--output", dest="OutputFolder",
                      help="Output folder to contain tagged images",
                      type="string")

    (options, args) = parser.parse_args()
    if not options.LogFile:
        print "please type python " \
              "geotagging.py --logfile=[filename] --intput=[folder] [--output=[folder]]"
    elif not options.InputFolder:
        print "please type python " \
              "geotagging.py --logfile=[filename] --intput=[folder] [--output=[folder]]s"
    else:
        trigger_list = LoadPX4log(options.LogFile)
        image_list = LoadImageList(options.InputFolder)

        if not options.OutputFolder:
            options.OutputFolder = "imagesWithTag"
        if not os.path.exists(options.OutputFolder):
            os.makedirs(options.OutputFolder)

        trigger_list = FilterTrigger(trigger_list, image_list)

        TagImages(trigger_list, image_list, options.OutputFolder)

if __name__ == "__main__":
    main()
