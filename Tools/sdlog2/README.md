====== PX4 LOG CONVERSION ======

On each log session (commonly started and stopped by arming and disarming the vehicle) a new file logxxx.bin is created. In many cases there will be only one logfile named log001.bin (only one flight).

There are two conversion scripts in this ZIP file:

logconv.m: This is a MATLAB script which will automatically convert and display the flight data with a GUI. If running this script, the second script can be ignored.

sdlog2_dump.py: This is a Python script (compatible with v2 and v3) which converts the self-describing binary log format to a CSV file. To export a CSV file from within a shell (Windows CMD or BASH on Linux / Mac OS), run:

```
python sdlog2_dump.py log001.bin -f "export.csv" -t "TIME" -d "," -n ""
```

geo_tag_images.py: Use this script to geotag a set of images. It uses GPS time and file creation date to synchronize the images, so it needs that the images have a valid creation date. Can generate a KML file to view where the photos were taken in Google Earth (including height).

```sh
python geo_tag_images.py --logfile=mylog.bin --input=images/ --output=tagged/ --kml -v

python geo_tag_images.py -l=mylog.bin -i=images/ -o=tagged/ --kml -v
```

geotagging.py: Use this script to geotag a set of images. It uses the CAM trigger data from the log file for image association.
	
```sh
python geotagging.py --logfile=mylog.bin --input=images/ --output=tagged/
```

Python can be downloaded from http://python.org, but is available as default on Mac OS and Linux.
