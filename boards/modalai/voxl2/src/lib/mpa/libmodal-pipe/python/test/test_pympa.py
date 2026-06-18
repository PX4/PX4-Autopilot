#!/usr/bin/python3

import time
import numpy as np
from pympa import *

print('Subscribing to camera channel')

max_width = 5000;
max_height = 4000;
image_buffer = np.zeros(max_width*max_height, dtype='uint8')

meta = camera_image_metadata_t()

channel = pympa_camera_subscribe('hires_small_color', 'test_pympa')

if channel != -1:
    print('Subscribed to camera channel: ' + str(channel))

    print('Trying to get an image...')
    time.sleep(0.25)

    img_size = pympa_get_image(channel, image_buffer, meta)

    if img_size:
        print('Got an image of size: ' + str(img_size))
    else:
        print('Couldn\'t get an image')

    pympa_close_sub(channel)



else:
    print('Failed to subscribe to camera channel')

print('Finished camera subscription test, trying camera publish')

channel = pympa_create_pub('pympa_test', 'test_pympa', 'camera_image_metadata_t')

if channel != -1:
    print('Created camera pulisher channel: ' + str(channel))

    pympa_publish_image(channel, image_buffer, meta)

    # Sleep a little so that you can go check /run/mpa to see if the pipe is there
    time.sleep(5)

    pympa_close_pub(channel)

else:
    print('Failed to create camera publishing channel')

print('testing complete')









