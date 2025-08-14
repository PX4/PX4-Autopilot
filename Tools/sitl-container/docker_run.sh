#!/bin/bash

sudo docker run \
  --rm -it \
  --device /dev/fuse --cap-add SYS_ADMIN \
  -p 14540:14540/udp \
  -p 14550:14550/udp \
  -p 8888:8888/udp \
  px4-sitl:local
