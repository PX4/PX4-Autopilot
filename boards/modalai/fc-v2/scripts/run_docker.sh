#!/bin/bash

# Run this from the px4 project top level directory
docker run -it --rm --privileged -v `pwd`:/usr/local/workspace px4io/px4-dev-nuttx-focal:2022-08-12
