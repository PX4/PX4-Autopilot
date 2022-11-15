#!/bin/bash

# Run this from the px4 project top level directory
docker run -it --rm -v `pwd`:/usr/local/workspace rb5-flight-px4-build-docker
