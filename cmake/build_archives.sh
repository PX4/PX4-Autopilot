#!/bin/bash
d=$PWD && mkdir -p $d/build_arm && cd $d/build_arm && cmake .. && time make link_exports -j
