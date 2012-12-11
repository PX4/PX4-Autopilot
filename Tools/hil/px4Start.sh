#!/bin/bash
./jsbsim/runsim.py \
    --script=jsbsim/rascal_test.xml \
    --home=36,128,1000,0 \
    --simout=127.0.0.1:49006 \
    --simin=127.0.0.1:49000 \
    --wind=0,0,0 \
    --fgout=127.0.0.1:6000
