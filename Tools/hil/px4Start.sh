#!/bin/bash
./jsbsim/runsim.py \
    --script=jsbsim/easystar_test \
    --home=37.6166110,-122.4161053,25,0 \
    --fgout=127.0.0.1:5503 \
    --simout=127.0.0.1:49006 \
    --simin=127.0.0.1:49000 \
    --wind=0,0,0
