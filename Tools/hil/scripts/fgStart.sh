#!/bin/bash
aircraft=easystar
portFdm=5503

fgfs --fdm=external \
--aircraft=$aircraft \
--native-fdm=socket,in,30,,$portFdm,udp \
--prop:/sim/frame-rate-throttle-hz=30 \
--lat=37.6166110 \
--lon=-122.4161053 \
--timeofday=noon \
--altitude=1000 \
--heading=90 \
--geometry=400x300 \
--vc=0 \
--roll=0 \
--pitch=0 \
--wind=0@0 \
--turbulence=0.0 \
--timeofday=noon \
--disable-sound \
--notrim
