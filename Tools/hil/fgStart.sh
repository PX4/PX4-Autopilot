#!/bin/bash
num=1
aircraft=easystar
callsign=px4-autopilot
mpPortStart=5000
portMulti=$(($num+$mpPortStart))
portFdm=6000

fgfs --fdm=external \
--callsign=$callsign \
--aircraft=$aircraft \
--multiplay=in,10,$ip,$portMulti \
--multiplay=out,10,$mpServer,5000 \
--native-fdm=socket,in,10,,$portFdm,udp \
--prop:/sim/frame-rate-throttle-hz=30 \
--timeofday=noon \
--geometry=400x300 \
--vc=30 \
--altitude=1000 \
--heading=90 \
--roll=0 \
--pitch=0 \
--wind=0@0 \
--turbulence=0.0 \
--timeofday=noon \
--notrim
