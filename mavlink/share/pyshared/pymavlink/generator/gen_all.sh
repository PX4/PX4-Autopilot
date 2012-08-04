#!/bin/sh

for protocol in 0.9 1.0; do
 for xml in message_definitions/v$protocol/*.xml; do
     base=$(basename $xml .xml)
     ./mavgen.py --lang=C --wire-protocol=$protocol --output=C/include_v$protocol $xml || exit 1
     ./mavgen.py --lang=python --wire-protocol=$protocol --output=python/mavlink_${base}_v$protocol.py $xml || exit 1
 done
done

cp -f python/mavlink_ardupilotmega_v0.9.py ../mavlink.py
cp -f python/mavlink_ardupilotmega_v1.0.py ../mavlinkv10.py
