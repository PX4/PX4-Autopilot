#!/bin/sh

rm -rf include/mavlink/v1.0/px4
cd share/pyshared/pymavlink
python generator/mavgen.py --lang=C --wire-protocol=1.0 -o ../../../include/mavlink/v1.0/ ../../../message_definitions/v1.0/px4.xml
