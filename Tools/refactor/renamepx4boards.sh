#!/bin/bash
for f in `ls px4[!-]*`; do git mv $f `echo $f | sed -e "s~^px4~px4-~"`; done
