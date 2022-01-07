#!/bin/sh

port=`cat /proc/tty/driver/serial | grep DTR | cut -d':' -f1`
echo "${port}"

