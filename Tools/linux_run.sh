#!/bin/bash

if [ ! -c /tmp/ttyS0 ] || [ ! -c /tmp/ttyS1 ]
	then
	echo "Need to create /tmp/ttyS[01]"
	echo "socat PTY,link=/tmp/ttyS0 PTY,link=/tmp/ttyS1"
	exit 1
fi

if [ ! -d /fs/msdcard ] && [ ! -w /fs/msdcard ]
	then
	echo "Need to create/mount writable /fs/microsd"
	echo "sudo mkdir -p /fs/msdcard"
	echo "sudo chmod 777 /fs/msdcard"
	exit 1
fi

if [ ! -d /eeprom ] && [ ! -w /eeprom ]
	then
	echo "Need to create/mount writable /eeprom"
	echo "sudo mkdir -p /eeprom"
	echo "sudo chmod 777 /eeprom"
	exit 1
fi

Build/linux_default.build/mainapp linux-config/linuxtest/init/rc.S
