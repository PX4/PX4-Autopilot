#!/bin/bash

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

Build/posix_default.build/mainapp posix-configs/posixtest/init/rc.S
