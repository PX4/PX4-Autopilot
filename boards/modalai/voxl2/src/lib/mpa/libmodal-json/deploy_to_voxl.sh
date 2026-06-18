#!/bin/bash
################################################################################
# Copyright (c) 2025 ModalAI, Inc. All rights reserved.
#
# Installs the deb package on target.
# Requires the package to be built and an adb or ssh connection.
################################################################################
set -e

SSH_PWD="oelinux123"


print_usage(){
	echo ""
	echo " Deploy a deb package to VOXL over adb or ssh"
	echo " You must run ./make_package.sh first to build the package."
	echo ""
	echo " Usage to push over adb:"
	echo "  ./deploy_to_voxl.sh"
	echo "  ./deploy_to_voxl.sh adb"
	echo ""
	echo " Usage to push over ssh:"
	echo "   if VOXL_IP env variable is set:"
	echo "  ./deploy_to_voxl.sh ssh"
	echo ""
	echo "   manually specifying a full IP address:"
	echo "  ./deploy_to_voxl.sh ssh 192.168.1.123"
	echo ""
	echo "   manually specifying last block of the IP"
	echo "  ./deploy_to_voxl.sh ssh 123"
	echo ""
}


#check adb (default) or ssh or print help text
if [ "$#" -lt 1 ]; then
	#no options = default adb
	DEPLOY_MODE="ADB"
else
	#Parse arg
	case $1 in

		"adb")
			DEPLOY_MODE="adb"
			;;

		"ssh")
			DEPLOY_MODE="ssh"
			#check command line for push ip
			if [ "$#" -gt 1 ]; then
				if  echo $2 | grep -xq "[0-9]*" ; then
					#echo "Number"
					SEND_IP=192.168.1.$2
				elif  echo $2 | grep -xq "[0-9]*\.[0-9]*\.[0-9]*\.[0-9]*" ; then
					#echo "Full IP"
					SEND_IP=$2
				else
					echo "Invalid IP address: $2"
					exit 1
				fi
			elif ! [ -z ${VOXL_IP+x} ]; then
				SEND_IP="${VOXL_IP}"
			else
				echo "Did not find a VOXL_IP env variable,"
				echo ""
				echo "If you would like to push over ssh automatically,"
				echo "please export VOXL_IP in your bashrc"
				echo ""
				read -p "Please enter an IP to push to:" SEND_IP
			fi
			;;

		"h"|"-h"|"help"|"--help")
			print_usage
			exit 0
			;;

		*)
			print_usage
			exit 1
			;;
	esac
fi



# count package files in current directory
NUM_DEB=$(ls -1q *.deb 2>/dev/null | wc -l)

if [[ $NUM_DEB -eq "0" ]]; then
	echo "ERROR: missing deb"
	echo "run make_package.sh first"
	exit 1
elif [[  $NUM_DEB -gt "1" ]]; then
	echo "ERROR: more than 1 deb file found"
	echo "make sure there is at most one of each in the current directory"
	exit 1
fi

DPKG_CHECK_STRING='command -v dpkg &> /dev/null; echo -n $?'

if [ "$DEPLOY_MODE" == "ssh" ]; then
	if ! command -v sshpass &> /dev/null ;then
		echo ""
		echo "You do not have sshpass installed"
		echo "Please install sshpass to use the install via ssh feature"
		echo ""
		exit 1
	fi

	echo "searching for ssh device"
	until ping -c1 $SEND_IP &>/dev/null; do :; done
	echo "checking VOXL for dpkg"

	if sshpass -p oelinux123 ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null root@$SEND_IP "$DPKG_CHECK_STRING" 2>/dev/null | grep -q 0 ; then
		echo "dpkg detected";
		FILE=$(ls -1q $PACKAGE*.deb)
		sshpass -p oelinux123 scp -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $FILE root@$SEND_IP:/data/$FILE &>/dev/null
		sshpass -p oelinux123 ssh -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null root@$SEND_IP "dpkg -i --force-downgrade --force-depends /data/$FILE" 2>/dev/null

	else
		echo "[ERROR] dpkg not found on VOXL"
		echo "Was the password for VOXL's root user changed? (this script assumes default of oelinux123)"
		exit 1
	fi

else
	if ! command -v adb &> /dev/null ;then
		echo ""
		echo "You do not have adb installed"
		echo "Please install adb to use the install via adb feature"
		echo ""
		exit 1
	fi

	echo "searching for ADB device"
	adb wait-for-device
	echo "checking VOXL for dpkg"

	if adb shell "$DPKG_CHECK_STRING" | grep -q 0 ; then
		echo "dpkg detected";
		FILE=(*.deb)
		adb push $FILE /data/$FILE
		adb shell "dpkg -i --force-downgrade --force-depends /data/$FILE"

	else
		echo "[ERROR] dpkg not found on VOXL"
		echo "Was the password for VOXL's root user changed? (this script assumes default of oelinux123)"
		exit 1
	fi
fi

echo "DONE"
