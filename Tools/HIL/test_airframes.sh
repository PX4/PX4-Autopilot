#! /bin/bash

# exit when any command fails
set -e

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

SERIAL_DEVICE=$1

if [ ! -e "${SERIAL_DEVICE}" ]
then
	echo "Invalid serial device ${SERIAL_DEVICE}"
	exit -1
fi

# all airframes (from ROMFS/px4fmu_common/init.d/airframes/)
# $(find . -regex '.*/[0-9].*' -exec basename {} \; | cut -d "_" -f 1)
ALL_AIRFRAMES=${@:2}
echo "airframes: ${ALL_AIRFRAMES}"

for airframe in $ALL_AIRFRAMES
do
	echo
	echo
	echo
	echo "########################################################################################################################"
	echo " Airframe: $airframe"
	echo "########################################################################################################################"
	echo

	${DIR}/nsh_param_set.py --device ${SERIAL_DEVICE} --name SYS_AUTOSTART  --value $airframe
	${DIR}/nsh_param_set.py --device ${SERIAL_DEVICE} --name CBRK_BUZZER    --value 782097

	# enable all mavlink instances
	${DIR}/run_nsh_cmd.py --device ${SERIAL_DEVICE} --cmd 'param set MAV_0_CONFIG 101' || true
	${DIR}/run_nsh_cmd.py --device ${SERIAL_DEVICE} --cmd 'param set MAV_1_CONFIG 102' || true
	${DIR}/run_nsh_cmd.py --device ${SERIAL_DEVICE} --cmd 'param set MAV_2_CONFIG 103' || true

	# enable all GPS
	${DIR}/run_nsh_cmd.py --device ${SERIAL_DEVICE} --cmd 'param set GPS_1_CONFIG 201' || true
	${DIR}/run_nsh_cmd.py --device ${SERIAL_DEVICE} --cmd 'param set GPS_1_CONFIG 202' || true

	${DIR}/run_nsh_cmd.py --device ${SERIAL_DEVICE} --cmd 'param reset SYS_HITL'
	${DIR}/run_nsh_cmd.py --device ${SERIAL_DEVICE} --cmd 'param status'
	${DIR}/run_nsh_cmd.py --device ${SERIAL_DEVICE} --cmd 'param save'
	${DIR}/run_nsh_cmd.py --device ${SERIAL_DEVICE} --cmd 'bsondump /fs/mtd_params'

	${DIR}/reboot.py --device ${SERIAL_DEVICE}

	${DIR}/run_nsh_cmd.py --device ${SERIAL_DEVICE} --cmd 'param status'
	${DIR}/run_nsh_cmd.py --device ${SERIAL_DEVICE} --cmd 'bsondump /fs/mtd_params' || true
	${DIR}/run_nsh_cmd.py --device ${SERIAL_DEVICE} --cmd 'bsondump /fs/microsd/parameters_backup.bson' || true

	${DIR}/run_nsh_cmd.py --device ${SERIAL_DEVICE} --cmd 'ps'
	${DIR}/run_nsh_cmd.py --device ${SERIAL_DEVICE} --cmd 'work_queue status'

	${DIR}/run_nsh_cmd.py --device ${SERIAL_DEVICE} --cmd 'mavlink status' || true
	${DIR}/run_nsh_cmd.py --device ${SERIAL_DEVICE} --cmd 'mavlink stop-all'

done
