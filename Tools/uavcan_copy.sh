#!/bin/bash
UAVCANFW=../ROMFS/px4fmu_common/uavcan/fw
ROMFS_REFIX=_
echo Removing : $UAVCANFW
rm -fr $UAVCANFW
for f in $(find firmware -type f -name "*.*.bin")
do
  UAVCAN_NAME=$(echo $f | cut -d"/" -f2 | cut -d. -f1-4 | cut -d- -f1-2)
  UAVCAN_HW=$(echo $f cut -d/ -f2 | cut -d. -f1-4 | cut -d- -f3)
  DST=${ROMFS_REFIX}$(echo $f | cut -d. -f3-7 | cut -d- -f1,2,3).$(echo $f | cut -d. -f6,7)
  # deal with legacy non conforming naming
  if [[ ${DST:(-7)} == bin.bin ]]
  then
    echo " WARNING: Improper name format!!!!!!!!! $f see should be <uavcan_name>-<HW_MAJOR>.<HW_MINOR)-<HW_MAJOR>.<HW_MINOR).<git hash[8]>.bin"
    DST=${DST%????}
  fi
  echo Processing file: $f Length:${#DST}
  if [ ${#DST} -le 28 ]
  then
    if [ -d "${UAVCANFW}/${UAVCAN_NAME}/${UAVCAN_HW}" ]
    then
      echo " ERROR: name colision directory ${UAVCANFW}/${UAVCAN_NAME}/${UAVCAN_HW} exits!"
      exit 2
    fi
    echo " Creating Directory ${UAVCANFW}/${UAVCAN_NAME}/${UAVCAN_HW}"
    mkdir -p ${UAVCANFW}/${UAVCAN_NAME}/${UAVCAN_HW}
    echo " Copying $f to ${UAVCANFW}/${UAVCAN_NAME}/${UAVCAN_HW}/${DST}"
    cp $f ${UAVCANFW}/${UAVCAN_NAME}/${UAVCAN_HW}/${DST}
  else
    echo " ERROR: $DST is ${#DST} charaters and needs to be less than or equal to 28"
    exit 1
  fi
done
exit 0
