#!/bin/bash

if [ -z "$2" ]
  then
	echo "You need to specify the install path for the micro-RTPS agent first and then the path to fastrtpsgen script."
    exit
fi

CLIENT_FOLDER="src/examples/micrortps_client"
AGENT_FOLDER="$1"
FASTRTPSGEN_PATH="$2"
#"/PATH/TO/Fast-RTPS/fastrtpsgen/scripts"

if [ ! -d "${CLIENT_FOLDER}" ]; then
  mkdir -p ${CLIENT_FOLDER}
fi


cp msg/templates/urtps/UART_node.* ${CLIENT_FOLDER}
cp msgenerated/microRTPS_client_CMakeLists.txt ${CLIENT_FOLDER}/CMakeLists.txt
cp msgenerated/microRTPS_client.cpp ${CLIENT_FOLDER}

rm ${AGENT_FOLDER}/*.cxx ${AGENT_FOLDER}/*.h ${AGENT_FOLDER}/CMakeLists.txt 2>/dev/null

if [ ! -d "${AGENT_FOLDER}/idl" ]; then
  mkdir -p ${AGENT_FOLDER}/idl
fi
rm ${AGENT_FOLDER}/idl/* 2>/dev/null
cp msgenerated/*.idl ${AGENT_FOLDER}/idl

if [ ! -d "${AGENT_FOLDER}/fastrtpsgen" ]; then
  mkdir -p ${AGENT_FOLDER}/fastrtpsgen
fi
cd ${AGENT_FOLDER}/fastrtpsgen
rm ${AGENT_FOLDER}/fastrtpsgen/*.cxx ${AGENT_FOLDER}/fastrtpsgen/*.h ${AGENT_FOLDER}/fastrtpsgen/makefile* 2>/dev/null
for idl_file in ${AGENT_FOLDER}/idl/*.idl; do
	${FASTRTPSGEN_PATH}/fastrtpsgen -example x64Linux2.6gcc $idl_file
done

if [ ! -d "${AGENT_FOLDER}/build" ]; then
  mkdir -p ${AGENT_FOLDER}/build
fi
rm ${AGENT_FOLDER}/build/* 2>/dev/null
cp * ../
rm ../*PubSubMain.cxx ../makefile*
cd -

cp msg/templates/urtps/UART_node.* ${AGENT_FOLDER}
cp msgenerated/microRTPS_agent_CMakeLists.txt ${AGENT_FOLDER}/CMakeLists.txt
cp msgenerated/*Publisher* ${AGENT_FOLDER} 2>/dev/null
cp msgenerated/*Subscriber* ${AGENT_FOLDER} 2>/dev/null
cp msgenerated/microRTPS_agent.cxx ${AGENT_FOLDER}

#rm * && cmake .. -DCMAKE_PREFIX_PATH=/PATH/TO/Fast-RTPS/install && make all
