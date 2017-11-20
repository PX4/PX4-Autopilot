#!/bin/bash

# set up virtual screen to enable headless mode
Xvfb :1 -screen 0 1024x768x16 &
export DISPLAY=:1.0

# start Jenkins slave
java -jar slave.jar -jnlpUrl http://${JENKINS_MASTER_URL}/computer/HIL_${HIL_HW}_${JENKINS_SLAVE_ID}/slave-agent.jnlp -secret ${JENKINS_SLAVE_SECRET} -workDir "/home/hil/"
#/bin/bash