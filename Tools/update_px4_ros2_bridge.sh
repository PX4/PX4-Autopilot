#!/usr/bin/env bash
set -e

agent_template_files_updated=0
code_generator_files_updated=0

# parse help argument
if [[ $1 == "-h" ]] || [[ $1 == "--help" ]]; then
  echo -e "Usage: update_px4_ros2_bridge.bash [options...] \t This script allows to update px4_ros_com and px4_msgs in a specified directory." >&2
  echo
  echo -e "\t--ws_dir \t\t Location of the ament/colcon workspace. Default: $HOME/colcon_ws."
  echo -e "\t--px4_ros_com \t\t Updates px4_ros_com microRTPS agent code generation and templates."
  echo -e "\t--px4_msgs \t\t Updates px4_msgs messages definition files."
  echo -e "\t--all \t\t Updates both px4_ros_com and px4_msgs."
  echo
  exit 0
fi

# parse the arguments
while [ $# -gt 0 ]; do
  if [[ $1 == *"--"* ]]; then
    v="${1/--/}"
    if [ ! -z $2 ]; then
      declare $v="$2"
    else
      declare $v=1
    fi
  fi
  shift
done

# get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# get PX4-Autopilot directory
PX4_DIR=$(cd "$(dirname "$SCRIPT_DIR")" && pwd)

function compare_and_update () {
  cmp -s "$1" "$2"
  if [ $? -eq 1 ]; then
    cp "$1" "$2"
    return 0
  fi

  return 1;
}

# update microRTPS agent code generators / helpers
function update_agent_code {
  declare -a templates=( \
    "microRTPS_agent.cpp.em" \
    "microRTPS_timesync.cpp.em" \
    "microRTPS_timesync.h.em" \
    "microRTPS_transport.cpp" \
    "microRTPS_transport.h" \
    "Publisher.cpp.em" \
    "Publisher.h.em" \
    "Subscriber.cpp.em" \
    "Subscriber.h.em" \
    "RtpsTopics.cpp.em" \
    "RtpsTopics.h.em" \
    )

  for file in ${templates[@]}; do
    compare_and_update "$PX4_DIR/msg/templates/urtps/$file" "$ws_dir/src/px4_ros_com/templates/$file" \
      && echo -e "--\t\t- '$ws_dir/src/px4_ros_com/templates/$file' updated" && ((agent_template_files_updated+=1))
  done
  if [ $agent_template_files_updated -eq 0 ]; then
    echo -e "--\t\t- No template files updated"
  elif [ $agent_template_files_updated -eq 1 ]; then
    echo -e "--\t\t  - 1 template file updated"
  else
    echo -e "--\t\t  - $agent_template_files_updated template files updated"
  fi
}

# update microRTPS agent code templates
function update_agent_templates {
  declare -a code_generators=( \
    "uorb_rtps_classifier.py" \
    "generate_microRTPS_bridge.py" \
    "px_generate_uorb_topic_files.py" \
    )
  for file in ${code_generators[@]}; do
    compare_and_update "$PX4_DIR/msg/tools/$file $ws_dir/src/px4_ros_com/scripts/$file" \
      && echo -e "--\t\t- '$ws_dir/src/px4_ros_com/scripts/$file' updated" && ((code_generator_files_updated+=1))
  done
  if [ $code_generator_files_updated -eq 0 ]; then
    echo -e "--\t\t- No code generators / helpers files updated"
  elif [ $code_generator_files_updated -eq 1 ]; then
    echo -e "--\t\t  - 1 code generator / helper file updated"
  else
    echo -e "--\t\t  - $code_generator_files_updated code generator / helper files updated"
  fi
}

# update px4_ros_com files
function update_px4_ros_com {
  python3 $PX4_DIR/msg/tools/uorb_to_ros_urtps_topics.py -i $PX4_DIR/msg/tools/urtps_bridge_topics.yaml -o $ws_dir/src/px4_ros_com/templates/urtps_bridge_topics.yaml
  echo -e "---------------          \033[1mmicroRTPS agent code generation and templates update\033[0m          ----------------"
  echo "-------------------------------------------------------------------------------------------------------"
  update_agent_code
  update_agent_templates
  return 0
}

# function to update px4_msgs
function update_px4_msgs {
  find "$ws_dir/src/px4_msgs/msg/" -maxdepth 1 -type f -delete
  python3 $PX4_DIR/msg/tools/uorb_to_ros_msgs.py $PX4_DIR/msg/ $ws_dir/src/px4_msgs/msg/
}

# decisor
echo "-------------------------------------------------------------------------------------------------------"
if [ -d "${ws_dir}" ]; then
  ws_dir=$(cd "$ws_dir" && pwd)
  if [ ! -z ${all} ]; then
    update_px4_ros_com
    echo "-------------------------------------------------------------------------------------------------------"
    update_px4_msgs
  elif [ ! -z ${px4_ros_com} ]; then
    update_px4_ros_com
    echo "-------------------------------------------------------------------------------------------------------"
  elif [ ! -z ${px4_msgs} ]; then
    update_px4_msgs
  fi
  echo -e "--------------------------------          \033[0;32mUpdate successful!\033[0m          ---------------------------------"
  echo "-------------------------------------------------------------------------------------------------------"
  exit 0
else
  echo -e "-- \033[0;31mWorkspace directory doesn't exist...\033[0m"
  echo -e "----------------------------------          \033[0;31mUpdate failed!\033[0m          -----------------------------------"
  echo "-------------------------------------------------------------------------------------------------------"
  exit $ERRCODE
fi
