#!/usr/bin/env bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd ../../../../../../..
pwd
set -e
while true
do
  sudo time nice -n -20 $SCRIPT_DIR/build/tests/src/rl/environments/mujoco/ant/cuda/test_rl_environments_mujoco_ant_training_ppo_cuda 2>> $SCRIPT_DIR/benchmark_results.txt
  sleep 1
done
