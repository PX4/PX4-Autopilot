set -e
: "${DEEPSWEEP_SERVER:?DEEPSWEEP_SERVER is not set}"
: "${JOB_ID:?JOB_ID is not set}"

# work
cd rl-tools
while true; do
  echo curl --fail -X POST $DEEPSWEEP_SERVER/jobs/$JOB_ID/tasks
  task=$(curl --fail -X POST $DEEPSWEEP_SERVER/jobs/$JOB_ID/tasks)
  echo task: $task
  task_id=$(echo $task | jq -r '.task_id')
  echo $task $task_id
  CMD="MKL_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 RL_TOOLS_SAVE_TRAJECTORIES=0 RL_TOOLS_NN_ANALYTICS=0 RL_TOOLS_EXTRACK_EXPERIMENT=$JOB_ID ./build/src/foundation_policy/foundation_policy_pre_training $(echo $task | jq -r '.spec')"
  echo $CMD
  eval $CMD
  result='{"return": "finished"}'
  curl -X POST -H 'Content-Type: application/json' --data "$result" $DEEPSWEEP_SERVER/jobs/$JOB_ID/tasks/$task_id
done

# done
