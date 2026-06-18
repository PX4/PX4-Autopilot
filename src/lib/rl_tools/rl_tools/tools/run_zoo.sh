# note this should be run from the project dir
# usage: ./tools/run.sh /home/jonas/rl-tools/cmake-build-release/src/rl/zoo/rl_zoo_l2f_sac [start-seed] [end-seed] [num-processes]
if [ "$#" -ne 4 ]; then
    echo "Usage: $0 <path-to-executable> <start-seed> <end-seed> <num-processes>"
    exit 1
fi
set -e
export MKL_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=$MKL_NUM_THREADS
export RL_TOOLS_EXTRACK_EXPERIMENT=$(date '+%Y-%m-%d_%H-%M-%S')
echo "RL_TOOLS_EXTRACK_EXPERIMENT=$RL_TOOLS_EXTRACK_EXPERIMENT"
echo "Dummy run (to create dictionaries) -----------------"
timeout 2s $1 -s $2 --extrack-experiment $RL_TOOLS_EXTRACK_EXPERIMENT || true
echo "Real run -------------------------------------------"
seq $2 $3 | xargs -P $4 -I {} $1 -s {} --extrack-experiment $RL_TOOLS_EXTRACK_EXPERIMENT