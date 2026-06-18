set -e
SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"
RL_TOOLS_ROOT=$(cd $SCRIPTPATH/../..; pwd -P)
echo $RL_TOOLS_ROOT

(cd /tmp && rm -rf rl_tools_build_bare; mkdir rl_tools_build_bare && cd rl_tools_build_bare && cmake $RL_TOOLS_ROOT && cmake --build . -j16)

echo "Success!"