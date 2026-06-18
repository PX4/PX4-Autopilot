SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
RL_TOOLS_INCLUDE_DIR=${RL_TOOLS_INCLUDE_DIR:-$SCRIPT_DIR/../../../../../../include}
RL_TOOLS_BUILD_DIR=${RL_TOOLS_BUILD_DIR:-./static/build}

echo RL_TOOLS_INCLUDE_DIR: $RL_TOOLS_INCLUDE_DIR
echo RL_TOOLS_BUILD_DIR: $RL_TOOLS_BUILD_DIR
cd $SCRIPT_DIR
mkdir -p $RL_TOOLS_BUILD_DIR
EXPORTED_FUNCTIONS="['_proxy_create_training_state', '_proxy_training_step', '_proxy_get_step', '_proxy_get_evaluation_count', '_proxy_get_evaluation_return', '_proxy_destroy_training_state', '_proxy_get_state_dim', '_proxy_get_state_value']"
em++ -DRL_TOOLS_STEP_LIMIT=10000 -DRL_TOOLS_BENCHMARK --std=c++17  -msimd128 -O3 -s WASM=1 -I$RL_TOOLS_INCLUDE_DIR -s EXPORTED_FUNCTIONS="$EXPORTED_FUNCTIONS" -s MODULARIZE=1 -s EXPORT_ES6=1  -s ENVIRONMENT='web' --pre-js prejs.js -o $RL_TOOLS_BUILD_DIR/wasm_interface_benchmark.js wasm_interface.cpp
em++ --std=c++17 -msimd128 -O3 -s WASM=1 -I$RL_TOOLS_INCLUDE_DIR -s EXPORTED_FUNCTIONS="$EXPORTED_FUNCTIONS" -s MODULARIZE=1 -s EXPORT_ES6=1 -s ENVIRONMENT='web' --pre-js prejs.js -o $RL_TOOLS_BUILD_DIR/wasm_interface.js wasm_interface.cpp
