SCRIPT_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
RL_TOOLS_INCLUDE_DIR=${RL_TOOLS_INCLUDE_DIR:-$SCRIPT_DIR/../../../../include}
JSON_INCLUDE_DIR=${JSON_INCLUDE_DIR:-/usr/include}
RL_TOOLS_BUILD_DIR=${RL_TOOLS_BUILD_DIR:-./static/build}


echo RL_TOOLS_INCLUDE_DIR: $RL_TOOLS_INCLUDE_DIR
echo RL_TOOLS_BUILD_DIR: $RL_TOOLS_BUILD_DIR
cd $SCRIPT_DIR
mkdir -p $RL_TOOLS_BUILD_DIR
EXPORTED_FUNCTIONS="['_proxy_create', '_proxy_step', '_proxy_step_message', '_proxy_destroy', '_proxy_num_messages', '_proxy_pop_message', '_proxy_delete_message']"
#EXPORTED_FUNCTIONS="['_proxy_create_training_state', '_proxy_training_step', '_proxy_get_step', '_proxy_get_evaluation_count', '_proxy_get_evaluation_return', '_proxy_destroy_training_state', '_proxy_get_state_dim', '_proxy_get_state_value']"
echo JSON_INCLUDE_DIR: $JSON_INCLUDE_DIR
em++ --std=c++17 -msimd128 -O3 -s WASM=1 -I$RL_TOOLS_INCLUDE_DIR -I$JSON_INCLUDE_DIR -s EXPORTED_FUNCTIONS="$EXPORTED_FUNCTIONS" -s MODULARIZE=1 -s EXPORT_ES6=1 -s EXPORTED_RUNTIME_METHODS=UTF8ToString,stringToNewUTF8 -s ENVIRONMENT='web' --pre-js prejs.js -o $RL_TOOLS_BUILD_DIR/wasm_interface.js wasm_interface.cpp
