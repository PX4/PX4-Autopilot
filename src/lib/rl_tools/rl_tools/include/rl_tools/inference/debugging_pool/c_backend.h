#ifdef RL_TOOLS_ENABLE_DEBUGGING_POOL
char rl_tools_debugging_pool_names[RL_TOOLS_DEBUGGING_POOL_NUMBER][RL_TOOLS_DEBUGGING_POOL_NAME_LENGTH];
float rl_tools_debugging_pool[RL_TOOLS_DEBUGGING_POOL_NUMBER][RL_TOOLS_DEBUGGING_POOL_SIZE];
uint64_t rl_tools_debugging_pool_indices[RL_TOOLS_DEBUGGING_POOL_NUMBER];
uint64_t rl_tools_debugging_pool_index = 0;
bool rl_tools_debugging_pool_locked = false;
bool rl_tools_debugging_pool_updated = false;
#endif

static void rl_tools_reset_debuging_pool(){
    if(!rl_tools_debugging_pool_locked){
        rl_tools_debugging_pool_index = 0;
        rl_tools_debugging_pool_updated = true;
    }
}
static void rl_tools_add_to_debuging_pool(const char* name, const float* values, uint64_t num){
    if(!rl_tools_debugging_pool_locked && rl_tools_debugging_pool_index < RL_TOOLS_DEBUGGING_POOL_NUMBER){
        uint32_t size = (num < RL_TOOLS_DEBUGGING_POOL_SIZE ? num : RL_TOOLS_DEBUGGING_POOL_SIZE);
        for(uint64_t i = 0; i < size; i++){
            rl_tools_debugging_pool[rl_tools_debugging_pool_index][i] = values[i];
        }
        if(portable_strlen(name) < RL_TOOLS_DEBUGGING_POOL_NAME_LENGTH){
            portable_strcpy(rl_tools_debugging_pool_names[rl_tools_debugging_pool_index], name);
        } else {
            portable_strcpy(rl_tools_debugging_pool_names[rl_tools_debugging_pool_index], "name too long");
        }
        rl_tools_debugging_pool_indices[rl_tools_debugging_pool_index] = size;
        rl_tools_debugging_pool_index++;
        rl_tools_debugging_pool_updated = true;
    }
}
