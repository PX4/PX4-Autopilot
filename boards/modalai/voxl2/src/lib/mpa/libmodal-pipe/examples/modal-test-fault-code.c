#include "modal_pipe_common.h"

int main()
{
    fault_t fault = {
        .source = "test",
        .submodule = "test_submodule",
        .level = "test_level",
        .message = "test_message"
    };
    return write_fault_code(&fault);
}
