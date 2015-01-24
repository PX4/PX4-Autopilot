/* builtin command list - automatically generated, do not edit */
#include <nuttx/config.h>
#include <nuttx/binfmt/builtin.h>
extern int px4_simple_app_main(int argc, char *argv[]);
const struct builtin_s g_builtins[] = {
    {"px4_simple_app", SCHED_PRIORITY_DEFAULT, 1024, px4_simple_app_main},
    {NULL, 0, 0, NULL}
};
const int g_builtin_count = 1;
