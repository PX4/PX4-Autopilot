/* builtin command list - automatically generated, do not edit */
#include <nuttx/config.h>
#include <nuttx/binfmt/builtin.h>
#include <nuttx/config.h>
uint8_t romfs_img_len = 0;
uint8_t romfs_img[] = {};
${builtin_apps_decl_string}
const struct builtin_s g_builtins[] = {
${builtin_apps_string}
	{NULL, 0, 0, NULL}
};
const int g_builtin_count = ${command_count};
