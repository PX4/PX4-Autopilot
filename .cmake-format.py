dangle_parens = True
line_width = 120
tab_size = 4
max_subargs_per_line = 1

# If comment markup is enabled, don't reflow the first comment block in
# eachlistfile. Use this to preserve formatting of your
# copyright/licensestatements.
first_comment_is_literal = True

# Specify structure for custom cmake functions
additional_commands = {
  "px4_parse_function_args": {
    "kwargs": {
      "NAME": "*",
      "ONE_VALUE": "*",
      "MULTI_VALUE": "*",
      "REQUIRED": "*",
      "ARGN": "*"
    }
  },
  "px4_add_module": {
    "flags": [
      "EXTERNAL",
      "DYNAMIC",
      "UNITY_BUILD"
    ], 
    "kwargs": {
      "MODULE": "*",
      "MAIN": "*",
      "STACK": "*",
      "STACK_MAIN": "*",
      "STACK_MAX": "*",
      "PRIORITY": "*",
      "COMPILE_FLAGS": "*",
      "LINK_FLAGS": "*",
      "SRCS": "*",
      "INCLUDES": "*",
      "DEPENDS": "*",
      "MODULE_CONFIG": "*"
    }
  }  
}
