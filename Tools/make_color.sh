#!/bin/sh
# make_color.sh
# 
#   Author: Simon Wilks (simon@uaventure.com)
#
# A compiler color coder.
#
# To invoke this script everytime you run make simply create the alias:
#
#     alias make='<your-firmware-directory>/Tools/make_color.sh'
#
# Color codes:
#
# white        "\033[1,37m"
# yellow       "\033[1,33m"
# green        "\033[1,32m"
# blue         "\033[1,34m"
# cyan         "\033[1,36m"
# red          "\033[1,31m"
# magenta      "\033[1,35m"
# black        "\033[1,30m"
# darkwhite    "\033[0,37m"
# darkyellow   "\033[0,33m"
# darkgreen    "\033[0,32m"
# darkblue     "\033[0,34m"
# darkcyan     "\033[0,36m"
# darkred      "\033[0,31m"
# darkmagenta  "\033[0,35m"
# off          "\033[0,0m"
#
OFF="\o033[0m"
WARN="\o033[1;33m"
ERROR="\o033[1;31m" 
INFO="\o033[0;37m"

make ${@} 2>&1 | sed "s/make\[[0-9]\].*/$INFO & $OFF/;s/.*: warning: .*/$WARN & $OFF/;s/.*: error: .*/$ERROR & $OFF/" 
