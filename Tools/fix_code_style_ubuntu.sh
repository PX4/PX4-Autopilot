#!/bin/sh
astyle \
    --style=linux		\
    --indent=force-tab=8	\
    --indent-cases		\
    --indent-preprocessor	\
    --break-blocks=all		\
    --pad-oper			\
    --pad-header		\
    --unpad-paren		\
    --keep-one-line-blocks	\
    --keep-one-line-statements	\
    --align-pointer=name	\
    --suffix=none		\
    --lineend=linux		\
    $*
    #--ignore-exclude-errors-x	\
    #--exclude=EASTL		\
    #--align-reference=name	\
