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
    --align-reference=name	\
    --suffix=none		\
    --ignore-exclude-errors-x	\
    --lineend=linux		\
    --exclude=EASTL		\
    --add-brackets		\
    --max-code-length=120	\
    --preserve-date             \
    $*
