#!/usr/bin/env bash

DIR="$(dirname $(readlink -f $0))"
HOOK_FILE_DST="$DIR/../../.git/hooks/pre-push"
HOOK_FILE_SRC="$DIR/pre-push"

mkdir -p $DIR/../../.git/hooks
cp $HOOK_FILE_SRC $HOOK_FILE_DST
