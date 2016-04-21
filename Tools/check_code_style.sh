#!/usr/bin/env bash

file=$1

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

if [ -f "$file" ];
then
  ${DIR}/fix_code_style.sh --quiet < $file > $file.pretty
  diffsize=$(diff -y --suppress-common-lines $file $file.pretty | wc -l)
  rm -f $file.pretty
  if [ $diffsize -ne 0 ]; then
    echo $file 'bad formatting, please run "./Tools/fix_code_style.sh' $file'"'
    exit 1
  fi
fi

