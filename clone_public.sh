#!/bin/bash

set -euo pipefail

while read -r repo
do
  [[ "${repo}" == *saluki-v? ]] || [[ "${repo}" == *pfsoc_crypto ]] && continue
  git submodule update --init --recursive "${repo}"
done <<< "$(git submodule status | awk '{print $2}')"
