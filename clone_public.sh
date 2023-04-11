#!/bin/bash

set -euo pipefail

while read -r repo
do
  [[ "${repo}" == *saluki-?? ]] || \
  [[ "${repo}" == *pfsoc_crypto ]]  || \
  [[ "${repo}" == *pfsoc_keystore ]]  || \
  [[ "${repo}" == *pf_crypto ]] && continue
  git submodule update --init --recursive "${repo}"
done <<< "$(git submodule status | awk '{print $2}')"
