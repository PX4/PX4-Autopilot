#!/bin/bash

set -euxo pipefail

output_dir=$1
packaging/build_debian.sh ${output_dir}

exit 0
