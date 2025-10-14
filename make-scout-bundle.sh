#!/usr/bin/env bash

make clean
make ark_fmu-v6x_default
make px4_fmu-v6x

tag=$(git describe --tags --match "*" --dirty)

tmp=$(mktemp -d)
mkdir -p $tmp/"px4-custom-$tag"
cp build/ark_fmu-v6x_default/*.px4 $tmp/"px4-custom-$tag"
cp build/px4_fmu-v6x_default/*.px4 $tmp/"px4-custom-$tag"

echo "rev: $(git rev-parse HEAD)" > $tmp/"px4-custom-$tag"/src-info
echo "remote: $(git remote get-url origin)" >> $tmp/"px4-custom-$tag"/src-info

dir=$(pwd)
pushd $tmp
tar -czvf "$dir/px4-custom-$tag.tar.gz" "px4-custom-$tag"
popd
