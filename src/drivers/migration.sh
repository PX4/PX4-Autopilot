#!/bin/sh

HOME="/home/kalyan/Documents/src/PX4-Autopilot/src/drivers"

for d in $( find -type d ); do
	cd $d
	count=`ls -1 *.yaml 2>/dev/null | wc -l`
	[ $count == 0 ] && cd $HOME && continue
	echo $d
	[ -f "module.yaml" ] && mv module.yaml module.abc.yaml
	yq eval-all '. as $item ireduce ({}; . *+ $item)' *.yaml > module.yaml
	rm module.*.yaml
	rm *param*.c
	cd $HOME
done
