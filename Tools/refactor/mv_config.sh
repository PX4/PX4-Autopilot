#!/bin/bash
# $1 is srcdir
# $2 is ${OS}/boards dir
src=`realpath $1`
dest=`realpath $2`
for f in `cd $src && ls *.cmake `; do
	cd $dest
	echo $dest
	boarddir="$dest/`echo $f | cut -d'_' -f2`"
	mkdir -p $boarddir
	git mv $src/$f $boarddir/config_`echo $f | cut -d'_' -f 3`
done

