#!/bin/sh
#
# Copyright 2004, 2005 Free Software Foundation, Inc.
# Contributed by Ben Elliston <bje@gnu.org>.
#
# This test reads pairs from config-sub.data: an alias and its
# canonical triplet.  The config.sub scripts is invoked and the test
# checks that the alias expands to the expected canonical triplet.

verbose=false

function run_config_sub ()
{
    rc=0
    while read alias canonical ; do
	output=`sh ../config.sub $alias`
	if test $output != $canonical ; then
	    echo "FAIL: $alias -> $output, but expected $canonical"
	    rc=1
	else
	    $verbose && echo "PASS: $alias"
	fi
    done < config-sub.data
    return $rc
}

run_config_sub
rc=$?
if test $rc -eq 0 ; then
    $verbose || echo "PASS: config.sub checks"
else
    test $rc -eq 1 && echo "Unexpected failures."
fi

exit $rc
