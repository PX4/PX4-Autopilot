#!/usr/bin/env bash
#
# Make sure the git submodules are ready to build, before any build starts:
#
#   at the commit PX4 records   nothing to do
#   missing                     fetch it
#   at another commit           warn and build it as it is (an error in CI)
#
# A submodule at another commit is never reset: that is how changes to a
# submodule are developed and tested. Set GIT_SUBMODULES_ARE_EVIL to skip
# the check entirely.

[ -n "$GIT_SUBMODULES_ARE_EVIL" ] && exit 0

cd "$(dirname "$0")/.." || exit 1

# nothing to check outside a git checkout, e.g. a source archive
git rev-parse --is-inside-work-tree > /dev/null 2>&1 || exit 0

# Fetch the missing submodules of one repository ('-' in git submodule status),
# naming only those so submodules at another commit are left alone. Run in
# PX4 and then, through foreach, in every submodule, which reaches nested
# submodules missing inside a submodule someone changed.
fetch_missing='
	missing=$(git submodule status | sed -n "s/^-[0-9a-f]* \([^ ]*\).*/\1/p")
	[ -z "$missing" ] || {
		echo "Fetching submodules in ${displaypath:-.}:" $missing
		git submodule --quiet sync --recursive -- $missing &&
		git submodule --quiet update --init --recursive --jobs 8 -- $missing
	}'
eval "$fetch_missing" && git submodule --quiet foreach --recursive "$fetch_missing" || {
	echo -e "\033[31mError: could not fetch submodules\033[0m"
	exit 1
}

# '+' at another commit, 'U' merge conflict
changed=$(git submodule status --recursive | sed -n "s/^[+U][0-9a-f]* \([^ ]*\).*/   \1/p")
if [ -n "$changed" ]; then
	if [ "$CI" = "true" ]; then
		echo -e "\033[31mError: submodules not at the commit PX4 records:\033[0m"
	else
		echo -e "\033[33mWarning: building submodules not at the commit PX4 records:\033[0m"
	fi
	echo "$changed"
	echo "To check out the recorded commits, run:"
	echo -e "   \033[94mgit submodule sync --recursive && git submodule update --init --recursive\033[0m"
	# CI must build exactly what the commit under test records
	[ "$CI" = "true" ] && exit 1
fi

exit 0
