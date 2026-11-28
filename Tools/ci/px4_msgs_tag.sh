#!/usr/bin/env bash
# Tag PX4/px4_msgs to match a stable PX4 release tag.
#
# For a stable tag vX.Y.Z this checks that the release/X.Y branch of px4_msgs
# carries exactly the message and service definitions of that PX4 tag, sets
# the px4_msgs package version to X.Y.Z, and pushes that commit together with
# an annotated vX.Y.Z tag.
#
# Pre-release tags (alpha, beta, rc) are skipped. A tag that already sits on
# the tip of the px4_msgs release branch is left alone, so the script can be
# run again for the same tag.
#
# Both repositories are fetched into a scratch directory, so the result does
# not depend on the refs or the history of the checkout this runs from.
#
# Usage:
#   Tools/ci/px4_msgs_tag.sh <tag>
#
# Environment:
#   PX4_URL       PX4-Autopilot remote to read the tag from
#   PX4_MSGS_URL  px4_msgs remote to tag and push to

set -euo pipefail

PX4_URL="${PX4_URL:-https://github.com/PX4/PX4-Autopilot.git}"
PX4_MSGS_URL="${PX4_MSGS_URL:-https://github.com/PX4/px4_msgs.git}"

die() {
	echo "error: $*" >&2
	exit 1
}

tag="${1:-}"
if [ -z "$tag" ]; then
	echo "Usage: $0 <tag>" >&2
	exit 2
fi

if [[ ! "$tag" =~ ^v([0-9]+)\.([0-9]+)\.([0-9]+)$ ]]; then
	echo "${tag} is not a stable vX.Y.Z tag, skipping"
	exit 0
fi

version="${tag#v}"
branch="release/${BASH_REMATCH[1]}.${BASH_REMATCH[2]}"

work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT

px4="$work/px4.git"
git init --quiet --bare "$px4"
git -C "$px4" fetch --quiet --depth 1 "$PX4_URL" \
	"refs/tags/${tag}:refs/tags/${tag}" ||
	die "could not fetch ${tag} from ${PX4_URL}"
tag_date="$(git -C "$px4" for-each-ref \
	--format='%(creatordate:iso8601)' "refs/tags/${tag}")"

git clone --quiet "$PX4_MSGS_URL" "$work/px4_msgs"
cd "$work/px4_msgs"

git show-ref --verify --quiet "refs/remotes/origin/${branch}" ||
	die "px4_msgs has no ${branch} branch"
git checkout --quiet -B "$branch" "origin/${branch}"

head="$(git rev-parse HEAD)"
existing="$(git rev-parse --verify --quiet "refs/tags/${tag}^{commit}" || true)"
if [ -n "$existing" ]; then
	if [ "$existing" = "$head" ]; then
		echo "px4_msgs ${tag} is already on the tip of ${branch}, nothing to do"
		exit 0
	fi
	die "px4_msgs ${tag} is on ${existing}, but ${branch} is at ${head}"
fi

# Lay the PX4 definitions over the checkout the same way sync_to_px4_msgs.yml
# does. Anything git then reports as changed never reached px4_msgs.
paths=(msg)
if git -C "$px4" cat-file -e "refs/tags/${tag}:srv" 2>/dev/null; then
	paths+=(srv)
fi
mkdir "$work/px4"
git -C "$px4" archive "refs/tags/${tag}" "${paths[@]}" | tar -x -C "$work/px4"

for kind in msg srv; do
	rm -f "$kind"/*."$kind" "$kind"/versioned/*."$kind"
	[ -d "$work/px4/$kind" ] || continue
	mkdir -p "$kind"
	cp "$work/px4/$kind"/*."$kind" "$kind"/
	cp "$work/px4/$kind"/versioned/*."$kind" "$kind"/ 2>/dev/null || true
done

if [ -n "$(git status --porcelain -- msg srv)" ]; then
	git status --short -- msg srv
	die "definitions differ between PX4 ${tag} and px4_msgs ${branch}"
fi

sed "s:<version>.*</version>:<version>${version}</version>:" package.xml \
	> "$work/package.xml"
cat "$work/package.xml" > package.xml
grep -q "<version>${version}</version>" package.xml ||
	die "could not set the version in package.xml"

if ! git diff --quiet -- package.xml; then
	git commit --quiet -m "Set version to ${version}" -- package.xml
fi

GIT_COMMITTER_DATE="$tag_date" git tag -a "$tag" \
	-m "PX4 msgs and srvs definitions matching PX4 stable release ${version}"

git push --atomic origin "refs/heads/${branch}" "refs/tags/${tag}"
echo "px4_msgs ${tag} pushed on $(git rev-parse --short HEAD) (${branch})"
