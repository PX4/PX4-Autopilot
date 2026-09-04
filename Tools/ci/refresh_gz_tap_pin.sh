#!/usr/bin/env bash
#
# Move Tools/setup/gz-tap-pin.txt to the newest osrf/simulation commit whose
# Gazebo formulae all have a bottle Homebrew would pour on this machine.
#
# macos.sh --sim-tools pins the osrf/simulation tap to gz-tap-pin.txt so
# Gazebo pours from bottles even while OSRF has them pulled (see that file).
# The pin is a point in time, so this walks the tap's first-parent history
# from origin HEAD back to the current pin, checks each commit out, and asks
# Homebrew whether every osrf/simulation formula macos.sh installs, plus
# their osrf/simulation runtime dependencies, carries a bottle for this
# host. Each bottle tarball is then HEAD-requested so a bottle block whose
# tarball is gone does not count. The first commit that passes becomes the
# new pin. The tap checkout is restored on exit either way.
#
# A formula that has no bottle at the current pin either is not required to
# have one: in practice that is the gz-harmonic meta-formula, which OSRF
# never bottles and which builds in seconds. Everything that poured at the
# pin has to keep pouring.
#
# The formula names are read from macos.sh so there is no second list to
# keep in sync. The bottle tag match is Homebrew's own, which on macOS also
# accepts a bottle built on an older macOS of the same arch, so a commit
# that passes on the oldest macOS CI runs on passes on the newer ones too.
#
# Exits 0 whether or not a newer bottled commit exists; only errors exit
# non-zero. Under GitHub Actions the result also lands in $GITHUB_OUTPUT as
# needs_bump=true|false and old_pin, plus new_pin and new_pin_short when
# needs_bump is true.
#
# Usage: Tools/ci/refresh_gz_tap_pin.sh

set -euo pipefail

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
ROOT_DIR=$(git -C "$DIR" rev-parse --show-toplevel)
PIN_FILE="${ROOT_DIR}/Tools/setup/gz-tap-pin.txt"
MACOS_SH="${ROOT_DIR}/Tools/setup/macos.sh"
TAP="osrf/simulation"

log() { echo "[refresh_gz_tap_pin] $*"; }
die() { log "ERROR: $*" >&2; exit 1; }

output() {
	log "$1=$2"
	if [[ -n ${GITHUB_OUTPUT:-} ]]; then
		echo "$1=$2" >> "$GITHUB_OUTPUT"
	fi
}

# Runs inside Homebrew via `brew ruby`. ARGV is the tap name followed by
# the osrf/simulation formulae macos.sh installs (brew scrubs everything
# but HOMEBREW_* from the environment, so arguments it is). Prints one line
# per osrf/simulation formula in their runtime closure, either
# "bottled <name> <version> <url>" or "no bottle <name> <version>", and
# exits 3 if any bottle is missing.
#
# Formula#bottle is deliberately not used: `brew ruby` runs in a child of
# the brew process, which has already consumed the HOMEBREW_*_DEFAULT_PREFIX
# variables Homebrew derives its default cellar from, so under `brew ruby`
# every bottle without an explicit cellar looks like it was built in a
# foreign prefix. The tag match and the bottle URL do not depend on that.
read -r -d '' BOTTLED_RB <<'EOF' || true
tap = ARGV.shift
host = Utils::Bottles.tag
formulae = {}
ARGV.each do |name|
  root = Formulary.factory(name)
  deps = root.recursive_dependencies do |_, dep|
    next Dependable::PRUNE if dep.build? || dep.test? || dep.optional?
  end
  ([root] + deps.map(&:to_formula)).each do |f|
    formulae[f.full_name] = f if f.tap&.name == tap
  end
end
missing = 0
formulae.each_value do |f|
  bottle = f.bottle_for_tag(host) if f.pour_bottle?
  if bottle
    puts "bottled #{f.full_name} #{f.pkg_version} #{bottle.url}"
  else
    puts "no bottle #{f.full_name} #{f.pkg_version}"
    missing += 1
  end
end
exit(missing.zero? ? 0 : 3)
EOF

command -v brew > /dev/null || die "Homebrew is required"

# The osrf/simulation formulae macos.sh installs. Read from the script so
# this never drifts from PX4_SIM_BREW_PACKAGES.
ROOTS=$(grep -oE "${TAP}/[A-Za-z0-9@._+-]+" "$MACOS_SH" | sort -u | tr '\n' ' ')
[[ -n $ROOTS ]] || die "macos.sh installs nothing from ${TAP}"

OLD_PIN=$(grep -v '^#' "$PIN_FILE" | tr -d '[:space:]')
[[ $OLD_PIN =~ ^[0-9a-f]{40}$ ]] || die "no commit SHA in ${PIN_FILE}"

# Keep brew from resetting the tap checkout or nagging mid-run.
export HOMEBREW_NO_AUTO_UPDATE=1 HOMEBREW_NO_ENV_HINTS=1

brew tap "$TAP"
# Homebrew 6.0+ refuses to load formulae from untrusted taps; same guard
# as macos.sh.
if brew trust --help &> /dev/null; then
	brew trust "$TAP"
fi
TAP_DIR=$(brew --repo "$TAP")

git_tap() { git -C "$TAP_DIR" "$@"; }

[[ -z $(git_tap status --porcelain) ]] || die "${TAP_DIR} has local changes, refusing to touch it"
ORIG_REF=$(git_tap symbolic-ref --quiet --short HEAD || git_tap rev-parse HEAD)
restore_tap() { git_tap checkout --quiet "$ORIG_REF"; }
trap restore_tap EXIT

if [[ $(git_tap rev-parse --is-shallow-repository) == true ]]; then
	git_tap fetch --quiet --unshallow
fi
git_tap fetch --quiet origin
git_tap remote set-head origin --auto > /dev/null
TAP_HEAD=$(git_tap rev-parse origin/HEAD)

git_tap merge-base --is-ancestor "$OLD_PIN" origin/HEAD \
	|| die "pin ${OLD_PIN} is not an ancestor of ${TAP} HEAD ${TAP_HEAD}, bump it by hand"

# Bottle report for the checked-out tap commit; exit 3 means at least one
# formula has no bottle, anything else non-zero is an error.
bottle_report() {
	# shellcheck disable=SC2086 # word-split on purpose
	brew ruby -e "$BOTTLED_RB" "$TAP" $ROOTS
}

output old_pin "$OLD_PIN"
git_tap checkout --quiet --detach "$OLD_PIN"
rc=0
PIN_REPORT=$(bottle_report) || rc=$?
[[ $rc -eq 0 || $rc -eq 3 ]] || die "bottle check failed (exit ${rc}) at the pin ${OLD_PIN}"
EXEMPT=$(printf '%s\n' "$PIN_REPORT" | awk '$1 == "no" { print $3 }')
[[ -z $EXEMPT ]] || log "not bottled at the pin either, not required: $(tr '\n' ' ' <<< "$EXEMPT")"

# Every formula that poured at the pin still has a bottle for this host at
# the checked-out tap commit, and every bottle tarball is still served.
bottled_here() {
	local rc=0 report name url
	report=$(bottle_report) || rc=$?
	printf '%s\n' "$report" | sed 's/^/    /'
	[[ $rc -eq 0 || $rc -eq 3 ]] || return "$rc"
	for name in $(printf '%s\n' "$report" | awk '$1 == "no" { print $3 }'); do
		grep -qx "$name" <<< "$EXEMPT" || return 3
	done
	for url in $(printf '%s\n' "$report" | awk '$1 == "bottled" { print $4 }'); do
		curl -sfI -o /dev/null "$url" || { log "bottle tarball missing: ${url}"; return 3; }
	done
}

CANDIDATES=$(git_tap rev-list --first-parent "${OLD_PIN}..origin/HEAD")
# shellcheck disable=SC2086 # word-split on purpose
set -- $CANDIDATES
log "${TAP} HEAD ${TAP_HEAD} is $# commit(s) ahead of the pin, checking newest first for ${ROOTS}"

NEW_PIN=""
for sha in "$@"; do
	log "$(git_tap log -1 --format='%h %cs %s' "$sha")"
	git_tap checkout --quiet --detach "$sha"
	rc=0
	bottled_here || rc=$?
	case $rc in
		0) NEW_PIN=$sha; break ;;
		3) ;;
		*) die "bottle check failed (exit ${rc}) at ${sha}" ;;
	esac
done

if [[ -z $NEW_PIN ]]; then
	log "no commit newer than the pin is fully bottled here, keeping ${OLD_PIN}"
	output needs_bump false
	exit 0
fi

awk -v old="$OLD_PIN" -v new="$NEW_PIN" '$0 == old { print new; next } { print }' "$PIN_FILE" > "${PIN_FILE}.tmp"
mv "${PIN_FILE}.tmp" "$PIN_FILE"
log "pinned ${TAP} to ${NEW_PIN}, $(git_tap rev-list --count "${NEW_PIN}..origin/HEAD") commit(s) behind HEAD"
output needs_bump true
output new_pin "$NEW_PIN"
output new_pin_short "${NEW_PIN:0:12}"
