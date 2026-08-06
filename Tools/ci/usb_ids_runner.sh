#!/usr/bin/env bash
# This script is meant to be used by the usb_ids.yml workflow in a github runner
# It finds the board defconfigs changed by the triggering PR or push and checks
# them against the Dronecode USB ID registry (https://github.com/Dronecode/usb-ids)
# using Tools/check_usb_ids.py
set -euo pipefail

# Provided by the workflow / runner environment:
#   GITHUB_EVENT_NAME, GITHUB_REPOSITORY, GITHUB_SHA (default runner env)
#   GH_TOKEN     - token for the gh api calls
#   PR_NUMBER    - pull request number (pull_request events only)
#   BEFORE_SHA   - github.event.before (push events only)

changed=$(mktemp)

if [ "${GITHUB_EVENT_NAME}" = "pull_request" ]; then
    gh api "repos/${GITHUB_REPOSITORY}/pulls/${PR_NUMBER}/files" --paginate \
        --jq '.[] | select(.status != "removed") | .filename' > "${changed}"
elif [ "${BEFORE_SHA}" = "0000000000000000000000000000000000000000" ]; then
    # push to a new branch: no before-sha to diff against, check the head commit
    git show --pretty=format: --name-only "${GITHUB_SHA}" > "${changed}"
else
    gh api "repos/${GITHUB_REPOSITORY}/compare/${BEFORE_SHA}...${GITHUB_SHA}" \
        --jq '.files[] | select(.status != "removed") | .filename' > "${changed}"
fi

defconfigs=$(grep -E '^boards/[^/]+/[^/]+/nuttx-config/.+/defconfig$' "${changed}" || true)

if [ -z "${defconfigs}" ]; then
    echo "No board defconfig changes, nothing to check."
    exit 0
fi

echo "Changed defconfigs:"
echo "${defconfigs}"

pip install --user --quiet pyyaml
echo "${defconfigs}" | xargs python3 Tools/check_usb_ids.py check
