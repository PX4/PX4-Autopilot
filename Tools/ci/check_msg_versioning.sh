#! /bin/bash
# Copy a git diff between two commits if msg versioning is added

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

PX4_SRC_DIR="$DIR/.."

BASE_COMMIT="$1"
HEAD_COMMIT="$2"
if [ -z "${BASE_COMMIT}" ] || [ -z "${HEAD_COMMIT}" ]
then
  echo "Usage: $0 <base_commit> <head_commit>"
  exit 1
fi

failed=0

# Iterate git diff files between BASE_COMMIT and HEAD_COMMIT
modified_files="$(git --no-pager diff --no-color --name-only --diff-filter=M "${BASE_COMMIT}"..."${HEAD_COMMIT}")"
all_files="$(     git --no-pager diff --no-color --name-only                 "${BASE_COMMIT}"..."${HEAD_COMMIT}")"
for file in ${modified_files}
do
  if [[ "$file" == msg/versioned/*.msg ]]; then
    echo "Modified msg file: ${file}"
    # A modified versioned .msg file requires:
    # - Incrementing the version
    # - An old .msg version exists
    # - A translation header exists and is included

    # Ignore changes to comments or constants
    content_a=$(git show "${BASE_COMMIT}:${file}" | grep -o '^[^#]*' | grep -v =)
    content_b=$(git show "${HEAD_COMMIT}:${file}" | grep -o '^[^#]*' | grep -v =)
    if [ "${content_a}" == "${content_b}" ]; then
      echo "No version update required for ${file}"
      continue
    fi

    diff=$(git --no-pager diff --no-color --diff-filter=M "${BASE_COMMIT}"..."${HEAD_COMMIT}" -- "${file}")
    old_version=$(echo "${diff}" | sed -n 's/^-uint32 MESSAGE_VERSION = \([0-9]*\).*/\1/p')
    new_version=$(echo "${diff}" | sed -n 's/^+uint32 MESSAGE_VERSION = \([0-9]*\).*/\1/p')

    # Check that the version is incremented
    if [ -z "${new_version}" ] || [ -z "${old_version}" ]; then
      echo "ERROR: Missing version update for ${file}"
      failed=1
    else
      if [ $((new_version-old_version)) -ne 1 ]; then
        echo "ERROR: Version not incremented by +1 for ${file}"
        failed=1
      fi
    fi

    # Check that an old version exists
    filename=$(basename -- "$file")
    filename="${filename%.*}"
    old_version_file="px4_msgs_old/msg/${filename}V${old_version}.msg"
    if [[ "${all_files}" != *"${old_version_file}"* ]]; then
      echo "ERROR: Old message version does not exist for ${file} (missing ${old_version_file})"
      failed=1
    fi

    # Check that a translation header got added by checking for a modification to all_translations.h
    # If it is changed, we assume a new header got added too, so we don't explicitly check for that
    if [[ "${modified_files}" != *"translations/all_translations.h"* ]]; then
      echo "ERROR: missing modification to translations/all_translations.h"
      failed=1
    fi
  fi
done

if [ ${failed} -ne 0 ]; then
  echo ""
  echo "ERROR: missing message versioning due to changed .msg file(s) (see above)"
  echo "Check the documentation under https://docs.px4.io/main/en/ros2/px4_ros2_msg_translation_node.html for how to add a new version"
  exit 1
fi
