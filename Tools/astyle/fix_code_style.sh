#!/usr/bin/env bash
# Format the given files in place with astyle 3.1 and the PX4 rules in astylerc.
#
# Prefers astyle-py (pip install astyle-py, see Tools/setup/requirements.txt)
# which bundles astyle 3.1 independent of the OS. Falls back to a natively
# installed astyle of a supported version.
#
# Without file arguments it only checks that a usable astyle is available.

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
ASTYLE_PY_VERSION="3.1"
ASTYLE_NATIVE_VERSIONS=("2.06" "3.0" "3.0.1" "3.1")

if command -v astyle_py >/dev/null 2>&1; then
	ASTYLE=(astyle_py)
elif python3 -c "import astyle_py" >/dev/null 2>&1; then
	ASTYLE=(python3 -m astyle_py)
fi

if [[ -n "${ASTYLE[*]}" ]]; then
	ASTYLE+=("--astyle-version=${ASTYLE_PY_VERSION}" --options="${DIR}/astylerc" --quiet)
else
	NATIVE_VERSION=$(astyle --version 2>/dev/null | sed -n 's/^Artistic Style Version //p')
	for v in "${ASTYLE_NATIVE_VERSIONS[@]}"; do
		if [[ "$NATIVE_VERSION" == "$v" ]]; then
			ASTYLE=(astyle --options="${DIR}/astylerc" --suffix=none --lineend=linux --preserve-date --quiet)
		fi
	done
fi

if [[ -z "${ASTYLE[*]}" ]]; then
	echo "Error: no usable astyle found${NATIVE_VERSION:+ (installed astyle ${NATIVE_VERSION} is not supported)}." >&2
	echo "Install astyle 3.1 via: python3 -m pip install -r ${DIR}/../setup/requirements.txt" >&2
	exit 1
fi

if [[ $# -eq 0 ]]; then
	exit 0
fi

"${ASTYLE[@]}" "$@" >/dev/null
