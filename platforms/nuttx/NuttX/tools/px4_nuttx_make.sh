#!/bin/sh
# Run make for a NuttX library or the NuttX apps with parallel jobs.
#
# If the build was started with an explicit job count, e.g.
# `make px4_fmu-v5x_default -j1`, MAKEFLAGS carries it (and with GNU make 4.4
# its jobserver) through ninja to here, so honour it. Otherwise use the number
# of CPUs this process may run on, which, unlike the host's core count,
# respects limits like docker --cpuset-cpus. The load limit keeps the NuttX
# makes that ninja runs concurrently from oversubscribing the host.

case " ${MAKEFLAGS} " in
	*" -j"*) exec make "$@" ;;
esac

jobs=$(nproc 2>/dev/null || getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)
exec make -j"${jobs}" -l"${jobs}" "$@"
