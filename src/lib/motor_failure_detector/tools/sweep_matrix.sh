#!/usr/bin/env bash
# Sweep one trip band against its hold time and print the result as a matrix: each cell is how many
# logs false-trip at that (hold, band) pair, so the first 0 in a row is the lowest band that hold
# time supports. The other direction is parked at a value nothing reaches, so each side is measured
# on its own.
#
# Usage:  C2T=<slope> IDLE=<offset> sweep_matrix.sh <log-dir> under|over
#   env HOLDS="0.5 0.7 1.0"   hold times, one per row
#   env BANDS="1.5 1.75 2.0"  bands, one per column -- scale these to the vehicle, the defaults
#                             below suit a multirotor drawing a few amps per motor in hover
#   env MFD_REPLAY=...        binary location, as for batch_replay.sh
#
# Run from the repository root. Fit the model first (mfd_fit); the bands are deviations from it.
set -uo pipefail

DIR="${1:?usage: C2T=.. IDLE=.. sweep_matrix.sh <log-dir> under|over}"
SIDE="${2:-}"
: "${C2T:?set C2T to the fitted MOTFAIL_C2T}"
: "${IDLE:?set IDLE to the fitted MOTFAIL_IDLE}"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BATCH="$HERE/batch_replay.sh"

case "$SIDE" in
under)
	read -ra HOLDS <<<"${HOLDS:-0.1 0.3 0.5 0.7 1.0}"
	read -ra BANDS <<<"${BANDS:-1.0 1.25 1.5 1.75 2.0 2.5 3.0}"
	PARK=(--set MOTFAIL_OVER=999); BAND_P=MOTFAIL_UNDER; HOLD_P=MOTFAIL_UND_TIME ;;
over)
	read -ra HOLDS <<<"${HOLDS:-0.4 0.7 1.0 1.5 2.5}"
	read -ra BANDS <<<"${BANDS:-0.5 1.0 2.0 4.0 6.0 8.0}"
	PARK=(--set MOTFAIL_UNDER=999); BAND_P=MOTFAIL_OVER; HOLD_P=MOTFAIL_OVR_TIME ;;
*)
	echo "error: side must be 'under' or 'over', got '$SIDE'" >&2; exit 2 ;;
esac

echo "# I_exp = ${C2T}*u + ${IDLE} A, sweeping $BAND_P over $DIR"
printf "%-9s" "hold[s]"; printf "%7s" "${BANDS[@]}"; echo "   <- $BAND_P [A]  (logs tripped)"

for hold in "${HOLDS[@]}"; do
	printf "%-9s" "$hold"

	for band in "${BANDS[@]}"; do
		out=$("$BATCH" "$DIR" --set MOTFAIL_C2T="$C2T" --set MOTFAIL_IDLE="$IDLE" \
			"${PARK[@]}" --set "$BAND_P=$band" --set "$HOLD_P=$hold")

		# A missing summary means nothing ran; without this a broken invocation reads as "no trips".
		grep -q "^total=" <<<"$out" || { echo " (no verdict summary -- nothing ran)"; exit 9; }

		printf "%7s" "$(sed -n 's/.*FAIL=\([0-9]*\).*/\1/p' <<<"$out")"
	done

	echo
done
