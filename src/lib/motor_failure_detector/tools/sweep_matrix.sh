#!/usr/bin/env bash
# Sweep one trip band against its trigger time and print the result as a matrix: each cell is how
# many logs false-trip at that (trigger time, band) pair, so the first 0 in a row is the lowest band
# that trigger time supports. The other direction is disabled, so each side is measured on its own.
#
# Usage:  C2T=<slope> IDLE=<offset> sweep_matrix.sh <log-dir> under|over
#   env TRIGGER_TIMES="0.5 0.7 1.0"  trigger times, one per row
#   env BANDS="1.5 1.75 2.0"         bands, one per column -- scale these to the vehicle, the
#                                    defaults below suit a multirotor drawing a few amps per
#                                    motor in hover
#   env MFD_REPLAY=...               binary location, as for batch_replay.sh
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
	read -ra TRIGGER_TIMES <<<"${TRIGGER_TIMES:-0.1 0.3 0.5 0.7 1.0}"
	read -ra BANDS <<<"${BANDS:-1.0 1.25 1.5 1.75 2.0 2.5 3.0}"
	OTHER_OFF=(--set MOTFAIL_OVER=0); BAND_P=MOTFAIL_UNDER; TRIGGER_TIME_P=MOTFAIL_UND_TIME ;;
over)
	read -ra TRIGGER_TIMES <<<"${TRIGGER_TIMES:-0.4 0.7 1.0 1.5 2.5}"
	read -ra BANDS <<<"${BANDS:-0.5 1.0 2.0 4.0 6.0 8.0}"
	OTHER_OFF=(--set MOTFAIL_UNDER=0); BAND_P=MOTFAIL_OVER; TRIGGER_TIME_P=MOTFAIL_OVR_TIME ;;
*)
	echo "error: side must be 'under' or 'over', got '$SIDE'" >&2; exit 2 ;;
esac

echo "# I_exp = ${C2T}*u + ${IDLE} A, sweeping $BAND_P over $DIR"
printf "%-11s" "trigger[s]"; printf "%7s" "${BANDS[@]}"; echo "   <- $BAND_P [A]  (logs tripped)"

for trigger_time in "${TRIGGER_TIMES[@]}"; do
	printf "%-11s" "$trigger_time"

	for band in "${BANDS[@]}"; do
		out=$("$BATCH" "$DIR" --set MOTFAIL_C2T="$C2T" --set MOTFAIL_IDLE="$IDLE" \
			"${OTHER_OFF[@]}" --set "$BAND_P=$band" --set "$TRIGGER_TIME_P=$trigger_time")

		# Without a single evaluated log (no summary, empty directory, or every log skipped) the
		# cell would read as "no trips".
		ok=$(sed -n 's/.* OK=\([0-9]*\).*/\1/p' <<<"$out")
		fail=$(sed -n 's/.* FAIL=\([0-9]*\).*/\1/p' <<<"$out")
		(( ${ok:-0} + ${fail:-0} > 0 )) || { echo " (no log evaluated -- check the log directory)"; exit 9; }

		printf "%7s" "$fail"
	done

	echo
done
