#!/usr/bin/env bash
#
# Capture raw PPG samples (MAX30101 IR/red) from the board over RTT
# into a CSV file.  Requires firmware built with PPG_RAW_LOG=1 and a
# connected J-Link.  Quit other J-Link/nRF Connect tools first.
#
# Usage:  ./log_ppg.sh [seconds] [out.csv]
#         ./log_ppg.sh 60 ppg_run1.csv
#
set -euo pipefail

DUR="${1:-30}"
OUT="${2:-ppg_$(date +%Y%m%d_%H%M%S).csv}"
PORT=19021

JLINK="$(command -v JLinkExe || true)"
[ -z "$JLINK" ] && [ -x /Applications/SEGGER/JLink/JLinkExe ] && JLINK=/Applications/SEGGER/JLink/JLinkExe
[ -z "$JLINK" ] && { echo "ERROR: JLinkExe not found"; exit 1; }

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

cat > "$TMP/attach.jlink" <<EOF
connect
Sleep $(( (DUR + 4) * 1000 ))
exit
EOF

echo "Attaching RTT for ${DUR}s -> $OUT"
"$JLINK" -nogui 1 -AutoConnect 1 -Device NRF5340_XXAA_APP -If SWD -Speed 4000 \
         -RTTTelnetPort $PORT -CommanderScript "$TMP/attach.jlink" > "$TMP/jlink_out.txt" 2>&1 &
JPID=$!

# Wait for the RTT telnet server to come up (up to 10 s), then give
# JLink time to locate the RTT control block before the real client
# connects -- connecting too early yields a banner and no data.
for _ in $(seq 1 20); do
  if nc -z localhost $PORT 2>/dev/null; then break; fi
  sleep 0.5
done
sleep 2

nc -w "$DUR" localhost $PORT > "$TMP/rtt.txt" 2>/dev/null || true
kill "$JPID" 2>/dev/null || true
wait "$JPID" 2>/dev/null || true

{ echo "uptime_ms,ir,red,green"
  awk -F, '/^P,/ { print $2 "," $3 "," $4 "," $5 }' "$TMP/rtt.txt"; } > "$OUT"

# Device-computed HR/SpO2 (H,<ms>,<bpm_x10>,<qual> / O,<ms>,<spo2_x10>)
EVT="${OUT%.csv}.events.csv"
{ echo "type,uptime_ms,value_x10,quality"
  awk -F, '/^[HO],/ { print $1 "," $2 "," $3 "," (NF>3 ? $4 : "") }' "$TMP/rtt.txt"; } > "$EVT"

# Everything that isn't a P,/H,/O, CSV line -- BLE connect/disconnect,
# GATT notify errors, MTU exchange, boot banners, etc.  This is where
# the disconnect reason code shows up.
LOG="${OUT%.csv}.log"
grep -vE '^[PHO],' "$TMP/rtt.txt" > "$LOG" || true

N=$(( $(wc -l < "$OUT") - 1 ))
NE=$(( $(wc -l < "$EVT") - 1 ))
NL=$(wc -l < "$LOG")
echo "Wrote $N samples to $OUT (expected ~$(( DUR * 25 )) at 25 Hz)"
echo "Wrote $NE HR/SpO2 events to $EVT"
echo "Wrote $NL non-sample log lines to $LOG"

if [ "$N" -eq 0 ]; then
  echo "--- No samples. J-Link session output: ---"
  grep -iE "VTref|error|halt|RTT|Cortex|cannot|fail" "$TMP/jlink_out.txt" | head -10
  echo "--- Raw RTT bytes received: $(wc -c < "$TMP/rtt.txt") ---"
  head -c 200 "$TMP/rtt.txt"
  exit 1
fi
