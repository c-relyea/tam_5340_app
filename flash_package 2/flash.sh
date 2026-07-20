#!/usr/bin/env bash
#
# Flash the TAM board (nRF5340 dual-core) from prebuilt hex files.
# macOS / Linux.  Uses SEGGER JLinkExe -- the same path as
# "west flash --runner jlink", which auto-handles the nRF5340 unlock.
#
# Run:  ./flash.sh
#
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
APP_HEX="$DIR/tam_app.hex"      # application core (mcuboot + tfm + app)
NET_HEX="$DIR/tam_net.hex"      # network core (radio)
SPEED=4000

echo "==================================================================="
echo " TAM board flasher (JLink)"
echo " * Keep the battery (or ~3.8V bench supply) connected."
echo " * Quit the nRF Connect for Desktop app FIRST -- only one tool"
echo "   can use the J-Link probe at a time."
echo "==================================================================="

# --- locate JLinkExe -----------------------------------------------------
JLINK="$(command -v JLinkExe || true)"
[ -z "$JLINK" ] && [ -x /Applications/SEGGER/JLink/JLinkExe ] && JLINK=/Applications/SEGGER/JLink/JLinkExe
if [ -z "$JLINK" ]; then
  echo "ERROR: JLinkExe not found. Install the SEGGER J-Link Software pack:"
  echo "  https://www.segger.com/downloads/jlink/  (macOS: brew install --cask segger-jlink)"
  exit 1
fi
[ -f "$APP_HEX" ] || { echo "ERROR: missing $APP_HEX"; exit 1; }
[ -f "$NET_HEX" ] || { echo "ERROR: missing $NET_HEX"; exit 1; }

# --- helper: flash one core ---------------------------------------------
# $1 = jlink device name, $2 = hex file, $3 = label
flash_core() {
  local device="$1" hex="$2" label="$3"
  local script; script="$(mktemp)"
  # Each core is its own JLink session. loadfile programs (erasing the
  # affected sectors); r + g reset and run. JLinkExe auto-recovers a
  # locked nRF5340 on connect.
  cat > "$script" <<EOF
si SWD
speed $SPEED
device $device
connect
r
h
loadfile "$hex"
r
g
exit
EOF
  echo
  echo ">>> Flashing $label core ($device)..."
  "$JLINK" -nogui 1 -if swd -speed "$SPEED" -device "$device" -CommanderScript "$script"
  local rc=$?
  rm -f "$script"
  return $rc
}

# NETWORK core first, then APPLICATION core.
flash_core nrf5340_xxaa_net "$NET_HEX" "NETWORK"
flash_core nrf5340_xxaa_app "$APP_HEX" "APPLICATION"

echo
echo "DONE. Board flashed and running."
