TAM board flashing package (JLink)
==================================

Contents:
  tam_app.hex   - application core image (mcuboot + TF-M + app)
  tam_net.hex   - network core image (radio)
  flash.sh      - flasher for macOS / Linux
  flash.bat     - flasher for Windows
  README.txt    - this file

This board is a dual-core nRF5340. Both cores must be flashed. The scripts
do it in the correct order (network core first, then application core) using
SEGGER JLinkExe -- the same tool "west flash --runner jlink" uses, which
transparently unlocks a locked nRF5340 on connect.


ONE-TIME SETUP
--------------
Install the SEGGER J-Link Software and Documentation Pack:
  https://www.segger.com/downloads/jlink/
  (macOS: "brew install --cask segger-jlink" also works)

Verify:  run  "JLinkExe -CommanderScript /dev/null"  (or just "JLinkExe")
         -- it should start and print a version.

That's the only dependency. You do NOT need the SDK, VS Code, west, the
nRF Command Line Tools, or the nRF Connect Programmer app.


HARDWARE
--------
  - Connect the J-Link to the board's 6-pin Tag-Connect (TC2030) debug port.
  - KEEP THE BATTERY CONNECTED (or a ~3.8V bench supply on the battery
    pads) during flashing. Without a battery the power rail is unstable and
    flashing fails with "low voltage" / "could not power up DAP" errors.
  - Plug in USB-C for power.


FLASH
-----
macOS / Linux:   open a terminal in this folder and run:   ./flash.sh
                 (first time only:  chmod +x flash.sh)

Windows:         double-click flash.bat  (or run it from a Command Prompt)


IMPORTANT: ONE TOOL ON THE PROBE AT A TIME
------------------------------------------
Only one program can use the J-Link at once. Before flashing, fully QUIT
(not just close the window) the nRF Connect for Desktop app, any RTT viewer,
and any IDE that might be talking to the probe. Two tools fighting over the
probe causes "access port is not available" / "could not power up DAP".


TROUBLESHOOTING
---------------
"could not power up DAP" /
"CPU could not be halted"    -> Probe or board is in a stuck state. Fix:
                                1) unplug the J-Link USB, wait 5 s, replug
                                2) power-cycle the board (battery, then USB-C)
                                3) quit all other J-Link/nRF Connect apps
                                then re-run the script.

"low voltage detected"       -> battery/bench supply not connected or unstable.
                                Connect a stable ~3.8V source and retry.

Nothing happens / no probe   -> check the Tag-Connect is seated squarely and
                                firmly (all 6 spring pins in contact).


NOTE
----
Do NOT flash this board with the nRF Connect Programmer desktop app. It has
caused lock-ups and probe-contention issues. Use these scripts (JLinkExe).
