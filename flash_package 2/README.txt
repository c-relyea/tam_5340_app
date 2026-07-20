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

FLASH
-----
macOS / Linux:   open a terminal in this folder and run:   ./flash.sh
                 (first time only:  chmod +x flash.sh)

Windows:         double-click flash.bat  (or run it from a Command Prompt)



