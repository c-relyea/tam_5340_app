@echo off
REM Flash the TAM board (nRF5340 dual-core) from prebuilt hex files. Windows.
REM Uses SEGGER JLink.exe -- same path as "west flash --runner jlink".
REM Double-click, or run from a Command Prompt in this folder: flash.bat

setlocal
set DIR=%~dp0
set APP_HEX=%DIR%tam_app.hex
set NET_HEX=%DIR%tam_net.hex
set SPEED=4000

echo ===================================================================
echo  TAM board flasher (JLink)
echo  * Keep the battery (or ~3.8V bench supply) connected.
echo  * Quit the nRF Connect for Desktop app FIRST -- only one tool
echo    can use the J-Link probe at a time.
echo ===================================================================

REM Locate JLink.exe (added to PATH by the J-Link installer)
set JLINK=JLink.exe
where %JLINK% >nul 2>nul
if errorlevel 1 (
  echo ERROR: JLink.exe not found. Install the SEGGER J-Link Software pack:
  echo   https://www.segger.com/downloads/jlink/
  exit /b 1
)

REM --- NETWORK core ---
echo.
echo ^>^>^> Flashing NETWORK core...
(
  echo si SWD
  echo speed %SPEED%
  echo device nrf5340_xxaa_net
  echo connect
  echo r
  echo h
  echo loadfile "%NET_HEX%"
  echo r
  echo g
  echo exit
) > "%TEMP%\tam_net.jlink"
%JLINK% -nogui 1 -if swd -speed %SPEED% -device nrf5340_xxaa_net -CommanderScript "%TEMP%\tam_net.jlink"
if errorlevel 1 goto :err

REM --- APPLICATION core ---
echo.
echo ^>^>^> Flashing APPLICATION core...
(
  echo si SWD
  echo speed %SPEED%
  echo device nrf5340_xxaa_app
  echo connect
  echo r
  echo h
  echo loadfile "%APP_HEX%"
  echo r
  echo g
  echo exit
) > "%TEMP%\tam_app.jlink"
%JLINK% -nogui 1 -if swd -speed %SPEED% -device nrf5340_xxaa_app -CommanderScript "%TEMP%\tam_app.jlink"
if errorlevel 1 goto :err

echo.
echo DONE. Board flashed and running.
exit /b 0

:err
echo.
echo FLASH FAILED. See README.txt (power / probe contention / wedged probe).
exit /b 1
