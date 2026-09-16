@echo off
REM =====================================================================
REM  run_everything.bat  -  ONE-CLICK FULL DEMO
REM
REM  Starts everything the jury needs to see:
REM    1. The original Pygame simulation window (primary demo, keys 1-9,0)
REM    2. MATLAB desktop, which then (via run_everything.m):
REM         - smoke-tests the Python bridge (existing A* planner)
REM         - runs the 5-scenario MATLAB demo        -> figures + metrics
REM         - runs the Simulink end-to-end validation -> figure + metrics
REM         - opens adaptive_path_planning.slx in the Simulink tab and
REM           starts a live 20 s run (Display blocks / Scope update)
REM
REM  See README_MATLAB_INTEGRATION.md for details.
REM =====================================================================

setlocal
set "PROJ=U:\SIH-16-9-26-2-"
set "PYEXE=C:\Users\itzke\AppData\Local\Programs\Python\Python312\python.exe"
set "MATEXE=C:\Program Files\MATLAB\R2026a\bin\matlab.exe"

REM --- Environment fix: this variable breaks Simulink's codegen batch call
REM --- ("..._cgxe.bat is not recognized"). Clearing it is required when the
REM --- shell that launches MATLAB was started from a sandboxed environment.
set "NoDefaultCurrentDirectoryInExePath="

cd /d "%PROJ%"

echo.
echo [1/2] Starting PYGAME simulation  (main2_collision_free.py) ...
start "PYGAME_DEMO" "%PYEXE%" "%PROJ%\main2_collision_free.py"

echo [2/2] Starting MATLAB  (run_everything) - desktop takes ~30-60 s ...
start "MATLAB_DEMO" "%MATEXE%" -sd "%PROJ%" -r "run_everything"

echo.
echo  Both processes launched. Expect:
echo   - Pygame window      : live Indian-road simulation (keys 1-9,0)
echo   - MATLAB desktop     : Command Window log + validation figures
echo   - Simulink tab       : adaptive_path_planning model, live run
echo.
endlocal
