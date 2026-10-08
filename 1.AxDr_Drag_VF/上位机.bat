@echo off
rem ===========================================================================
rem  AxDr FOC host GUI launcher
rem
rem  USAGE:  double-click this file,  or:  上位机.bat COM5
rem
rem  IMPORTANT: this file is intentionally ASCII-only.
rem    cmd.exe reads .bat files using the system ANSI codepage (GBK on a
rem    Chinese Windows), NOT UTF-8. If you save Chinese text here as UTF-8
rem    it gets decoded as garbage and the script can fail to parse.
rem    Keep it ASCII; put Chinese only inside the Python program.
rem ===========================================================================
setlocal

set "PY="

rem 1) prefer the Python bundled with DSH (pyserial already installed)
if exist "C:\Users\yanju\.dsh\dsh-runtimes\dsh-primary-runtime\dependencies\python\python.exe" (
    set "PY=C:\Users\yanju\.dsh\dsh-runtimes\dsh-primary-runtime\dependencies\python\python.exe"
)

rem 2) fall back to python on PATH
if not defined PY (
    where python >nul 2>nul
    if not errorlevel 1 set "PY=python"
)

rem 3) last resort: the py launcher
if not defined PY (
    where py >nul 2>nul
    if not errorlevel 1 set "PY=py"
)

if not defined PY (
    echo.
    echo [ERROR] Python not found.
    echo.
    echo Run it manually instead:
    echo   C:\Users\yanju\.dsh\dsh-runtimes\dsh-primary-runtime\dependencies\python\python.exe tools\host_gui.py
    echo.
    pause
    exit /b 1
)

if not exist "%~dp0tools\host_gui.py" (
    echo.
    echo [ERROR] tools\host_gui.py not found next to this .bat
    echo   expected: %~dp0tools\host_gui.py
    echo.
    pause
    exit /b 1
)

echo Using Python: %PY%
echo Starting GUI ...
echo.

"%PY%" "%~dp0tools\host_gui.py" %*

set "RC=%ERRORLEVEL%"
if not "%RC%"=="0" (
    echo.
    echo [EXIT %RC%] something went wrong - send the messages above.
    pause
)

endlocal
