@echo off
rem Launcher for the MSFS to MOZA force feedback bridge.
rem
rem Uses a packaged ffbbridge.exe if one is sitting next to this script, and
rem otherwise runs from source, creating the Python environment on first use.
rem Arguments pass straight through: run.bat doctor, run.bat bench sweep.

setlocal
cd /d "%~dp0"

rem A packaged build wins: it needs nothing installed.
set "EXE="
if exist "ffbbridge.exe" set "EXE=ffbbridge.exe"
if exist "dist\ffbbridge.exe" set "EXE=dist\ffbbridge.exe"
if defined EXE (
    "%EXE%" %*
    goto :done
)

set "PY=.venv\Scripts\python.exe"
if not exist "%PY%" (
    echo Setting up a Python environment. This happens once and takes a minute.
    echo.
    where py >nul 2>nul
    if errorlevel 1 (
        python -m venv .venv
    ) else (
        py -3 -m venv .venv
    )
    if not exist "%PY%" (
        echo.
        echo Could not create the environment. Install Python 3.11 or newer from
        echo https://www.python.org/downloads/ and run this again. Tick "Add
        echo python.exe to PATH" in the installer.
        goto :done
    )
    "%PY%" -m pip install --upgrade pip
    "%PY%" -m pip install -e ".[gui]"
    if errorlevel 1 (
        echo.
        echo The install failed. The output above should say why.
        goto :done
    )
    echo.
)

"%PY%" -m ffbbridge.app.main %*

:done
rem Double-clicked rather than run from a terminal: hold the window open so any
rem message is readable.
if "%~1"=="" pause
endlocal
