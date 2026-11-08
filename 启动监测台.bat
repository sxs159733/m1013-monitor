@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\pythonw.exe" (
    echo [setup] Creating virtual environment and installing dependencies...
    python -m venv .venv
    ".venv\Scripts\python.exe" -m pip install -r requirements.txt
    if errorlevel 1 (
        echo [error] Dependency installation failed. Check network or Python install.
        pause
        exit /b 1
    )
)
start "" ".venv\Scripts\pythonw.exe" "app.py"
endlocal
