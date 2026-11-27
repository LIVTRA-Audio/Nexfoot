@echo off
cd /d "%~dp0"
py -3 tools\nexfoot.py build %*
exit /b %ERRORLEVEL%
