@echo off
%1 mshta vbscript:CreateObject("Shell.Application").ShellExecute("cmd.exe","/c %~s0 ::","","runas",1)(window.close)&&exit
cd /d "%~dp0"

if defined PYTHON (
    %PYTHON% .\scripts\create_soft_link.py
) else (
    python .\scripts\create_soft_link.py
)
exit /b
