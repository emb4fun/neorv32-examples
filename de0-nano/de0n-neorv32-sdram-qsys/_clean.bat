@echo off

del *.bak /S
cd hw
call _clean.bat
cd ..
cd sw
call _clean.bat
cd ..
