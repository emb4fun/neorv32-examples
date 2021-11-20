@echo off
cls
mkdir .\build 
del .\build\*.jic >NUL 2>&1
del .\build\*.map >NUL 2>&1
del .\build\*.sof >NUL 2>&1

echo Please wait...
copy .\output_files\*.sof .\build >NUL
%ALTERA_PROGRAM_DIR_V15%\quartus\bin64\quartus_cpf -c .\cfg\de0n.cof

echo.
pause

