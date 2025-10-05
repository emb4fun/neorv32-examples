@echo off
cls
mkdir .\build 

echo Please wait...
copy .\output_files\*.sof .\build >NUL
%ALTERA_PROGRAM_DIR_V20%\quartus\bin64\quartus_cpf -c .\cfg\de0cv.cof

echo.
pause

