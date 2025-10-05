@echo off
cls
mkdir .\build 
 
echo Please wait...
copy .\output_files\*.sof .\build >NUL
%ALTERA_PROGRAM_DIR_V20%\quartus\bin64\quartus_cpf -c .\cfg\c5g.cof
copy .\cfg\c5g-neorv32-blinky.cdf .\build >NUL

pause

