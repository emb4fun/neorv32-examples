@echo off
cls
 
echo Please wait...
copy .\output_files\*.sof .\build >NUL
%ALTERA_PROGRAM_DIR_V15%\quartus\bin64\quartus_cpf -c .\cfg\c5g.cof
copy .\cfg\c5g-neorv32-blinky.cdf .\build >NUL

pause

