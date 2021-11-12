@echo off
cls
 
echo Please wait...
copy .\output_files\*.sof .\build >NUL
%ALTERA_PROGRAM_DIR_V15%\quartus\bin64\quartus_cpf -c .\cfg\de0n.cof

pause

