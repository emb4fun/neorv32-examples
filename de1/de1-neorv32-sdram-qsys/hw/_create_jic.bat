@echo off
cls
 
del .\build\*.jic 2>NUL
del .\build\*.map 2>NUL
del .\build\*.sof 2>NUL

echo Please wait...
copy .\output_files\*.sof .\build >NUL
%ALTERA_PROGRAM_DIR_V11%\quartus\bin\quartus_cpf -c .\cfg\de1.cof

pause

