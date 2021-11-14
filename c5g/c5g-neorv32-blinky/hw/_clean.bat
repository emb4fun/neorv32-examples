@echo off
cls

del *.bak /S
del *.qws
del c5_pin_model_dump.txt
del .\build\*.jic
del .\build\*.map
del .\build\*.sof
del .\build\*.cdf
rmdir "db" /S /Q
rmdir "incremental_db" /S /Q 
rmdir "output_files" /S /Q
del PLLJ_PLLSPE_INFO.txt
