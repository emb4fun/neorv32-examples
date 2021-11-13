@echo off
cls

del *.bak /S
del *.qws
del PLLJ_PLLSPE_INFO.txt
del c5_pin_model_dump.txt
del pll_sys.xml
del .\build\*.jic
del .\build\*.map
del .\build\*.sof
rmdir "db" /S /Q
rmdir "incremental_db" /S /Q 
rmdir "output_files" /S /Q
rmdir "simulation" /S /Q
del *.qws
del qsys_core.html
