@echo off

call "setup_mingw.bat"

cd .

chcp 1252

if "%1"=="" ("F:\MATLAB\R2021b\bin\win64\gmake"  -f cubic_trajectory_generator_pos_vel_rtw.mk all) else ("F:\MATLAB\R2021b\bin\win64\gmake"  -f cubic_trajectory_generator_pos_vel_rtw.mk %1)
@if errorlevel 1 goto error_exit

exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
exit /B 1