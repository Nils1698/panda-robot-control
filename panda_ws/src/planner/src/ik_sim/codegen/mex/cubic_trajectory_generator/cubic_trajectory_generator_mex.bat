@echo off
call setEnv.bat
"F:\MATLAB\R2021b\toolbox\shared\coder\ninja\win64\ninja.exe" -t compdb cc cxx cudac > compile_commands.json
"F:\MATLAB\R2021b\toolbox\shared\coder\ninja\win64\ninja.exe" -v %*
