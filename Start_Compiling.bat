PATH = %PATH%;C:\Program Files\STMicroelectronics\st_toolset\stvp;C:\Program Files (x86)\STMicroelectronics\st_toolset\stvp;C:\SDCC\bin
REM ;%~dp0tools\cygwin\bin
cd %~dp0
del main.hex
sdcc --version
make -f Makefile_windows clean
make -f Makefile_windows
ren main.ihx main.hex


make -f Makefile_windows clean

STVP_CmdLine -BoardName=ST-LINK -ProgMode=SWIM -Port=USB -Device=STM8S105x6 -FileProg=main.hex -verbose -no_loop

pause
exit
