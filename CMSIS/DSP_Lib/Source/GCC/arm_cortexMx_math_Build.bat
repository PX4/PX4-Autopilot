
SET TMP=C:\Temp
SET TEMP=C:\Temp

SET UVEXE=C:\Keil\UV4\UV4.EXE

@echo   Building DSP Library for Cortex-M0 Little Endian
%UVEXE% -rb arm_cortexM0x_math.uvproj -t"DSP_Lib CM0 LE" -o"DSP_Lib CM0 LE.txt" -j0

@echo   Building DSP Library for Cortex-M3 Little Endian
%UVEXE% -rb arm_cortexM3x_math.uvproj -t"DSP_Lib CM3 LE" -o"DSP_Lib CM3 LE.txt" -j0

@echo   Building DSP Library for Cortex-M4 Little Endian
%UVEXE% -rb arm_cortexM4x_math.uvproj -t"DSP_Lib CM4 LE" -o"DSP_Lib CM4 LE.txt" -j0

@echo   Building DSP Library for Cortex-M4 with FPU Little Endian
%UVEXE% -rb arm_cortexM4x_math.uvproj -t"DSP_Lib CM4 LE FPU" -o"DSP_Lib CM4 LE FPU.txt" -j0
