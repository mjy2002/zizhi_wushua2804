@echo off
chcp 65001 >nul
echo ======================================
echo      STM32工程一键清理脚本
echo  删除编译产物、缓存、临时文件
echo ======================================
echo.

:: 删除Keil编译输出 Objects Listings
for /r %%d in (Objects Listings Debug Release) do (
    if exist "%%d" (
        [rmdir] /s /q "%%d"
        [echo] 删除目录: %%d
    )
)

:: 删除各种编译输出文件 *.hex *.bin *.elf *.map *.axf
del /s /q *.hex *.bin *.elf *.map *.axf *.lst *.crf *.o *.d *.i *.srec

:: 删除CubeMX本地缓存
del /s /q *.mxproject
rd /s /q .cubemx 2>nul
del /s /q *.ioc_backup

:: 删除Keil本地布局缓存（uvopt uvoptx uvgui）
del /s /q *.uvopt
del /s /q *.uvoptx
del /s /q *.uvgui*

:: 删除VSCode工作区缓存
rd /s /q .vscode 2>nul

:: 删除keil临时文件
del /s /q *.bak *.$$$

echo.
echo ✅清理完成！
echo 提示：源码、ioc、uvprojx工程文件均保留未删除
echo.
pause
