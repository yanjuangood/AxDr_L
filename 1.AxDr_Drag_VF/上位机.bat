@echo off
rem ===========================================================================
rem  AxDr FOC 上位机 启动器
rem
rem  双击本文件即可启动。会自动找可用的 Python, 然后跑 tools\host_gui.py
rem
rem  想指定串口: 在本文件上右键 -> 编辑, 把最后一行改成
rem      ... host_gui.py COM5
rem  或者用命令行:  上位机.bat COM5
rem ===========================================================================
chcp 65001 >nul
setlocal

set "PY="

rem 优先用 DSH 自带的 Python (里面已经装好 pyserial)
if exist "C:\Users\yanju\.dsh\dsh-runtimes\dsh-primary-runtime\dependencies\python\python.exe" (
    set "PY=C:\Users\yanju\.dsh\dsh-runtimes\dsh-primary-runtime\dependencies\python\python.exe"
)

rem 退而求其次: PATH 里的 python / py
if not defined PY (
    where python >nul 2>nul && set "PY=python"
)
if not defined PY (
    where py >nul 2>nul && set "PY=py"
)

if not defined PY (
    echo.
    echo [错误] 找不到 Python。
    echo.
    echo 请安装 Python 3, 或者直接手动运行:
    echo     C:\Users\yanju\.dsh\dsh-runtimes\dsh-primary-runtime\dependencies\python\python.exe tools\host_gui.py
    echo.
    pause
    exit /b 1
)

echo 用 Python: %PY%
echo.

"%PY%" "%~dp0tools\host_gui.py" %*

if errorlevel 1 (
    echo.
    echo [程序异常退出] 把上面的报错信息发出来
    pause
)
endlocal
