# ===========================================================================
#  One-click flash: download the newest .elf to STM32G431RBTx via OpenOCD
#
#  Usage:  powershell -ExecutionPolicy Bypass -File .\flash.ps1
#          (or: pwsh -File .\flash.ps1)
#
#  NOTE: kept ASCII-only on purpose. Windows PowerShell 5.1 reads .ps1 files
#        as ANSI when there is no BOM, which corrupts non-ASCII text.
# ===========================================================================

$ErrorActionPreference = 'Stop'

$root    = $PSScriptRoot
$openocd = 'D:\CLion\xpack-openocd-0.12.0-7\bin\openocd.exe'
$cfg     = Join-Path $root 'openocd.cfg'

if (-not (Test-Path -LiteralPath $openocd)) {
    Write-Host "[ERROR] OpenOCD not found: $openocd" -ForegroundColor Red
    exit 1
}
if (-not (Test-Path -LiteralPath $cfg)) {
    Write-Host "[ERROR] config not found: $cfg" -ForegroundColor Red
    exit 1
}

# newest AxDr.elf under build\ or CLion's cmake-build-*\
$elf = Get-ChildItem -LiteralPath $root -Recurse -Filter 'AxDr.elf' -File -ErrorAction SilentlyContinue |
       Sort-Object LastWriteTime -Descending | Select-Object -First 1

if (-not $elf) {
    Write-Host "[ERROR] AxDr.elf not found. Build first (CLion Ctrl+F9, or 'cmake --build build/Debug')." -ForegroundColor Red
    exit 1
}

# OpenOCD -c goes through the Tcl interpreter, where '\' is an escape char.
# Use forward slashes and wrap the path in braces.
$elfPath = $elf.FullName -replace '\\', '/'
$cfgPath = $cfg          -replace '\\', '/'

Write-Host "ELF    : $($elf.FullName)"
Write-Host "Config : $cfg"
Write-Host ""

& $openocd -f $cfgPath -c "program {$elfPath} verify reset exit"
exit $LASTEXITCODE
