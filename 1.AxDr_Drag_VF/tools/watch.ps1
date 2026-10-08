# ===========================================================================
#  watch.ps1 - sample firmware variables over SWD while the board keeps running
#
#  Usage:
#     pwsh -File .\tools\watch.ps1 -Seconds 20 -Interval 2 -Vars 0x20003d2c,0x20000130
#
#  How it works: OpenOCD halts the core for a few ms, reads RAM with 'mdw',
#  then resumes. The firmware never notices.
#
#  Getting the addresses:
#     arm-none-eabi-gdb -batch -q build\Debug\AxDr.elf -ex "print &pm.ctrl.wr_set"
#
#  NOTE: ASCII-only on purpose. Windows PowerShell 5.1 reads .ps1 files as ANSI
#        when there is no BOM, which corrupts non-ASCII text.
# ===========================================================================

param(
    [double]   $Seconds  = 20,
    [double]   $Interval = 2,
    [double]   $Delay    = 10,      # wait this long after reset before 1st sample
    [string[]] $Vars     = @('0x20003d2c', '0x20000130', '0x200003b8'),
    [switch]   $NoReset
)

$ErrorActionPreference = 'Stop'

$openocd = 'D:\CLion\xpack-openocd-0.12.0-7\bin\openocd.exe'
if (-not (Test-Path -LiteralPath $openocd)) {
    Write-Host "[ERROR] OpenOCD not found: $openocd" -ForegroundColor Red
    exit 1
}

# Build one OpenOCD command list: connect, reset (optional), then sample loop
$cmd = @(
    '-f', 'interface/stlink.cfg',
    '-f', 'target/stm32g4x.cfg',
    '-c', 'adapter speed 4000',
    '-c', 'init'
)
if (-not $NoReset) {
    $cmd += @('-c', 'reset run')
    $cmd += @('-c', "sleep $([int]($Delay * 1000))")
}

$samples = [int]($Seconds / $Interval)
if ($samples -lt 1) { $samples = 1 }

for ($s = 0; $s -lt $samples; $s++) {
    $cmd += @('-c', 'halt')
    $cmd += @('-c', "echo TICK-$($s + 1)")
    foreach ($v in $Vars) {
        $cmd += @('-c', "mdw $v 1")
    }
    $cmd += @('-c', 'resume')
    if ($s -lt ($samples - 1)) {
        $cmd += @('-c', "sleep $([int]($Interval * 1000))")
    }
}

$cmd += @('-c', 'halt')
$cmd += @('-c', 'reset run')
$cmd += @('-c', 'shutdown')

Write-Host "Sampling $($Vars.Count) variable(s) every ${Interval}s for ${Seconds}s ..."
Write-Host ("Addresses: " + ($Vars -join ', '))
Write-Host ""

# OpenOCD writes everything (banner, echo, mdw results) to stderr, and with
# $ErrorActionPreference='Stop' that would abort the script. Relax it and
# convert every record to a plain string so Select-String can match.
$ErrorActionPreference = 'Continue'
$out = & $openocd @cmd 2>&1 | ForEach-Object { "$_" }

$out | Select-String -Pattern 'TICK|0x[0-9a-f]{8}:'

exit 0
