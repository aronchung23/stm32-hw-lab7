<#
  HW-Lab7 demo helper (no PuTTY needed)
  - flashes lab07_dsp to the B-L475E-IOT01A (ST-LINK)
  - resets the board
  - shows the UART output (ST-LINK Virtual COM Port, 115200 8N1)

  Usage (VS Code terminal, in E:\C++\lab07):
    powershell -ExecutionPolicy Bypass -File .\demo.ps1                 # flash + monitor
    powershell -ExecutionPolicy Bypass -File .\demo.ps1 -NoFlash        # only reset + monitor
    powershell -ExecutionPolicy Bypass -File .\demo.ps1 -Seconds 20 -Log uart_log.txt
  Stop with Ctrl+C.
#>
param(
    [switch]$NoFlash,
    [int]$Seconds = 0,         # 0 = run until Ctrl+C
    [string]$Log = ''          # optional: also save the UART output to this file
)

$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$proj = 'lab07_dsp'
$elf  = Join-Path $root "$proj\Debug\$proj.elf"

# STM32CubeProgrammer CLI (from STM32CubeCLT or STM32CubeIDE)
$cli = @(
    'C:\ST\STM32CubeCLT_1.22.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe',
    'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
) | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $cli) {
    $cli = Get-ChildItem 'C:\ST' -Recurse -Filter STM32_Programmer_CLI.exe -ErrorAction SilentlyContinue |
           Select-Object -First 1 -ExpandProperty FullName
}
if (-not $cli) { Write-Host 'STM32_Programmer_CLI.exe not found' -ForegroundColor Red; exit 1 }

# find the ST-LINK Virtual COM port
$com = Get-CimInstance Win32_PnPEntity |
       Where-Object { $_.Name -match 'STLink Virtual COM Port \((COM\d+)\)' } |
       ForEach-Object { $Matches[1] } | Select-Object -First 1
if (-not $com) { Write-Host 'ST-LINK Virtual COM port not found - is the board plugged in (USB ST-LINK)?' -ForegroundColor Red; exit 1 }

Write-Host "== Project : $proj" -ForegroundColor Cyan
Write-Host "== COM port: $com (115200 8N1)" -ForegroundColor Cyan

if (-not $NoFlash) {
    if (-not (Test-Path $elf)) { Write-Host "Build the project first (missing $elf)" -ForegroundColor Red; exit 1 }
    Write-Host '== Flashing ...' -ForegroundColor Cyan
    $r = & $cli -c port=SWD mode=UR -w $elf -v 2>&1
    if (-not ($r | Select-String 'Download verified successfully')) {
        $r | Select-Object -Last 15 | ForEach-Object { Write-Host $_ }
        Write-Host '== Flash FAILED' -ForegroundColor Red; exit 1
    }
    Write-Host '== Flash OK (verified)' -ForegroundColor Green
}

$sp = New-Object System.IO.Ports.SerialPort $com, 115200, 'None', 8, 'One'
$sp.Encoding = [System.Text.Encoding]::UTF8
try {
    try { $sp.Open() }
    catch {
        Write-Host "== $com is busy: another terminal (an old demo.ps1 / PuTTY) is still using it." -ForegroundColor Yellow
        Write-Host '   Press Ctrl+C in that terminal (or close it), then run this command again.' -ForegroundColor Yellow
        exit 1
    }
    $sp.DiscardInBuffer()
    if ($Log) { Set-Content -Path (Join-Path $root $Log) -Value '' -NoNewline -Encoding UTF8 }
    & $cli -c port=SWD mode=UR -rst 2>&1 | Out-Null      # reset -> output starts from the banner
    Write-Host '== Board reset, UART output below (Ctrl+C to stop) ==' -ForegroundColor Green
    $sw = [Diagnostics.Stopwatch]::StartNew()
    while ($Seconds -le 0 -or $sw.Elapsed.TotalSeconds -lt $Seconds) {
        $s = $sp.ReadExisting()
        if ($s) {
            [Console]::Write($s)
            if ($Log) { [IO.File]::AppendAllText((Join-Path $root $Log), $s, [Text.Encoding]::UTF8) }
        }
        Start-Sleep -Milliseconds 50
    }
}
finally {
    if ($sp.IsOpen) { $sp.Close() }
    Write-Host "`n== COM port closed" -ForegroundColor Cyan
}
