# flash.ps1
#
# Build and upload the sorter firmware to an Arduino GIGA R1 WiFi.
#
# The GIGA runs the *same* sketch on both cores, so a full flash is two passes:
#   1) Cortex-M7 (main core)     - setup()/loop(), the Serial printout
#   2) Cortex-M4 (co-processor)  - the acquisition + detector pipeline
# Both passes use the same USB port and the M7 goes first (see README).
#
#   .\flash.ps1                 auto-detect the GIGA, flash M7 then M4
#   .\flash.ps1 -Port COM6      use an explicit port
#   .\flash.ps1 -SkipUpload     compile both cores only (no board needed)
#   .\flash.ps1 -Monitor        open the 115200 serial monitor afterwards
#   .\flash.ps1 -Cli C:\path\to\arduino-cli.exe
#
# arduino-cli is looked up on PATH first and then in the copy bundled with the
# "Arduino Maker Workshop" VS Code extension, which is not always on PATH.

[CmdletBinding()]
param(
    [string] $Port = '',
    [string] $Cli  = '',
    [switch] $SkipUpload,
    [switch] $Monitor
)

$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot

$mainFqbn = 'arduino:mbed_giga:giga'
$cm4Fqbn  = 'arduino:mbed_giga:giga:target_core=cm4'

function Resolve-ArduinoCli {
    param([string] $Explicit)
    if ($Explicit) { return $Explicit }
    $cmd = Get-Command arduino-cli -ErrorAction SilentlyContinue
    if ($cmd) { return $cmd.Source }
    $glob = Join-Path $env:USERPROFILE '.vscode\extensions\thelastoutpostworkshop.arduino-maker-workshop-*\arduino_cli\win32\arduino-cli.exe'
    $found = @(Get-ChildItem -Path $glob -ErrorAction SilentlyContinue | Sort-Object FullName -Descending)
    if ($found.Count -gt 0) { return $found[0].FullName }
    throw 'arduino-cli not found: install it (winget install ArduinoSA.CLI) or pass -Cli <path>.'
}

function Resolve-GigaPort {
    param([string] $Explicit, [string] $CliPath)
    if ($Explicit) { return $Explicit }
    $json = & $CliPath board list --format json 2>$null | ConvertFrom-Json
    foreach ($p in $json.detected_ports) {
        if ($p.matching_boards -and ($p.matching_boards.fqbn -contains 'arduino:mbed_giga:giga')) {
            return $p.port.address
        }
    }
    # A board sitting in the bootloader may not report a matching FQBN.
    $fallback = @([System.IO.Ports.SerialPort]::GetPortNames())
    if ($fallback.Count -eq 1) {
        Write-Host "warning: no GIGA identified, falling back to the only serial port $($fallback[0])" -ForegroundColor Yellow
        return $fallback[0]
    }
    throw 'No Arduino GIGA detected. Use the USB-C *programming* port (next to the DC jack), or pass -Port COMx.'
}

$cliPath = Resolve-ArduinoCli -Explicit $Cli
Write-Host "arduino-cli : $cliPath"

if (-not $SkipUpload) {
    $Port = Resolve-GigaPort -Explicit $Port -CliPath $cliPath
    Write-Host "port        : $Port"
}

$targets = @(
    @{ Name = 'Cortex-M7 (main)';         Fqbn = $mainFqbn },
    @{ Name = 'Cortex-M4 (co-processor)'; Fqbn = $cm4Fqbn  }
)

foreach ($target in $targets) {
    Write-Host ''
    Write-Host "== $($target.Name): compile" -ForegroundColor Cyan
    & $cliPath compile --fqbn $target.Fqbn .
    if ($LASTEXITCODE -ne 0) { throw "compile failed: $($target.Fqbn)" }

    if ($SkipUpload) { continue }

    Write-Host "== $($target.Name): upload" -ForegroundColor Cyan
    & $cliPath upload --fqbn $target.Fqbn -p $Port .
    if ($LASTEXITCODE -ne 0) {
        throw "upload failed: $($target.Fqbn). If the port disappeared, double-tap the GIGA RESET button and pass the bootloader -Port (see README)."
    }
}

Write-Host ''
if ($SkipUpload) {
    Write-Host 'Both cores compiled (no upload: -SkipUpload).' -ForegroundColor Green
    exit 0
}

Write-Host 'Flashed both cores. Open the serial monitor at 115200 for the boot banner.' -ForegroundColor Green

if ($Monitor) { & $cliPath monitor -p $Port -c baudrate=115200 }
