# flash.ps1
#
# Build and upload the sorter firmware to an Arduino GIGA R1 WiFi.
#
# The GIGA runs the *same* sketch on both cores, so a full flash is two passes:
#   1) Cortex-M7 (main core)     - setup()/loop(), the Serial printout
#   2) Cortex-M4 (co-processor)  - the acquisition + detector pipeline
# The M7 goes first, then the M4 (see README). The board re-enumerates while it is
# being flashed - the bootloader keeps the touched port and the app comes back on a
# different one - so the port is re-detected between the two passes.
#
#   .\flash.ps1                 auto-detect the GIGA, flash M7 then M4
#   .\flash.ps1 -Port COM6      use an explicit port
#   .\flash.ps1 -SkipUpload     compile both cores only (no board needed)
#   .\flash.ps1 -Monitor        open the 115200 serial monitor afterwards
#   .\flash.ps1 -Split 50_50    choose another flash split (default 75_25)
#   .\flash.ps1 -Cli C:\path\to\arduino-cli.exe
#
# The flash split has to be stated explicitly: `upload.address_m4` is only
# defined *inside* the split menu options of boards.txt, arduino-cli does not
# apply menu defaults, and an unset split therefore leaves the M4 upload address
# empty. The upload then runs as `dfu-util --dfuse-address=:leave`, which fails
# with "Only DfuSe file version 1.1a is supported". "75_25" (1.5MB M7 + 0.5MB M4)
# puts the M4 image at 0x08180000 - exactly where the M7 looks for it
# (CM4_BINARY_START) - and leaves 1441792 bytes for the M7 sketch.
#
# arduino-cli is looked up on PATH first and then in the copy bundled with the
# "Arduino Maker Workshop" VS Code extension, which is not always on PATH.

[CmdletBinding()]
param(
    [string] $Port  = '',
    [string] $Cli   = '',
    [string] $Split = '75_25',
    [switch] $SkipUpload,
    [switch] $Monitor
)

$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot

$mainFqbn = "arduino:mbed_giga:giga:split=$Split"
$cm4Fqbn  = "arduino:mbed_giga:giga:target_core=cm4,split=$Split"

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

function Test-SerialPort {
    param([string] $Name)
    return ([System.IO.Ports.SerialPort]::GetPortNames()) -contains $Name
}

$cliPath = Resolve-ArduinoCli -Explicit $Cli
Write-Host "arduino-cli : $cliPath"
Write-Host "flash split : $Split"

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

    # The board re-enumerates while it is being flashed: the port we touched (say
    # COM7) ends up owned by the bootloader and the app comes back on a *different*
    # port (COM6 here). Uploading the M4 to the stale port fails inside dfu-util
    # with "No DFU capable USB device available" (uploading error: exit status 74),
    # so re-resolve whenever the port we hold has gone away.
    if (-not (Test-SerialPort $Port)) {
        Write-Host "port $Port is gone (the board re-enumerated in the previous pass) - re-detecting" -ForegroundColor Yellow
        $Port = Resolve-GigaPort -Explicit '' -CliPath $cliPath
        Write-Host "port        : $Port"
    }

    Write-Host "== $($target.Name): upload" -ForegroundColor Cyan
    & $cliPath upload --fqbn $target.Fqbn -p $Port .
    if ($LASTEXITCODE -ne 0) {
        throw ("upload failed: $($target.Fqbn).`n" +
               "  * dfu-util 'No DFU capable USB device available': the 1200-bps touch did not " +
               "reach the bootloader. The board re-enumerates while it is flashed, so the port " +
               "the board was found on can be gone by the second pass (this script re-detects it " +
               "- check 'arduino-cli board list'). Close whatever is holding the port (VS Code " +
               "Serial Monitor panel, another monitor) and retry; on a fresh machine install " +
               "the board's WinUSB driver once with post_install.bat from the core folder.`n" +
               "  * 'Only DfuSe file version 1.1a is supported': the upload address was empty - " +
               "pass an explicit -Split (see the header of this script).`n" +
               "  * If the port disappeared, double-tap the GIGA RESET button and upload again " +
               "(see README).")
    }
}

Write-Host ''
if ($SkipUpload) {
    Write-Host 'Both cores compiled (no upload: -SkipUpload).' -ForegroundColor Green
    exit 0
}

Write-Host 'Flashed both cores. Open the serial monitor at 115200 for the boot banner.' -ForegroundColor Green

if ($Monitor) { & $cliPath monitor -p $Port -c baudrate=115200 }
