# monitor.ps1
#
# Open a serial terminal on the board from the VS Code integrated terminal -
# the command-line equivalent of "Tools > Serial Monitor".
#
#   .\monitor.ps1                  # auto-detect the board, 115200 baud
#   .\monitor.ps1 -Baud 9600
#   .\monitor.ps1 -Port COM7       # skip auto-detection
#   .\monitor.ps1 -Cli C:\path\to\arduino-cli.exe
#
# Quit with Ctrl+C. Only one program can hold a COM port at a time, so close any
# other monitor first (VS Code Serial Monitor panel, Arduino IDE, PuTTY, ...).
#
# arduino-cli is looked up on PATH first and then in the copy bundled with the
# "Arduino Maker Workshop" VS Code extension, which is not always on PATH.

[CmdletBinding()]
param(
    [string] $Port = '',
    [int]    $Baud = 115200,
    [string] $Cli  = ''
)

$ErrorActionPreference = 'Stop'

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

function Resolve-Port {
    param([string] $CliPath)
    $json = & $CliPath board list --format json 2>$null | ConvertFrom-Json
    foreach ($p in $json.detected_ports) {
        if ($p.matching_boards -and ($p.matching_boards.fqbn -contains 'arduino:mbed_giga:giga')) {
            return $p.port.address
        }
    }
    $all = @([System.IO.Ports.SerialPort]::GetPortNames())
    if ($all.Count -eq 1) { return $all[0] }
    if ($all.Count -eq 0) { throw 'No serial port found. Plug the board in and retry, or pass -Port COMx.' }
    throw "No Arduino GIGA detected. Available ports: $($all -join ', '). Pass -Port COMx to pick one."
}

$cliPath = Resolve-ArduinoCli -Explicit $Cli
if (-not $Port) { $Port = Resolve-Port -CliPath $cliPath }

Write-Host "monitor: $Port @ $Baud baud - Ctrl+C to quit" -ForegroundColor Cyan
& $cliPath monitor -p $Port -c "baudrate=$Baud"
