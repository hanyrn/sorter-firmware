# verify_adc_pinmap.ps1
# ---------------------------------------------------------------------------
# Cross-checks the AD7606 pin map so a wiring slip or a stale README is caught
# before it reaches the hardware:
#
#   * all 32 data lines in AD7606_DB_LINES[] (config.h) must exist in the
#     installed GIGA core (variant.cpp) and carry the right (port, bit)
#   * the 6 shared control pins (CONVST/RESET/RD/CS/BUSY0/BUSY1) must too
#   * README.md must list exactly the same pins, in the same order
#   * no port/bit and no Arduino pin may be used twice
#   * no used pin may carry an on-board peripheral function (LED, USB, CAN,
#     radio, BOOT, I2C1, DAC ...) - i.e. it must be plain GPIO
#
# Usage: powershell -ExecutionPolicy Bypass -File test\verify_adc_pinmap.ps1
# Exit code 0 = all checks passed, 1 = at least one mismatch.
# Requires the arduino:mbed_giga core to be installed.
# ---------------------------------------------------------------------------
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$pkg  = Join-Path $env:LOCALAPPDATA 'Arduino15\packages\arduino\hardware\mbed_giga'
$vg   = Get-ChildItem $pkg -Directory -ErrorAction SilentlyContinue |
        Sort-Object Name -Descending |
        ForEach-Object { Join-Path $_.FullName 'variants\GIGA' } |
        Where-Object { Test-Path (Join-Path $_ 'variant.cpp') } |
        Select-Object -First 1
if (-not $vg) { Write-Host 'GIGA core not found - install arduino:mbed_giga first.'; exit 1 }
Write-Host "GIGA core  : $vg"
Write-Host "Sketch     : $root"

# ---------------------------------------------------------------- variant.cpp
$variant  = @{}        # Arduino pin number -> PinName (e.g. 22 -> PJ_12)
$alias    = @{}        # 'A0' -> 76
$reserved = @{}        # Arduino pin number -> on-board function (e.g. 86 -> LEDR)
$inArray  = $false
foreach ($line in (Get-Content (Join-Path $vg 'variant.cpp'))) {
    if ($line -match 'PinDescription\s+g_APinDescription') { $inArray = $true; continue }
    if (-not $inArray) { continue }
    if ($line -match '^\s*};') { break }
    if ($line -notmatch '^\s*\{\s*([A-Za-z0-9_]+)\s*,.*?//\s*(.+)$') { continue }
    $pinName = $Matches[1]
    $comment = $Matches[2].Trim()
    $num = -1; $label = ''
    if ($comment -match '^A(\d+)\b') {
        $aIdx = [int]$Matches[1]
        $num  = 76 + $aIdx                            # PIN_A0 = 76 on the GIGA
        if ($comment -match '\(D(\d+)\)') { $num = [int]$Matches[1] }
        $alias["A$aIdx"] = $num
        $label = (($comment -replace '^A\d+\b', '') -replace '\(D\d+\)', '').Trim()
    } elseif ($comment -match '^D(\d+)\b') {
        $num   = [int]$Matches[1]
        $label = ($comment -replace '^D\d+\b', '').Trim()
    }
    if ($num -lt 0) { continue }
    $variant[$num] = $pinName
    if ($label -ne '') { $reserved[$num] = $label }
}
if ($variant.Count -eq 0) { Write-Host 'Could not parse variant.cpp.'; exit 1 }
Write-Host ("Parsed     : {0} pins, {1} with an on-board function" -f $variant.Count, $reserved.Count)

function Decode([string]$pinName) {
    if ($pinName -match '^P([A-K])_(\d+)$') {
        return [pscustomobject]@{ Port = "GPIO$($Matches[1])"; Bit = [int]$Matches[2] }
    }
    return $null
}

# 'A0' / 'D22' -> Arduino pin number.
function ResolvePin([string]$token) {
    if ($token -match '^A\d+$')    { return $alias[$token] }
    if ($token -match '^D(\d+)$')  { return [int]$Matches[1] }
    return $null
}

# ------------------------------------------------------------------- config.h
$cfg  = Get-Content (Join-Path $root 'config.h')
$data = New-Object System.Collections.ArrayList
foreach ($line in $cfg) {
    foreach ($m in [regex]::Matches($line, '\{\s*([A-Z]\d+)\s*,\s*(GPIO[A-K])\s*,\s*(\d+)\s*\}')) {
        [void]$data.Add([pscustomobject]@{
            Token = $m.Groups[1].Value
            Port  = $m.Groups[2].Value
            Bit   = [int]$m.Groups[3].Value })
    }
}

$ctrlOrder    = @('CONVST','RESET','RD','CS','BUSY0','BUSY1')
$ctrlTokens   = @{}   # 'CONVST' -> 'A0'   (Arduino name used in config.h)
$ctrlNumbers  = @{}   # 'CONVST' -> 76     (D76 = A0)
$ctrlPortBits = @{}
foreach ($n in $ctrlOrder) {
    foreach ($line in $cfg) {
        if ($line -match "AD7606_$n`_PIN\s*=\s*([AD]\d+)") {
            $t = $Matches[1]; $ctrlTokens[$n] = $t; $ctrlNumbers[$n] = ResolvePin $t
        }
        if ($line -match "AD7606_$n`_PORT\s+(GPIO[A-K])") { $ctrlPortBits[$n] = $Matches[1] }
        if ($line -match "AD7606_$n`_BIT\s+(\d+)")        { $ctrlPortBits[$n] = $ctrlPortBits[$n] + $Matches[1] }
    }
}

# --------------------------------------------------------------------- README
$rd         = Get-Content (Join-Path $root 'README.md')
$readmeData = New-Object System.Collections.ArrayList
$readmeCtrl = New-Object System.Collections.ArrayList
foreach ($line in $rd) {
    if ($line -match '^\|\s*[01]\s*\|\s*DB\d+\.\.DB\d+') {
        $cells = $line.Trim() -split '\|'
        foreach ($m in [regex]::Matches($cells[3], 'D\d+')) { [void]$readmeData.Add($m.Value) }
    } elseif ($line -match '^\|\s*(CONVST|RESET|RD|CS|BUSY module [01])\b') {
        $cells = $line.Trim() -split '\|'
        if ($cells[2] -match '([AD]\d+)') { [void]$readmeCtrl.Add($Matches[1]) }
    }
}

# --------------------------------------------------------------------- checks
$fails = 0
function Fail([string]$msg) { Write-Host "FAIL: $msg"; $script:fails++ }

Write-Host ''
Write-Host '=== data lines: config.h vs variant.cpp ==='
if ($data.Count -ne 32) { Fail "expected 32 data lines, found $($data.Count)" }
$i = 0
foreach ($e in $data) {
    $num = ResolvePin $e.Token
    $pin = $variant[$num]
    $exp = Decode $pin
    $ok  = $exp -and $exp.Port -eq $e.Port -and $exp.Bit -eq $e.Bit
    if (-not $ok) { Fail "data line $i ($($e.Token)): config $($e.Port)$($e.Bit), variant $pin" }
    $verdict = 'FAIL'; if ($ok) { $verdict = 'OK' }
    Write-Host ("[{0,2}] {1,-4} = D{2,-3} = {3,-8} config says {4}{5}  {6}" -f `
        $i, $e.Token, $num, $pin, $e.Port, $e.Bit, $verdict)
    $i++
}

Write-Host ''
Write-Host '=== control pins: config.h vs variant.cpp ==='
foreach ($n in $ctrlOrder) {
    $tok = $ctrlTokens[$n]
    $num = $ctrlNumbers[$n]
    if ($null -eq $num) { Fail "${n}: cannot resolve pin '$tok' in config.h"; continue }
    $pin  = $variant[$num]
    $exp  = Decode $pin
    $want = $ctrlPortBits[$n]
    $got  = '(non-plain pin)'
    if ($exp) { $got = "$($exp.Port)$($exp.Bit)" }
    $ok = $exp -and $got -eq $want
    if (-not $ok) { Fail "$n ($tok = D$num): config $want, variant $pin" }
    $verdict = 'FAIL'; if ($ok) { $verdict = 'OK' }
    Write-Host ("{0,-7} {1,-3} = D{2,-3} = {3,-8} -> {4,-8} config says {5}  {6}" -f `
        $n, $tok, $num, $pin, $got, $want, $verdict)
}

Write-Host ''
Write-Host '=== README.md vs config.h ==='
if ($readmeData.Count -ne $data.Count) {
    Fail "README lists $($readmeData.Count) data pins, config.h has $($data.Count)"
} else {
    $bad = 0
    for ($k = 0; $k -lt $data.Count; $k++) {
        if ($readmeData[$k] -ne $data[$k].Token) {
            Fail "data entry ${k}: README=$($readmeData[$k]) config=$($data[$k].Token)"; $bad++
        }
    }
    if ($bad -eq 0) { Write-Host 'OK: all 32 data pins match (order included)' }
}
if ($readmeCtrl.Count -ne 6) {
    Fail "README control table has $($readmeCtrl.Count) pins"
} else {
    $bad = 0
    for ($k = 0; $k -lt 6; $k++) {
        $want = $ctrlTokens[$ctrlOrder[$k]]
        if ($readmeCtrl[$k] -ne $want) {
            Fail "$($ctrlOrder[$k]): README=$($readmeCtrl[$k]) config=$want"; $bad++
        }
    }
    if ($bad -eq 0) { Write-Host 'OK: all 6 control pins match (order included)' }
}

Write-Host ''
Write-Host '=== duplicates / reserved pins ==='
$all = New-Object System.Collections.ArrayList
$k = 0
foreach ($e in $data) {
    [void]$all.Add([pscustomobject]@{
        Name    = $e.Token
        Number  = ResolvePin $e.Token
        PortBit = "$($e.Port)$($e.Bit)"
        What    = "data line $k" })
    $k++
}
foreach ($n in $ctrlOrder) {
    [void]$all.Add([pscustomobject]@{
        Name    = $ctrlTokens[$n]
        Number  = $ctrlNumbers[$n]
        PortBit = $ctrlPortBits[$n]
        What    = $n })
}
$dup = $all | Group-Object PortBit | Where-Object { $_.Count -gt 1 }
if ($dup) {
    foreach ($d in $dup) { Fail "port/bit $($d.Name) used by $(($d.Group | ForEach-Object { $_.What }) -join ', ')" }
} else { Write-Host 'OK: every used port/bit is unique' }
$dupName = $all | Group-Object Name | Where-Object { $_.Count -gt 1 }
if ($dupName) { foreach ($d in $dupName) { Fail "pin $($d.Name) used twice" } }
else { Write-Host 'OK: every Arduino pin number is used once' }
$bad = 0
foreach ($p in $all) {
    if ($null -ne $p.Number -and $reserved.ContainsKey([int]$p.Number)) {
        Fail "$($p.What) uses $($p.Name), which is on-board '$($reserved[[int]$p.Number])'"; $bad++
    }
}
if ($bad -eq 0) { Write-Host 'OK: no used pin carries an on-board peripheral function' }

Write-Host ''
Write-Host "FAILURES=$fails"
if ($fails -gt 0) { exit 1 }
Write-Host 'Pin map OK.'
exit 0
