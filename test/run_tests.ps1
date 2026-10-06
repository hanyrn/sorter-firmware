# Build and run the host-side unit test for EventDetector.
# Requires a C++ compiler (g++ or clang++) on PATH.
$ErrorActionPreference = 'Stop'
$here = $PSScriptRoot
$src  = Join-Path $here 'test_event_detector.cpp'
$exe  = Join-Path $here 'test_event_detector.exe'

$cxx = $null
foreach ($c in 'g++', 'clang++') {
    if (Get-Command $c -ErrorAction SilentlyContinue) { $cxx = $c; break }
}
if (-not $cxx) {
    Write-Host 'No C++ compiler (g++/clang++) found on PATH.'
    Write-Host 'On this machine you can still validate the algorithm with:'
    Write-Host '    py -3 test\model_check.py'
    exit 1
}

& $cxx -std=c++14 -O2 -Wall -Wextra -I (Join-Path $here '..') $src -o $exe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $exe
exit $LASTEXITCODE
