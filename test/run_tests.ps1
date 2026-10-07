# Build and run the host-side unit tests for the sorter metrics code.
# Requires a C++ compiler (g++ or clang++) on PATH.
$ErrorActionPreference = 'Stop'
$here = $PSScriptRoot
$inc  = Join-Path $here '..'
$srcs = 'test_event_detector', 'test_event_aggregator'   # -> <name>.cpp

$cxx = $null
foreach ($c in 'g++', 'clang++') {
    if (Get-Command $c -ErrorAction SilentlyContinue) { $cxx = $c; break }
}
if (-not $cxx) {
    Write-Host 'No C++ compiler (g++/clang++) found on PATH.'
    Write-Host 'On this machine you can still validate the algorithms with:'
    Write-Host '    py -3 test\model_check.py'
    Write-Host '    py -3 test\model_check_aggregator.py'
    exit 1
}

$failed = 0
foreach ($name in $srcs) {
    $src = Join-Path $here "$name.cpp"
    $exe = Join-Path $here "$name.exe"
    Write-Host "== $name =="
    & $cxx -std=c++14 -O2 -Wall -Wextra -I $inc $src -o $exe
    if ($LASTEXITCODE -ne 0) { $failed++; continue }
    & $exe
    if ($LASTEXITCODE -ne 0) { $failed++ }
}

if ($failed -gt 0) {
    Write-Host "$failed test file(s) failed."
    exit 1
}
Write-Host 'All tests passed.'
exit 0
