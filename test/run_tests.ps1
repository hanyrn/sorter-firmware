# Build and run the host-side unit tests for the sorter metrics code.
# Requires a C++ compiler (g++ or clang++) on PATH.
$ErrorActionPreference = 'Stop'
$here = $PSScriptRoot
$inc  = Join-Path $here '..'
$srcs = 'test_event_detector', 'test_event_aggregator', 'test_signal_model'   # -> <name>.cpp

$cxx = $null
foreach ($c in 'g++', 'clang++') {
    if (Get-Command $c -ErrorAction SilentlyContinue) { $cxx = $c; break }
}
if (-not $cxx) {
    # Not on PATH: fall back to well-known install locations so the runner works
    # even when the compiler was installed after the shell started.  In particular
    # the WinLibs MinGW-w64 package installed through winget (...\WinGet\Packages\
    # BrechtSanders.WinLibs.*\mingw64\bin) is not added to PATH automatically.
    $candidates = @()
    $wingetPkgs = Join-Path $env:LOCALAPPDATA 'Microsoft\WinGet\Packages'
    if (Test-Path $wingetPkgs) {
        $candidates += Get-ChildItem $wingetPkgs -Recurse -Filter 'g++.exe' -ErrorAction SilentlyContinue |
            Select-Object -ExpandProperty FullName
    }
    $candidates += @(
        'C:\msys64\mingw64\bin\g++.exe',
        'C:\mingw64\bin\g++.exe',
        'C:\ProgramData\mingw64\bin\g++.exe'
    )
    foreach ($cand in $candidates) {
        if (Test-Path $cand) {
            $cxx = $cand
            # Prepend its folder so g++ can also find its own runtime DLLs.
            $env:PATH = (Split-Path $cand) + ';' + $env:PATH
            Write-Host "Using compiler: $cand"
            break
        }
    }
}
if (-not $cxx) {
    Write-Host 'No C++ compiler (g++/clang++) found on PATH or in the usual install locations.'
    Write-Host 'Install one, e.g.:  winget install BrechtSanders.WinLibs.POSIX.UCRT'
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
