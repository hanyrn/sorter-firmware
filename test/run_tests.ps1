# Build and run the host-side unit tests for the sorter metrics code.
# Requires a C++ compiler (g++ or clang++) on PATH.
$ErrorActionPreference = 'Stop'
$here = $PSScriptRoot
$inc  = Join-Path $here '..'
$srcs = 'test_event_detector', 'test_event_aggregator', 'test_signal_model', 'test_bench_detect'   # -> <name>.cpp

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
    # test_bench_detect gets a second, longer run on top of its 4-cycle default: the width
    # floor is only really exercised where the generator draws a needle whose *duration* is
    # close to it, and that only happens across many pattern cycles.  A floor compared
    # against the wrong number (the note under DET_MIN_WIDTH_SAMPLES in config.h) shows up
    # as "peak span(s) INSIDE the documented envelope were reported as `rejected`" - and it
    # does not show up in the default run at all (measured: 3 WIDTH_TOO_NARROW at 12 cycles,
    # 0 at 4).
    if ($name -eq 'test_bench_detect') {
        & $exe --cycles 12
        if ($LASTEXITCODE -ne 0) { $failed++ }
    }
}

if ($failed -gt 0) {
    Write-Host "$failed test file(s) failed."
    exit 1
}

# The harness and the viewer only meet on the shape of the --stream text, and no C++
# test notices when that shape changes: live_plot.py simply stops drawing the overlay
# the line carried.  Check the pair end to end now that the harness is built (Python
# only; skipped, not failed, when there is none).
$py = $null
foreach ($c in 'py', 'python') {
    if (Get-Command $c -ErrorAction SilentlyContinue) { $py = $c; break }
}
if ($py) {
    Write-Host '== stream_check (harness -> viewer protocol) =='
    $pyArgs = @((Join-Path $here 'stream_check.py'))
    if ($py -eq 'py') { $pyArgs = @('-3') + $pyArgs }
    & $py @pyArgs
    if ($LASTEXITCODE -ne 0) { $failed++ }

    # analyze_capture.py measures a capture rather than drawing it (where a detector
    # window sits on its pulse, whether a channel steps away from the code), so it has
    # no board to check against here - it builds its own capture with the defects
    # planted in it and checks the measurements against those.
    Write-Host '== analyze_capture --selftest (capture measurements) =='
    $pyArgs = @((Join-Path $here 'analyze_capture.py'), '--selftest')
    if ($py -eq 'py') { $pyArgs = @('-3') + $pyArgs }
    & $py @pyArgs
    if ($LASTEXITCODE -ne 0) { $failed++ }
} else {
    Write-Host '== stream_check skipped: no Python (py/python) on PATH =='
}

if ($failed -gt 0) {
    Write-Host "$failed test file(s) failed."
    exit 1
}
Write-Host 'All tests passed.'
exit 0
