$ErrorActionPreference = "Stop"

$repo = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $repo "build"
$testExe = Join-Path $buildDir "AutoRunnerTests.exe"

Set-Location $repo
New-Item -ItemType Directory -Force $buildDir | Out-Null

$gpp = Get-Command g++ -ErrorAction SilentlyContinue
$clangpp = Get-Command clang++ -ErrorAction SilentlyContinue
$cl = Get-Command cl -ErrorAction SilentlyContinue

if ($gpp) {
    Write-Host "Using g++: $($gpp.Source)"
    & $gpp.Source -std=c++11 -Wall -Wextra -Werror -Isrc test/AutoRunnerTests.cpp src/AutoRunner.cpp -o $testExe
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $testExe
    exit $LASTEXITCODE
}

if ($clangpp) {
    Write-Host "Using clang++: $($clangpp.Source)"
    & $clangpp.Source -std=c++11 -Wall -Wextra -Werror -Isrc test/AutoRunnerTests.cpp src/AutoRunner.cpp -o $testExe
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $testExe
    exit $LASTEXITCODE
}

if ($cl) {
    Write-Host "Using MSVC cl: $($cl.Source)"
    & $cl.Source /nologo /std:c++14 /W4 /WX /EHsc /Isrc test/AutoRunnerTests.cpp src/AutoRunner.cpp /Fe:$testExe
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    & $testExe
    exit $LASTEXITCODE
}

Write-Host "No supported C++ compiler found."
Write-Host "Install one of: g++, clang++, or MSVC Build Tools with cl.exe."
Write-Host "Then run: powershell -ExecutionPolicy Bypass -File scripts/run-host-tests.ps1"
exit 2
