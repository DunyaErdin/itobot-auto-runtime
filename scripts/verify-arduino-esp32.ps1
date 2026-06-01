$ErrorActionPreference = "Stop"

$repo = Split-Path -Parent $PSScriptRoot
Set-Location $repo

$arduino = Get-Command arduino-cli -ErrorAction SilentlyContinue

if (-not $arduino) {
    Write-Host "arduino-cli was not found."
    Write-Host "Install Arduino CLI, then install the ESP32 core:"
    Write-Host "  arduino-cli core update-index"
    Write-Host "  arduino-cli core install esp32:esp32"
    Write-Host "Verify examples with:"
    Write-Host "  arduino-cli compile --fqbn esp32:esp32:esp32 --libraries . examples/BasicAutoRunner"
    Write-Host "  arduino-cli compile --fqbn esp32:esp32:esp32 --libraries . examples/VisionPickupIntegration"
    exit 2
}

Write-Host "Using arduino-cli: $($arduino.Source)"
$coreList = & $arduino.Source core list
if (-not ($coreList -match "esp32:esp32")) {
    Write-Host "ESP32 Arduino core does not appear to be installed."
    Write-Host "Install it with:"
    Write-Host "  arduino-cli core update-index"
    Write-Host "  arduino-cli core install esp32:esp32"
    exit 2
}

& $arduino.Source compile --fqbn esp32:esp32:esp32 --libraries . examples/BasicAutoRunner
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $arduino.Source compile --fqbn esp32:esp32:esp32 --libraries . examples/VisionPickupIntegration
exit $LASTEXITCODE
