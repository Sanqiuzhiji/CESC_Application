param(
    [string]$PortName = "COM15",
    [switch]$RunMotionTests,
    [switch]$VerifyThermalShutdown
)

$ErrorActionPreference = "Stop"
$smoke = Join-Path $PSScriptRoot "cesc_serial_smoke.ps1"

Write-Output "[1/6] Verify ambient temperature ADC and safe READY state"
& $smoke -PortName $PortName -StreamDurationMs 200

Write-Output "[2/6] Persist maximum motor Iq = 500 mA without changing other user configuration"
& $smoke -PortName $PortName -SetMaximumIqMa 500

if (-not $RunMotionTests) {
    Write-Output "PAUSED motion tests were not requested. Re-run with -RunMotionTests after securing the motor and clearing the shaft area."
    exit 2
}

Write-Output "[3/6] Verify positive Iq FOC step at +500 mA (hard abort above 500 RPM)"
& $smoke -PortName $PortName -RunIqCurrentTest -IqCurrentMa 500 -IqDurationMs 800 -StreamDurationMs 100

Write-Output "[3/6] Verify negative Iq FOC step at -500 mA (hard abort above 500 RPM)"
& $smoke -PortName $PortName -RunIqCurrentTest -IqCurrentMa -500 -IqDurationMs 800 -StreamDurationMs 100

Write-Output "[4/6,5/6,6/6] Verify speed-current FOC and SVM path at +100 RPM"
& $smoke -PortName $PortName -RunSpeedTest -SpeedTargetRpm 100 -SpeedSamples 30 -SpeedLogEvery 5 -RequireSpeedCurrentFoc -StreamDurationMs 100

Write-Output "[4/6,5/6,6/6] Verify speed-current FOC and SVM path at -100 RPM"
& $smoke -PortName $PortName -RunSpeedTest -SpeedTargetRpm -100 -SpeedSamples 30 -SpeedLogEvery 5 -RequireSpeedCurrentFoc -StreamDurationMs 100

if ($VerifyThermalShutdown) {
    Write-Output "[1/6] Apply controlled external heat to the PCB NTC now; pass requires >=100 C, FAULT latch, zero thermal current limit and disabled gate."
    & $smoke -PortName $PortName -ExpectThermalShutdown -StreamDurationMs 100
} else {
    Write-Output "PAUSED over-temperature shutdown requires controlled external heating. Re-run with -RunMotionTests -VerifyThermalShutdown while the NTC is held above 100 C."
    exit 2
}

Write-Output "PASS phase-1 validation complete; every commanded speed was within +/-500 RPM."
