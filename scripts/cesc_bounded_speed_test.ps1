param(
    [string]$PortName = "COM15",
    [ValidateRange(-500, 500)]
    [int]$SpeedTargetRpm,
    [ValidateRange(5, 60)]
    [int]$TimeoutSeconds = 15,
    [ValidateRange(20, 300)]
    [int]$SpeedSamples = 20,
    [ValidateRange(1, 100)]
    [int]$SpeedLogEvery = 10
)

$ErrorActionPreference = "Stop"
$workspace = Split-Path -Parent $PSScriptRoot
$logDirectory = Join-Path $workspace ".codex_tmp"
[void][System.IO.Directory]::CreateDirectory($logDirectory)
$runId = [Guid]::NewGuid().ToString("N")
$stdoutPath = Join-Path $logDirectory "speed-$runId.out.log"
$stderrPath = Join-Path $logDirectory "speed-$runId.err.log"
$smokeScript = Join-Path $PSScriptRoot "cesc_serial_smoke.ps1"
$arguments = @(
    "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", $smokeScript,
    "-PortName", $PortName, "-StreamDurationMs", "100", "-RunSpeedTest",
    "-SpeedTargetRpm", $SpeedTargetRpm, "-SpeedSamples", $SpeedSamples,
    "-SpeedLogEvery", $SpeedLogEvery
)
$argumentText = ($arguments | ForEach-Object {
    if ($_ -match '[\s"]') { '"' + ($_ -replace '"', '\"') + '"' } else { $_ }
}) -join ' '
$process = Start-Process -FilePath "powershell.exe" -ArgumentList $argumentText `
    -RedirectStandardOutput $stdoutPath -RedirectStandardError $stderrPath `
    -WindowStyle Hidden -PassThru
$stopwatch = [System.Diagnostics.Stopwatch]::StartNew()

try {
    while (-not $process.HasExited -and
           $stopwatch.Elapsed.TotalSeconds -lt $TimeoutSeconds) {
        Start-Sleep -Milliseconds 100
        $process.Refresh()
    }
    if (-not $process.HasExited) {
        $process.Kill()
        [void]$process.WaitForExit(2000)
        Write-Output "TIMEOUT pid=$($process.Id) limit=${TimeoutSeconds}s"
        if (Test-Path -LiteralPath $stdoutPath) {
            Get-Content -LiteralPath $stdoutPath
        }
        if (Test-Path -LiteralPath $stderrPath) {
            [Console]::Error.WriteLine((Get-Content -Raw -LiteralPath $stderrPath))
        }
        exit 124
    }
    if (Test-Path -LiteralPath $stdoutPath) {
        Get-Content -LiteralPath $stdoutPath
    }
    if (Test-Path -LiteralPath $stderrPath) {
        [Console]::Error.WriteLine((Get-Content -Raw -LiteralPath $stderrPath))
    }
    exit $process.ExitCode
} finally {
    if (-not $process.HasExited) { $process.Kill() }
    $process.Dispose()
}
