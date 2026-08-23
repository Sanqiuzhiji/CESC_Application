param(
    [string]$PortName = "COM15",
    [int]$BaudRate = 115200,
    [int]$StreamDurationMs = 500,
    [switch]$RunCommissioningTest,
    [switch]$RunEncoderAlignment,
    [switch]$RunEncoderVoltageTest,
    [switch]$RunCurrentFocTest,
    [switch]$RunResistanceTest,
    [ValidateSet(-1, 1)]
    [int]$CommissioningDirection = 1
)

$ErrorActionPreference = "Stop"

function Get-CescCrc16([byte[]]$Bytes) {
    [uint16]$crc = 0
    foreach ($value in $Bytes) {
        $crc = [uint16]($crc -bxor ([uint16]$value -shl 8))
        for ($bit = 0; $bit -lt 8; ++$bit) {
            if (($crc -band 0x8000) -ne 0) {
                $crc = [uint16](($crc -shl 1) -bxor 0x1021)
            } else {
                $crc = [uint16]($crc -shl 1)
            }
        }
    }
    return $crc
}

function New-CescFrame([byte]$Service, [byte]$Command,
                       [uint16]$Sequence, [byte[]]$Payload) {
    [byte[]]$covered = @(1, 0, $Service, $Command,
        ($Sequence -band 0xff), ($Sequence -shr 8),
        ($Payload.Length -band 0xff), ($Payload.Length -shr 8)) + $Payload
    $crc = Get-CescCrc16 $covered
    return [byte[]](@(0x43, 0x45) + $covered +
        @(($crc -band 0xff), ($crc -shr 8)))
}

function Read-Available([System.IO.Ports.SerialPort]$Port, [int]$WaitMs) {
    Start-Sleep -Milliseconds $WaitMs
    $bytes = [System.Collections.Generic.List[byte]]::new()
    while ($Port.BytesToRead -gt 0) { $bytes.Add([byte]$Port.ReadByte()) }
    return $bytes.ToArray()
}

function Send-CescRequest([System.IO.Ports.SerialPort]$Port, [byte]$Service,
                          [byte]$Command, [uint16]$Sequence,
                          [byte[]]$Payload, [int]$WaitMs = 100) {
    $frame = New-CescFrame $Service $Command $Sequence $Payload
    $Port.Write($frame, 0, $frame.Length)
    return Read-Available $Port $WaitMs
}

function Assert-CescResponse([byte[]]$Bytes, [byte]$Service,
                             [byte]$Command, [uint16]$Sequence) {
    if ($Bytes.Length -lt 14 -or $Bytes[0] -ne 0x43 -or $Bytes[1] -ne 0x45) {
        throw "Missing CESC response for service=$Service command=$Command"
    }
    if ($Bytes[3] -ne 1 -or $Bytes[4] -ne $Service -or
        $Bytes[5] -ne $Command -or $Bytes[6] -ne ($Sequence -band 0xff) -or
        $Bytes[7] -ne ($Sequence -shr 8)) {
        throw "Unexpected CESC response header"
    }
    $length = [int]$Bytes[8] -bor ([int]$Bytes[9] -shl 8)
    if ($Bytes.Length -lt (12 + $length)) { throw "Truncated CESC response" }
    $expected = [int]$Bytes[10 + $length] -bor
        ([int]$Bytes[11 + $length] -shl 8)
    $actual = Get-CescCrc16 $Bytes[2..(9 + $length)]
    if ($expected -ne $actual) {
        $hex = ($Bytes | ForEach-Object { $_.ToString("X2") }) -join " "
        throw "CESC response CRC mismatch expected=$expected actual=$actual bytes=$hex"
    }
    $status = [int]$Bytes[10] -bor ([int]$Bytes[11] -shl 8)
    if ($status -ne 0) { throw "CESC status=$status" }
    return $Bytes[12..(9 + $length)]
}

function Invoke-CescRequestWithRetry(
    [System.IO.Ports.SerialPort]$Port, [byte]$Service, [byte]$Command,
    [uint16]$Sequence, [byte[]]$Payload, [int]$WaitMs = 100) {
    for ($attempt = 0; $attempt -lt 3; ++$attempt) {
        try {
            return Assert-CescResponse (
                Send-CescRequest $Port $Service $Command $Sequence $Payload $WaitMs
            ) $Service $Command $Sequence
        } catch {
            if ($attempt -eq 2) { throw }
            Start-Sleep -Milliseconds 20
        }
    }
}

$port = [System.IO.Ports.SerialPort]::new(
    $PortName, $BaudRate, "None", 8, "One")
$port.ReadTimeout = 200
$port.WriteTimeout = 1000
try {
    $port.Open()
    $port.DiscardInBuffer()
    $hello = Invoke-CescRequestWithRetry $port 0 0 1 ([byte[]]@(1,1,0,0,0,0))
    $sample = Invoke-CescRequestWithRetry $port 2 1 2 ([byte[]]@(0))
    $raw = [int]$sample[4] -bor ([int]$sample[5] -shl 8)
    $sensorStatus = [int]$sample[2]
    $angle = [BitConverter]::ToSingle([byte[]]$sample[6..9], 0)
    $positionCounts = [BitConverter]::ToInt32([byte[]]$sample[18..21], 0)
    $positionDegrees = [BitConverter]::ToSingle([byte[]]$sample[22..25], 0)
    $electricalRaw = [BitConverter]::ToUInt16([byte[]]$sample[26..27], 0)
    $electricalDegrees = [BitConverter]::ToSingle([byte[]]$sample[28..31], 0)
    $polePairs = [int]$sample[32]
    $angleFlags = [int]$sample[33]
    $electricalZeroRaw = [BitConverter]::ToUInt16([byte[]]$sample[34..35], 0)
    $alignedElectricalRaw = [BitConverter]::ToUInt16([byte[]]$sample[36..37], 0)
    $alignedElectricalDegrees = [BitConverter]::ToSingle([byte[]]$sample[38..41], 0)
    $expectedElectricalRaw = ($raw * $polePairs) -band 0x0fff
    if ($polePairs -ne 11 -or $electricalRaw -ne $expectedElectricalRaw) {
        throw "Electrical angle mismatch raw=$raw polePairs=$polePairs expected=$expectedElectricalRaw actual=$electricalRaw"
    }
    $power = Invoke-CescRequestWithRetry $port 5 0 3 ([byte[]]@())
    $powerState = [int]$power[0]
    $powerFlags = [int]$power[1]
    $drvFaults = [int]$power[2] -bor ([int]$power[3] -shl 8)
    $busRaw = [int]$power[4] -bor ([int]$power[5] -shl 8)
    $busMv = [BitConverter]::ToUInt32([byte[]]$power[6..9], 0)
    $currentSequence = [BitConverter]::ToUInt32([byte[]]$power[10..13], 0)
    $phaseText = [System.Collections.Generic.List[string]]::new()
    for ($phase = 0; $phase -lt 3; ++$phase) {
        $base = 14 + 8 * $phase
        $phaseRaw = [BitConverter]::ToUInt16([byte[]]$power[$base..($base + 1)], 0)
        $phaseOffset = [BitConverter]::ToUInt16([byte[]]$power[($base + 2)..($base + 3)], 0)
        $phaseCentered = [BitConverter]::ToInt32([byte[]]$power[($base + 4)..($base + 7)], 0)
        $phaseText.Add("$phaseRaw/$phaseOffset/$phaseCentered")
    }
    if ($powerState -ne 2) {
        throw "Power stage not READY, state=$powerState flags=$powerFlags drvFaults=$drvFaults busRaw=$busRaw busMv=$busMv phases=$($phaseText -join ',')"
    }
    if (($powerFlags -band 0x03) -ne 0) {
        throw "Unsafe output flags: EN_GATE/PWM flags=$powerFlags"
    }
    if (($powerFlags -band 0x04) -ne 0 -or $drvFaults -ne 0) {
        throw "Power-stage fault flags=$powerFlags drvFaults=$drvFaults"
    }
    [uint16]$nextSequence = 4
    $motorResult = "not-run"
    $selectedMotorTests = [int]($RunEncoderAlignment.IsPresent) +
        [int]($RunCommissioningTest.IsPresent) +
        [int]($RunEncoderVoltageTest.IsPresent) +
        [int]($RunCurrentFocTest.IsPresent) +
        [int]($RunResistanceTest.IsPresent)
    if ($selectedMotorTests -gt 1) {
        throw "Select only one motor test per invocation"
    }
    if ($RunResistanceTest) {
        [void](Assert-CescResponse (Send-CescRequest $port 5 6 $nextSequence ([byte[]]@())) 5 6 $nextSequence)
        ++$nextSequence
        Start-Sleep -Milliseconds 2800
        $powerAfter = Assert-CescResponse (Send-CescRequest $port 5 0 $nextSequence ([byte[]]@())) 5 0 $nextSequence
        ++$nextSequence
        $stateAfter = [int]$powerAfter[0]
        $flagsAfter = [int]$powerAfter[1]
        $faultsAfter = [int]$powerAfter[2] -bor ([int]$powerAfter[3] -shl 8)
        $testState = [int]$powerAfter[38]
        $resistanceSamples = [BitConverter]::ToUInt32([byte[]]$powerAfter[184..187], 0)
        $resistanceIqSumMa = [BitConverter]::ToInt64([byte[]]$powerAfter[188..195], 0)
        $resistanceVqSumMv = [BitConverter]::ToInt64([byte[]]$powerAfter[196..203], 0)
        $resistanceIqAverageA = if ($resistanceSamples -gt 0) { $resistanceIqSumMa / (1000.0 * $resistanceSamples) } else { 0.0 }
        $resistanceVqAverageV = if ($resistanceSamples -gt 0) { $resistanceVqSumMv / (1000.0 * $resistanceSamples) } else { 0.0 }
        $resistanceOhms = if ([Math]::Abs($resistanceIqAverageA) -gt 0.02) { $resistanceVqAverageV / $resistanceIqAverageA } else { 0.0 }
        if ($stateAfter -ne 2 -or ($flagsAfter -band 0x03) -ne 0 -or
            $faultsAfter -ne 0 -or $testState -ne 2 -or
            $resistanceSamples -lt 1000 -or $resistanceOhms -le 0.0) {
            throw "Resistance measurement failed state=$stateAfter flags=$flagsAfter faults=$faultsAfter testState=$testState samples=$resistanceSamples iqA=$resistanceIqAverageA vqV=$resistanceVqAverageV resistanceOhm=$resistanceOhms"
        }
        $motorResult = "resistance samples=$resistanceSamples iqAvgA=$resistanceIqAverageA vqAvgV=$resistanceVqAverageV phaseResistanceOhm=$resistanceOhms"
    }
    if ($RunEncoderVoltageTest -or $RunCurrentFocTest) {
        [byte]$directionByte = if ($CommissioningDirection -gt 0) { 1 } else { 255 }
        [byte]$motorCommand = if ($RunCurrentFocTest) { 5 } else { 4 }
        [void](Assert-CescResponse (Send-CescRequest $port 5 $motorCommand $nextSequence ([byte[]]@($directionByte))) 5 $motorCommand $nextSequence)
        ++$nextSequence
        [double]$alignmentStartPosition = $positionDegrees
        [double]$trackingStartPosition = 0.0
        [bool]$trackingBaselineCaptured = $false
        [double]$minimumDisplacement = 0.0
        [double]$maximumDisplacement = 0.0
        [double]$lastTrackingPosition = 0.0
        [double]$finalDisplacement = 0.0
        [double]$maximumStepDegrees = 0.0
        [double]$totalTravelDegrees = 0.0
        [int]$reverseSteps = 0
        [int]$movingSteps = 0
        $voltageTestStopwatch = [System.Diagnostics.Stopwatch]::StartNew()
        $voltageTestDeadline = [DateTime]::UtcNow.AddMilliseconds($(if ($RunCurrentFocTest) { 12300 } else { 11800 }))
        while ([DateTime]::UtcNow -lt $voltageTestDeadline) {
            # Leave CPU/USB headroom while the motor-control ISR is active.
            Start-Sleep -Milliseconds $(if ($RunCurrentFocTest) { 25 } else { 10 })
            $trackedSample = $null
            for ($attempt = 0; $attempt -lt 3; ++$attempt) {
                try {
                    $trackedSample = Assert-CescResponse (Send-CescRequest $port 2 1 $nextSequence ([byte[]]@(0)) 100) 2 1 $nextSequence
                    break
                } catch {
                    if ($attempt -eq 2) { throw }
                    Start-Sleep -Milliseconds 20
                }
            }
            ++$nextSequence
            [double]$trackedPosition = [BitConverter]::ToSingle([byte[]]$trackedSample[22..25], 0)
            # Firmware aligns for 1.5 s before starting position tracking. Do not
            # mistake that alignment motion for commanded forward/reverse motion.
            if (-not $trackingBaselineCaptured -and
                $voltageTestStopwatch.ElapsedMilliseconds -ge 1700) {
                $trackingStartPosition = $trackedPosition
                $lastTrackingPosition = $trackedPosition
                $trackingBaselineCaptured = $true
            }
            if ($trackingBaselineCaptured) {
                [double]$displacement = $trackedPosition - $trackingStartPosition
                [double]$trackingStep = $trackedPosition - $lastTrackingPosition
                [double]$absoluteStep = [Math]::Abs($trackingStep)
                if ($absoluteStep -gt 0.04) {
                    ++$movingSteps
                    $totalTravelDegrees += $absoluteStep
                    if (($CommissioningDirection -gt 0 -and $trackingStep -lt -0.04) -or
                        ($CommissioningDirection -lt 0 -and $trackingStep -gt 0.04)) {
                        ++$reverseSteps
                    }
                }
                if ($absoluteStep -gt $maximumStepDegrees) { $maximumStepDegrees = $absoluteStep }
                $lastTrackingPosition = $trackedPosition
                $finalDisplacement = $displacement
                if ($displacement -lt $minimumDisplacement) { $minimumDisplacement = $displacement }
                if ($displacement -gt $maximumDisplacement) { $maximumDisplacement = $displacement }
            }
        }
        $voltageTestStopwatch.Stop()
        $powerAfter = Assert-CescResponse (Send-CescRequest $port 5 0 $nextSequence ([byte[]]@())) 5 0 $nextSequence
        ++$nextSequence
        $stateAfter = [int]$powerAfter[0]
        $flagsAfter = [int]$powerAfter[1]
        $faultsAfter = [int]$powerAfter[2] -bor ([int]$powerAfter[3] -shl 8)
        $testState = [int]$powerAfter[38]
        $testCurrentSamples = [BitConverter]::ToUInt32([byte[]]$powerAfter[40..43], 0)
        $testCurrentText = [System.Collections.Generic.List[string]]::new()
        for ($phase = 0; $phase -lt 3; ++$phase) {
            $statsBase = 44 + 12 * $phase
            $phaseSum = [BitConverter]::ToInt64([byte[]]$powerAfter[$statsBase..($statsBase + 7)], 0)
            $phaseMin = [BitConverter]::ToInt16([byte[]]$powerAfter[($statsBase + 8)..($statsBase + 9)], 0)
            $phaseMax = [BitConverter]::ToInt16([byte[]]$powerAfter[($statsBase + 10)..($statsBase + 11)], 0)
            $phaseAverage = if ($testCurrentSamples -gt 0) { $phaseSum / [double]$testCurrentSamples } else { 0.0 }
            $testCurrentText.Add("avg=$phaseAverage,min=$phaseMin,max=$phaseMax")
        }
        $balanceAbsSum = [BitConverter]::ToUInt64([byte[]]$powerAfter[80..87], 0)
        $balanceAbsMax = [BitConverter]::ToUInt16([byte[]]$powerAfter[88..89], 0)
        $balanceAbsAverage = if ($testCurrentSamples -gt 0) { $balanceAbsSum / [double]$testCurrentSamples } else { 0.0 }
        $v0Samples = [BitConverter]::ToUInt32([byte[]]$powerAfter[90..93], 0)
        $v7Samples = [BitConverter]::ToUInt32([byte[]]$powerAfter[94..97], 0)
        $reconstructed = @(
            [BitConverter]::ToUInt32([byte[]]$powerAfter[98..101], 0),
            [BitConverter]::ToUInt32([byte[]]$powerAfter[102..105], 0),
            [BitConverter]::ToUInt32([byte[]]$powerAfter[106..109], 0))
        $transformSamples = [BitConverter]::ToUInt32([byte[]]$powerAfter[110..113], 0)
        $idSumMa = [BitConverter]::ToInt64([byte[]]$powerAfter[114..121], 0)
        $iqSumMa = [BitConverter]::ToInt64([byte[]]$powerAfter[122..129], 0)
        $idMinMa = [BitConverter]::ToInt32([byte[]]$powerAfter[130..133], 0)
        $idMaxMa = [BitConverter]::ToInt32([byte[]]$powerAfter[134..137], 0)
        $iqMinMa = [BitConverter]::ToInt32([byte[]]$powerAfter[138..141], 0)
        $iqMaxMa = [BitConverter]::ToInt32([byte[]]$powerAfter[142..145], 0)
        $iqTargetSumMa = [BitConverter]::ToInt64([byte[]]$powerAfter[146..153], 0)
        $iqTargetMinMa = [BitConverter]::ToInt32([byte[]]$powerAfter[154..157], 0)
        $iqTargetMaxMa = [BitConverter]::ToInt32([byte[]]$powerAfter[158..161], 0)
        $voltageSaturatedSamples = [BitConverter]::ToUInt32([byte[]]$powerAfter[162..165], 0)
        $integralDSaturatedSamples = [BitConverter]::ToUInt32([byte[]]$powerAfter[166..169], 0)
        $integralQSaturatedSamples = [BitConverter]::ToUInt32([byte[]]$powerAfter[170..173], 0)
        $voltageRequestSumCounts = [BitConverter]::ToUInt64([byte[]]$powerAfter[174..181], 0)
        $voltageRequestMaxCounts = [BitConverter]::ToUInt16([byte[]]$powerAfter[182..183], 0)
        $idAverageA = if ($transformSamples -gt 0) { $idSumMa / (1000.0 * $transformSamples) } else { 0.0 }
        $iqAverageA = if ($transformSamples -gt 0) { $iqSumMa / (1000.0 * $transformSamples) } else { 0.0 }
        $iqTargetAverageA = if ($transformSamples -gt 0) { $iqTargetSumMa / (1000.0 * $transformSamples) } else { 0.0 }
        $voltageSaturationPercent = if ($transformSamples -gt 0) { 100.0 * $voltageSaturatedSamples / $transformSamples } else { 0.0 }
        $voltageRequestAverageCounts = if ($transformSamples -gt 0) { $voltageRequestSumCounts / [double]$transformSamples } else { 0.0 }
        if ($stateAfter -ne 2 -or ($flagsAfter -band 0x03) -ne 0 -or
            $faultsAfter -ne 0 -or $testState -ne 2) {
            throw "Encoder voltage test unsafe/incomplete state=$stateAfter flags=$flagsAfter faults=$faultsAfter testState=$testState minDelta=$minimumDisplacement maxDelta=$maximumDisplacement"
        }
        if ($testCurrentSamples -lt 1000) {
            throw "Too few synchronized current samples: $testCurrentSamples"
        }
        if ($transformSamples -lt 1000) {
            throw "Too few Clarke/Park samples: $transformSamples"
        }
        if (-not $trackingBaselineCaptured) {
            throw "Encoder voltage test did not capture a post-alignment tracking baseline"
        }
        if (($CommissioningDirection -gt 0 -and $maximumDisplacement -lt 10.0) -or
            ($CommissioningDirection -lt 0 -and $minimumDisplacement -gt -10.0)) {
            throw "Encoder voltage test did not track after alignment direction=$CommissioningDirection alignmentStart=$alignmentStartPosition trackingStart=$trackingStartPosition minDelta=$minimumDisplacement maxDelta=$maximumDisplacement idAvgA=$idAverageA iqAvgA=$iqAverageA"
        }
        $testLabel = if ($RunCurrentFocTest) { "position-current-foc" } else { "encoder-feedback" }
        $motorResult = "$testLabel direction=$CommissioningDirection trackingStart=$trackingStartPosition finalMechanicalDelta=$finalDisplacement minMechanicalDelta=$minimumDisplacement maxMechanicalDelta=$maximumDisplacement movingSteps=$movingSteps reverseSteps=$reverseSteps maxStepDegrees=$maximumStepDegrees totalTravelDegrees=$totalTravelDegrees currentSamples=$testCurrentSamples currents=[$($testCurrentText -join ';')] balanceAbsAvg=$balanceAbsAverage balanceAbsMax=$balanceAbsMax v0=$v0Samples v7=$v7Samples reconstructed=$($reconstructed -join '/') transformSamples=$transformSamples idAvgA=$idAverageA idRangeMa=$idMinMa..$idMaxMa iqAvgA=$iqAverageA iqRangeMa=$iqMinMa..$iqMaxMa iqTargetAvgA=$iqTargetAverageA iqTargetRangeMa=$iqTargetMinMa..$iqTargetMaxMa voltageSat=$voltageSaturatedSamples/$transformSamples($voltageSaturationPercent`%) integralSatD=$integralDSaturatedSamples integralSatQ=$integralQSaturatedSamples voltageRequestCountsAvg=$voltageRequestAverageCounts voltageRequestCountsMax=$voltageRequestMaxCounts"
    }
    if ($RunEncoderAlignment) {
        [void](Assert-CescResponse (Send-CescRequest $port 5 3 $nextSequence ([byte[]]@())) 5 3 $nextSequence)
        ++$nextSequence
        Start-Sleep -Milliseconds 2750
        $sampleHeld = Assert-CescResponse (Send-CescRequest $port 2 1 $nextSequence ([byte[]]@(0))) 2 1 $nextSequence
        ++$nextSequence
        Start-Sleep -Milliseconds 450
        $powerAfter = Assert-CescResponse (Send-CescRequest $port 5 0 $nextSequence ([byte[]]@())) 5 0 $nextSequence
        ++$nextSequence
        $sampleAfter = Assert-CescResponse (Send-CescRequest $port 2 1 $nextSequence ([byte[]]@(0))) 2 1 $nextSequence
        ++$nextSequence
        $stateAfter = [int]$powerAfter[0]
        $flagsAfter = [int]$powerAfter[1]
        $faultsAfter = [int]$powerAfter[2] -bor ([int]$powerAfter[3] -shl 8)
        $testState = [int]$powerAfter[38]
        $calibratedFlags = [int]$sampleAfter[33]
        $zeroAfter = [BitConverter]::ToUInt16([byte[]]$sampleAfter[34..35], 0)
        $alignedAfter = [BitConverter]::ToUInt16([byte[]]$sampleAfter[36..37], 0)
        $alignedDegreesAfter = [BitConverter]::ToSingle([byte[]]$sampleAfter[38..41], 0)
        $heldFlags = [int]$sampleHeld[33]
        $heldAlignedRaw = [BitConverter]::ToUInt16([byte[]]$sampleHeld[36..37], 0)
        $heldAlignedDegrees = [BitConverter]::ToSingle([byte[]]$sampleHeld[38..41], 0)
        [double]$heldMechanicalAngle = [BitConverter]::ToSingle([byte[]]$sampleHeld[6..9], 0)
        [double]$alignmentMechanicalDelta =
            [BitConverter]::ToSingle([byte[]]$sampleAfter[6..9], 0) - $angle
        while ($alignmentMechanicalDelta -gt 180.0) { $alignmentMechanicalDelta -= 360.0 }
        while ($alignmentMechanicalDelta -lt -180.0) { $alignmentMechanicalDelta += 360.0 }
        if ($stateAfter -ne 2 -or ($flagsAfter -band 0x03) -ne 0 -or
            $faultsAfter -ne 0 -or $testState -ne 2 -or
            ($calibratedFlags -band 1) -eq 0 -or ($heldFlags -band 1) -eq 0) {
            throw "Encoder alignment unsafe/incomplete state=$stateAfter flags=$flagsAfter faults=$faultsAfter testState=$testState angleFlags=$calibratedFlags heldFlags=$heldFlags"
        }
        [int]$targetError = $heldAlignedRaw - 1024
        while ($targetError -gt 2048) { $targetError -= 4096 }
        while ($targetError -lt -2048) { $targetError += 4096 }
        if ([Math]::Abs($targetError) -gt 64) {
            throw "Held electrical angle is not near 90 degrees: raw=$heldAlignedRaw degrees=$heldAlignedDegrees error=$targetError zero=$zeroAfter"
        }
        $motorResult = "encoder-vector-validation targetRaw=1024 zeroRaw=$zeroAfter heldRaw=$heldAlignedRaw heldDegrees=$heldAlignedDegrees targetError=$targetError heldMechanicalAngle=$heldMechanicalAngle releasedRaw=$alignedAfter releasedDegrees=$alignedDegreesAfter mechanicalDelta=$alignmentMechanicalDelta"
    }
    if ($RunCommissioningTest) {
        [byte]$directionByte = if ($CommissioningDirection -gt 0) { 1 } else { 255 }
        $testTimer = [System.Diagnostics.Stopwatch]::StartNew()
        [void](Assert-CescResponse (Send-CescRequest $port 5 1 $nextSequence ([byte[]]@($directionByte))) 5 1 $nextSequence)
        ++$nextSequence
        [double]$previousAngle = $angle
        [double]$cumulativeAngle = 0.0
        [double]$motionCumulativeAngle = 0.0
        [double]$motionPreviousAngle = 0.0
        [double]$motionStartAngle = [double]::NaN
        $testDeadline = [DateTime]::UtcNow.AddMilliseconds(2700)
        while ([DateTime]::UtcNow -lt $testDeadline) {
            Start-Sleep -Milliseconds 10
            $angleSample = Assert-CescResponse (Send-CescRequest $port 2 1 $nextSequence ([byte[]]@(0)) 75) 2 1 $nextSequence
            ++$nextSequence
            [double]$trackedAngle = [BitConverter]::ToSingle([byte[]]$angleSample[6..9], 0)
            [double]$increment = $trackedAngle - $previousAngle
            while ($increment -gt 180.0) { $increment -= 360.0 }
            while ($increment -lt -180.0) { $increment += 360.0 }
            $cumulativeAngle += $increment
            $previousAngle = $trackedAngle
            if ($testTimer.ElapsedMilliseconds -ge 500) {
                if ([double]::IsNaN($motionStartAngle)) {
                    $motionStartAngle = $trackedAngle
                    $motionPreviousAngle = $trackedAngle
                } else {
                    [double]$motionIncrement = $trackedAngle - $motionPreviousAngle
                    while ($motionIncrement -gt 180.0) { $motionIncrement -= 360.0 }
                    while ($motionIncrement -lt -180.0) { $motionIncrement += 360.0 }
                    $motionCumulativeAngle += $motionIncrement
                    $motionPreviousAngle = $trackedAngle
                }
            }
        }
        $powerAfter = Assert-CescResponse (Send-CescRequest $port 5 0 $nextSequence ([byte[]]@())) 5 0 $nextSequence
        ++$nextSequence
        $sampleAfter = Assert-CescResponse (Send-CescRequest $port 2 1 $nextSequence ([byte[]]@(0))) 2 1 $nextSequence
        ++$nextSequence
        $stateAfter = [int]$powerAfter[0]
        $flagsAfter = [int]$powerAfter[1]
        $faultsAfter = [int]$powerAfter[2] -bor ([int]$powerAfter[3] -shl 8)
        $testState = [int]$powerAfter[38]
        $testSteps = [int]$powerAfter[39]
        $angleAfter = [BitConverter]::ToSingle([byte[]]$sampleAfter[6..9], 0)
        $angleDelta = $angleAfter - $angle
        while ($angleDelta -gt 180.0) { $angleDelta -= 360.0 }
        while ($angleDelta -lt -180.0) { $angleDelta += 360.0 }
        if ($stateAfter -ne 2 -or ($flagsAfter -band 0x03) -ne 0 -or
            $faultsAfter -ne 0 -or $testState -ne 2 -or $testSteps -ne 30) {
            throw "Commissioning test unsafe/incomplete state=$stateAfter flags=$flagsAfter faults=$faultsAfter testState=$testState steps=$testSteps"
        }
        $motorResult = "completed direction=$CommissioningDirection steps=$testSteps angleAfter=$angleAfter endpointDelta=$angleDelta totalDelta=$cumulativeAngle motionStart=$motionStartAngle motionDelta=$motionCumulativeAngle"
    }
    [byte[]]$subscription = @(0x10,0x27,0,0,1,3,0,0,1,0,2,0)
    $reply = Assert-CescResponse (Send-CescRequest $port 3 1 $nextSequence $subscription) 3 1 $nextSequence
    ++$nextSequence
    $streamId = [int]$reply[0] -bor ([int]$reply[1] -shl 8)
    $streamBytes = Read-Available $port $StreamDurationMs
    $streamFrames = 0
    for ($offset = 0; $offset + 12 -le $streamBytes.Length;) {
        if ($streamBytes[$offset] -ne 0x43 -or $streamBytes[$offset + 1] -ne 0x45) {
            ++$offset
            continue
        }
        $length = [int]$streamBytes[$offset + 8] -bor
            ([int]$streamBytes[$offset + 9] -shl 8)
        $total = 12 + $length
        if ($offset + $total -gt $streamBytes.Length) { break }
        if ($streamBytes[$offset + 3] -eq 3 -and
            $streamBytes[$offset + 4] -eq 3 -and
            $streamBytes[$offset + 5] -eq 0x80) { ++$streamFrames }
        $offset += $total
    }
    [void](Send-CescRequest $port 3 2 $nextSequence ([byte[]]@(
        ($streamId -band 0xff), ($streamId -shr 8))))
    if ($streamFrames -lt 5) { throw "Expected telemetry frames, got $streamFrames" }
    Write-Output "PASS port=$PortName sensorStatus=$sensorStatus angleRaw=$raw angle=$angle positionCounts=$positionCounts positionDegrees=$positionDegrees electricalRaw=$electricalRaw electricalDegrees=$electricalDegrees polePairs=$polePairs angleFlags=$angleFlags electricalZeroRaw=$electricalZeroRaw alignedElectricalRaw=$alignedElectricalRaw alignedElectricalDegrees=$alignedElectricalDegrees powerState=$powerState flags=$powerFlags drvFaults=$drvFaults busRaw=$busRaw busMv=$busMv currentSeq=$currentSequence phases(raw/offset/centered)=$($phaseText -join ',') motorTest=[$motorResult] streamId=$streamId frames=$streamFrames"
} finally {
    if ($port.IsOpen) {
        try { [void](Send-CescRequest $port 5 2 0xffff ([byte[]]@()) 20) } catch {}
    }
    if ($port.IsOpen) { $port.Close() }
    $port.Dispose()
}
