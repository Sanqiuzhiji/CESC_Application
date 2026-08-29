param(
    [string]$PortName = "COM15",
    [int]$BaudRate = 115200,
    [int]$StreamDurationMs = 500,
    [switch]$RunCommissioningTest,
    [switch]$RunEncoderAlignment,
    [switch]$RunEncoderVoltageTest,
    [switch]$RunCurrentFocTest,
    [switch]$RunIqCurrentTest,
    [switch]$RunSpeedTest,
    [switch]$RunPositionTest,
    [switch]$RunPositionProfileTest,
    [switch]$RunHapticTest,
    [switch]$RunTorqueTest,
    [switch]$RunResistanceTest,
    [switch]$RunInductanceTest,
    [switch]$RunFluxTest,
    [switch]$RunConfigurationTest,
    [switch]$ReadConfigurationOnly,
    [switch]$RestoreAndSaveDefaults,
    [ValidateRange(-300, 300)]
    [int]$IqCurrentMa = 50,
    [ValidateRange(500, 5000)]
    [int]$IqDurationMs = 800,
    [switch]$VerifyIqWatchdog,
    [ValidateRange(-3000000, 3000000)]
    [int]$SpeedTargetMdps = 2000,
    [ValidateRange(-500, 500)]
    [int]$SpeedTargetRpm = 0,
    [ValidateRange(20, 300)]
    [int]$SpeedSamples = 20,
    [ValidateRange(1, 100)]
    [int]$SpeedLogEvery = 1,
    [ValidateRange(-3600000, 3600000)]
    [int]$RelativePositionTargetMdeg = 5000,
    [ValidateRange(50, 3000000)]
    [int]$PositionProfileSpeedMdps = 10000,
    [ValidateRange(100, 600000)]
    [int]$PositionProfileAccelerationMdps2 = 10000,
    [ValidateRange(100, 600000)]
    [int]$PositionProfileDecelerationMdps2 = 10000,
    [ValidateRange(100, 360000)]
    [int]$HapticDetentSpacingMdeg = 15000,
    [ValidateRange(0, 300)]
    [int]$HapticDetentStrengthMa = 100,
    [ValidateRange(0, 100)]
    [int]$HapticDampingMaPerDps = 10,
    [ValidateRange(-69, 69)]
    [int]$TorqueTargetMnm = 12,
    [ValidateSet(-1, 1)]
    [int]$CommissioningDirection = 1
)

$ErrorActionPreference = "Stop"

if ($SpeedTargetRpm -ne 0) {
    $SpeedTargetMdps = $SpeedTargetRpm * 6000
}

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
            if ($attempt -eq 2) {
                throw "CESC request failed service=$Service command=$Command sequence=$Sequence`: $($_.Exception.Message)"
            }
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
    if ($RestoreAndSaveDefaults) {
        [void](Invoke-CescRequestWithRetry $port 4 5 2 ([byte[]]@()))
        [void](Invoke-CescRequestWithRetry $port 4 3 3 ([byte[]]@()))
        $defaultStatus = $null
        [uint16]$defaultSequence = 4
        for ($poll = 0; $poll -lt 20; ++$poll) {
            Start-Sleep -Milliseconds 250
            $defaultStatus = Invoke-CescRequestWithRetry $port 4 6 $defaultSequence ([byte[]]@())
            ++$defaultSequence
            if ($defaultStatus.Length -ge 10 -and $defaultStatus[8] -eq 0) { break }
        }
        $defaultConfig = Invoke-CescRequestWithRetry $port 4 1 $defaultSequence ([byte[]]@())
        $defaultTimeout = [BitConverter]::ToUInt32([byte[]]$defaultConfig[37..40], 0)
        if ($defaultStatus.Length -lt 10 -or $defaultStatus[0] -lt 1 -or
            $defaultStatus[1] -ne 0 -or $defaultStatus[8] -ne 0 -or
            $defaultStatus[9] -ne 1 -or $defaultTimeout -ne 1000) {
            throw "Restore/save defaults failed timeout=$defaultTimeout"
        }
        Write-Output "PASS configuration-defaults timeoutMs=$defaultTimeout source=$($defaultStatus[0]) sequence=$([BitConverter]::ToUInt32([byte[]]$defaultStatus[4..7], 0))"
        return
    }
    if ($ReadConfigurationOnly) {
        $current = Invoke-CescRequestWithRetry $port 4 1 2 ([byte[]]@())
        $configStatus = Invoke-CescRequestWithRetry $port 4 6 3 ([byte[]]@())
        $timeout = [BitConverter]::ToUInt32([byte[]]$current[37..40], 0)
        $storedSequence = [BitConverter]::ToUInt32([byte[]]$configStatus[4..7], 0)
        Write-Output "PASS configuration-read timeoutMs=$timeout source=$($configStatus[0]) dirty=$($configStatus[1]) sequence=$storedSequence slotA=$($configStatus[2]) slotB=$($configStatus[3])"
        return
    }
    if ($RunConfigurationTest) {
        [uint16]$configSequence = 2
        $info = Invoke-CescRequestWithRetry $port 4 0 $configSequence ([byte[]]@())
        ++$configSequence
        if ($info.Length -ne 8 -or [BitConverter]::ToUInt16([byte[]]$info[0..1], 0) -ne 1 -or
            [BitConverter]::ToUInt16([byte[]]$info[2..3], 0) -ne 41) {
            throw "Unexpected configuration info"
        }
        [byte[]]$before = Invoke-CescRequestWithRetry $port 4 1 $configSequence ([byte[]]@())
        ++$configSequence
        if ($before.Length -ne 41) { throw "Configuration payload length is not 41" }
        [uint32]$oldTimeout = [BitConverter]::ToUInt32([byte[]]$before[37..40], 0)
        [uint32]$newTimeout = if ($oldTimeout -eq 1100) { 1200 } else { 1100 }
        [byte[]]$updated = [byte[]]$before.Clone()
        [BitConverter]::GetBytes($newTimeout).CopyTo($updated, 37)
        [void](Invoke-CescRequestWithRetry $port 4 2 $configSequence $updated)
        ++$configSequence
        $dirty = Invoke-CescRequestWithRetry $port 4 6 $configSequence ([byte[]]@())
        ++$configSequence
        if ($dirty.Length -lt 8 -or $dirty[1] -ne 1) { throw "Staged configuration is not dirty" }
        [void](Invoke-CescRequestWithRetry $port 4 3 $configSequence ([byte[]]@()))
        ++$configSequence
        # A sector erase stalls flash-resident USB interrupt code briefly.
        # Let CDC re-arm its OUT endpoint before issuing the next request.
        $saved = $null
        for ($poll = 0; $poll -lt 20; ++$poll) {
            Start-Sleep -Milliseconds 250
            $saved = Invoke-CescRequestWithRetry $port 4 6 $configSequence ([byte[]]@())
            ++$configSequence
            if ($saved.Length -ge 10 -and $saved[8] -eq 0) { break }
        }
        [byte[]]$after = Invoke-CescRequestWithRetry $port 4 1 $configSequence ([byte[]]@())
        $readTimeout = [BitConverter]::ToUInt32([byte[]]$after[37..40], 0)
        if ($saved.Length -lt 10 -or $saved[0] -lt 1 -or $saved[0] -gt 2 -or
            $saved[1] -ne 0 -or $saved[8] -ne 0 -or $saved[9] -ne 1 -or
            $readTimeout -ne $newTimeout) {
            throw "Configuration save verification failed source=$($saved[0]) dirty=$($saved[1]) timeout=$readTimeout"
        }
        $sequenceSaved = [BitConverter]::ToUInt32([byte[]]$saved[4..7], 0)
        Write-Output "PASS configuration oldTimeoutMs=$oldTimeout newTimeoutMs=$newTimeout source=$($saved[0]) sequence=$sequenceSaved slotA=$($saved[2]) slotB=$($saved[3])"
        return
    }
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
    $cpuFaultText = "none"
    if ($power.Length -ge 383) {
        $faultMagic = [BitConverter]::ToUInt32([byte[]]$power[351..354], 0)
        if ($faultMagic -eq 0x43534654) {
            $faultType = [BitConverter]::ToUInt32([byte[]]$power[355..358], 0)
            $faultCfsr = [BitConverter]::ToUInt32([byte[]]$power[359..362], 0)
            $faultHfsr = [BitConverter]::ToUInt32([byte[]]$power[363..366], 0)
            $faultPc = [BitConverter]::ToUInt32([byte[]]$power[375..378], 0)
            $faultLr = [BitConverter]::ToUInt32([byte[]]$power[379..382], 0)
            $cpuFaultText = "type=$faultType cfsr=0x$($faultCfsr.ToString('X8')) hfsr=0x$($faultHfsr.ToString('X8')) pc=0x$($faultPc.ToString('X8')) lr=0x$($faultLr.ToString('X8'))"
        }
    }
    $phaseText = [System.Collections.Generic.List[string]]::new()
    for ($phase = 0; $phase -lt 3; ++$phase) {
        $base = 14 + 8 * $phase
        $phaseRaw = [BitConverter]::ToUInt16([byte[]]$power[$base..($base + 1)], 0)
        $phaseOffset = [BitConverter]::ToUInt16([byte[]]$power[($base + 2)..($base + 3)], 0)
        $phaseCentered = [BitConverter]::ToInt32([byte[]]$power[($base + 4)..($base + 7)], 0)
        $phaseText.Add("$phaseRaw/$phaseOffset/$phaseCentered")
    }
    $phaseExtremaText = [System.Collections.Generic.List[string]]::new()
    if ($power.Length -ge 80) {
        for ($phase = 0; $phase -lt 3; ++$phase) {
            $base = 44 + 12 * $phase
            $phaseMin = [BitConverter]::ToInt16(
                [byte[]]$power[($base + 8)..($base + 9)], 0)
            $phaseMax = [BitConverter]::ToInt16(
                [byte[]]$power[($base + 10)..($base + 11)], 0)
            $phaseExtremaText.Add("$phaseMin..$phaseMax")
        }
    }
    if ($powerState -ne 2) {
        throw "Power stage not READY, state=$powerState flags=$powerFlags drvFaults=$drvFaults busRaw=$busRaw busMv=$busMv phases=$($phaseText -join ',') extrema=$($phaseExtremaText -join ',')"
    }
    if (($powerFlags -band 0x03) -ne 0) {
        throw "Unsafe output flags: EN_GATE/PWM flags=$powerFlags"
    }
    if (($powerFlags -band 0x04) -ne 0 -or $drvFaults -ne 0) {
        throw "Power-stage fault flags=$powerFlags drvFaults=$drvFaults"
    }
    [uint16]$nextSequence = 4
    $commStatsBefore = Invoke-CescRequestWithRetry $port 0 4 $nextSequence ([byte[]]@())
    ++$nextSequence
    if ($commStatsBefore.Length -lt 52) {
        throw "Communication statistics response lacks USB diagnostics"
    }
    [uint32]$usbMutexBefore = [BitConverter]::ToUInt32([byte[]]$commStatsBefore[36..39], 0)
    [uint32]$usbSubmitBefore = [BitConverter]::ToUInt32([byte[]]$commStatsBefore[40..43], 0)
    [uint32]$usbCompleteBefore = [BitConverter]::ToUInt32([byte[]]$commStatsBefore[44..47], 0)
    [uint32]$usbDropBefore = [BitConverter]::ToUInt32([byte[]]$commStatsBefore[48..51], 0)
    $motorResult = "not-run"
    $selectedMotorTests = [int]($RunEncoderAlignment.IsPresent) +
        [int]($RunCommissioningTest.IsPresent) +
        [int]($RunEncoderVoltageTest.IsPresent) +
        [int]($RunCurrentFocTest.IsPresent) +
        [int]($RunIqCurrentTest.IsPresent) +
        [int]($RunSpeedTest.IsPresent) +
        [int]($RunPositionTest.IsPresent) +
        [int]($RunPositionProfileTest.IsPresent) +
        [int]($RunHapticTest.IsPresent) +
        [int]($RunTorqueTest.IsPresent) +
        [int]($RunResistanceTest.IsPresent) +
        [int]($RunInductanceTest.IsPresent) +
        [int]($RunFluxTest.IsPresent)
    if ($selectedMotorTests -gt 1) {
        throw "Select only one motor test per invocation"
    }
    if ($RunIqCurrentTest) {
        if ([Math]::Abs($IqCurrentMa) -lt 10) {
            throw "IqCurrentMa must have an absolute value of at least 10 mA"
        }
        [byte[]]$iqPayload = [BitConverter]::GetBytes([int]$IqCurrentMa)
        [int]$idMa = 0
        [int]$iqMa = 0
        $iqSampleCount = [Math]::Max(1, [int]($IqDurationMs / 100))
        [long]$idSumMa = 0
        [long]$iqSumMa = 0
        for ($sampleIndex = 0; $sampleIndex -lt $iqSampleCount; ++$sampleIndex) {
            [void](Assert-CescResponse (Send-CescRequest $port 5 9 $nextSequence $iqPayload) 5 9 $nextSequence)
            ++$nextSequence
            Start-Sleep -Milliseconds 100
            $powerAfter = Invoke-CescRequestWithRetry $port 5 0 $nextSequence ([byte[]]@())
            ++$nextSequence
            if ($powerAfter.Length -lt 322) {
                throw "Power-stage response lacks current-control diagnostics"
            }
            $stateNow = [int]$powerAfter[0]
            $flagsNow = [int]$powerAfter[1]
            $faultsNow = [int]$powerAfter[2] -bor ([int]$powerAfter[3] -shl 8)
            $controlMode = [int]$powerAfter[305]
            $idMa = [BitConverter]::ToInt32([byte[]]$powerAfter[306..309], 0)
            $iqMa = [BitConverter]::ToInt32([byte[]]$powerAfter[310..313], 0)
            $iqTargetMa = [BitConverter]::ToInt32([byte[]]$powerAfter[314..317], 0)
            $timeoutRemainingMs = [BitConverter]::ToUInt32([byte[]]$powerAfter[318..321], 0)
            $idSumMa += $idMa
            $iqSumMa += $iqMa
            if ($stateNow -ne 3 -or ($flagsNow -band 0x03) -ne 0x03 -or
                $faultsNow -ne 0 -or $controlMode -ne 1 -or
                $iqTargetMa -ne $IqCurrentMa -or $timeoutRemainingMs -eq 0) {
                throw "Iq control invalid state=$stateNow flags=$flagsNow faults=$faultsNow mode=$controlMode Id=$idMa Iq=$iqMa target=$iqTargetMa timeout=$timeoutRemainingMs"
            }
            Write-Output "Iq control sample=$sampleIndex Id=${idMa}mA Iq=${iqMa}mA target=${iqTargetMa}mA timeout=${timeoutRemainingMs}ms"
        }
        $transformSamples = [BitConverter]::ToUInt32([byte[]]$powerAfter[110..113], 0)
        $idTransformSumMa = [BitConverter]::ToInt64([byte[]]$powerAfter[114..121], 0)
        $iqTransformSumMa = [BitConverter]::ToInt64([byte[]]$powerAfter[122..129], 0)
        $idMinMa = [BitConverter]::ToInt32([byte[]]$powerAfter[130..133], 0)
        $idMaxMa = [BitConverter]::ToInt32([byte[]]$powerAfter[134..137], 0)
        $iqMinMa = [BitConverter]::ToInt32([byte[]]$powerAfter[138..141], 0)
        $iqMaxMa = [BitConverter]::ToInt32([byte[]]$powerAfter[142..145], 0)
        $voltageSaturatedSamples = [BitConverter]::ToUInt32([byte[]]$powerAfter[162..165], 0)
        $voltageRequestSumCounts = [BitConverter]::ToUInt64([byte[]]$powerAfter[174..181], 0)
        $voltageRequestMaxCounts = [BitConverter]::ToUInt16([byte[]]$powerAfter[182..183], 0)
        $idTransformAverageMa = if ($transformSamples -gt 0) { [Math]::Round($idTransformSumMa / [double]$transformSamples, 1) } else { 0.0 }
        $iqTransformAverageMa = if ($transformSamples -gt 0) { [Math]::Round($iqTransformSumMa / [double]$transformSamples, 1) } else { 0.0 }
        $voltageRequestAverageCounts = if ($transformSamples -gt 0) { [Math]::Round($voltageRequestSumCounts / [double]$transformSamples, 1) } else { 0.0 }
        Write-Output "Iq aggregate transforms=$transformSamples IdAvg=${idTransformAverageMa}mA IqAvg=${iqTransformAverageMa}mA IdRange=$idMinMa..$idMaxMa IqRange=$iqMinMa..$iqMaxMa voltageRequestAvg=$voltageRequestAverageCounts max=$voltageRequestMaxCounts saturated=$voltageSaturatedSamples"
        if ($VerifyIqWatchdog) {
            Start-Sleep -Milliseconds 700
        } else {
            [void](Assert-CescResponse (Send-CescRequest $port 5 2 $nextSequence ([byte[]]@())) 5 2 $nextSequence)
            ++$nextSequence
            Start-Sleep -Milliseconds 100
        }
        $powerAfter = Invoke-CescRequestWithRetry $port 5 0 $nextSequence ([byte[]]@())
        ++$nextSequence
        $sampleAfter = Invoke-CescRequestWithRetry $port 2 1 $nextSequence ([byte[]]@(0))
        ++$nextSequence
        $stateAfter = [int]$powerAfter[0]
        $flagsAfter = [int]$powerAfter[1]
        $controlModeAfter = [int]$powerAfter[305]
        if ($stateAfter -ne 2 -or ($flagsAfter -band 0x03) -ne 0 -or
            $controlModeAfter -ne 0) {
            $releaseKind = if ($VerifyIqWatchdog) { "watchdog" } else { "STOP" }
            throw "$releaseKind did not release Iq control state=$stateAfter flags=$flagsAfter mode=$controlModeAfter"
        }
        if ($VerifyIqWatchdog) {
            [byte[]]$latchedResponse = Send-CescRequest $port 5 9 $nextSequence $iqPayload
            if ($latchedResponse.Length -lt 14 -or
                $latchedResponse[0] -ne 0x43 -or $latchedResponse[1] -ne 0x45) {
                throw "Missing response while verifying watchdog restart latch"
            }
            [int]$latchedStatus = [int]$latchedResponse[10] -bor
                ([int]$latchedResponse[11] -shl 8)
            if ($latchedStatus -ne 5) {
                throw "Watchdog latch accepted restart without STOP status=$latchedStatus"
            }
            ++$nextSequence
            [void](Invoke-CescRequestWithRetry $port 5 2 $nextSequence ([byte[]]@()))
            ++$nextSequence
            [void](Invoke-CescRequestWithRetry $port 5 9 $nextSequence $iqPayload)
            ++$nextSequence
            [void](Invoke-CescRequestWithRetry $port 5 2 $nextSequence ([byte[]]@()))
            ++$nextSequence
        }
        $positionAfterDegrees = [BitConverter]::ToSingle([byte[]]$sampleAfter[22..25], 0)
        $positionDeltaDegrees = [Math]::Round($positionAfterDegrees - $positionDegrees, 3)
        $idAverageMa = [Math]::Round($idSumMa / [double]$iqSampleCount, 1)
        $iqAverageMa = [Math]::Round($iqSumMa / [double]$iqSampleCount, 1)
        $releaseKind = if ($VerifyIqWatchdog) { "watchdog" } else { "stop" }
        $restartLatchResult = if ($VerifyIqWatchdog) { " restartLatchVerified=true" } else { "" }
        $motorResult = "iq-current targetMa=$IqCurrentMa durationMs=$IqDurationMs polledIdAverageMa=$idAverageMa polledIqAverageMa=$iqAverageMa transformIdAverageMa=$idTransformAverageMa transformIqAverageMa=$iqTransformAverageMa iqRangeMa=$iqMinMa..$iqMaxMa voltageRequestCountsAvg=$voltageRequestAverageCounts voltageRequestCountsMax=$voltageRequestMaxCounts positionDeltaDegrees=$positionDeltaDegrees ${releaseKind}Verified=true$restartLatchResult"
    }
    if ($RunTorqueTest) {
        if ([Math]::Abs($TorqueTargetMnm) -lt 3) {
            throw "TorqueTargetMnm must have an absolute value of at least 3 mN*m"
        }
        [byte[]]$torquePayload = [BitConverter]::GetBytes([int]$TorqueTargetMnm)
        [int]$expectedIqMa = [Math]::Round($TorqueTargetMnm * 1000.0 / 230.0)
        for ($sampleIndex = 0; $sampleIndex -lt 8; ++$sampleIndex) {
            [void](Assert-CescResponse (Send-CescRequest $port 5 14 $nextSequence $torquePayload) 5 14 $nextSequence)
            ++$nextSequence
            Start-Sleep -Milliseconds 100
            $powerAfter = Invoke-CescRequestWithRetry $port 5 0 $nextSequence ([byte[]]@())
            ++$nextSequence
            $stateNow = [int]$powerAfter[0]
            $flagsNow = [int]$powerAfter[1]
            $faultsNow = [int]$powerAfter[2] -bor ([int]$powerAfter[3] -shl 8)
            $controlMode = [int]$powerAfter[305]
            $torqueIqTargetMa = [BitConverter]::ToInt32([byte[]]$powerAfter[314..317], 0)
            if ($stateNow -ne 3 -or ($flagsNow -band 0x03) -ne 0x03 -or
                $faultsNow -ne 0 -or $controlMode -ne 1 -or
                [Math]::Abs($torqueIqTargetMa - $expectedIqMa) -gt 1) {
                throw "Torque control invalid state=$stateNow flags=$flagsNow faults=$faultsNow mode=$controlMode expectedIq=$expectedIqMa iqTarget=$torqueIqTargetMa"
            }
            Write-Output "Torque sample=$sampleIndex target=${TorqueTargetMnm}mNm estimatedIq=${torqueIqTargetMa}mA"
        }
        [void](Assert-CescResponse (Send-CescRequest $port 5 2 $nextSequence ([byte[]]@())) 5 2 $nextSequence)
        ++$nextSequence
        Start-Sleep -Milliseconds 100
        $powerAfter = Invoke-CescRequestWithRetry $port 5 0 $nextSequence ([byte[]]@())
        ++$nextSequence
        if ([int]$powerAfter[0] -ne 2 -or ([int]$powerAfter[1] -band 0x03) -ne 0 -or
            [int]$powerAfter[305] -ne 0) { throw "STOP did not release torque control" }
        $motorResult = "torque targetMnm=$TorqueTargetMnm estimatedIqTargetMa=$expectedIqMa KtNmPerA=0.23 stopVerified=true"
    }
    if ($RunSpeedTest) {
        if ([Math]::Abs($SpeedTargetMdps) -lt 50) {
            throw "SpeedTargetMdps must have an absolute value of at least 50"
        }
        [byte[]]$speedPayload = [BitConverter]::GetBytes([int]$SpeedTargetMdps)
        [double]$speedPositionStart = $positionDegrees
        [long]$actualSpeedSumMdps = 0
        [double]$expectedRampSeconds = if ([Math]::Abs($SpeedTargetMdps) -ge 120000) {
            120000.0 / 20000.0 +
                ([Math]::Abs($SpeedTargetMdps) - 120000.0) / 120000.0
        } else {
            [Math]::Abs($SpeedTargetMdps) / 20000.0
        }
        [int]$minimumRampSamples = [Math]::Ceiling(
            ($expectedRampSeconds + 2.0) * 2.5)
        [int]$speedSamples = [Math]::Max($SpeedSamples, $minimumRampSamples)
        [int]$steadySampleStart = [Math]::Max(0, $speedSamples - 10)
        [long]$steadySpeedSumMdps = 0
        [int]$steadySpeedSamples = 0
        for ($sampleIndex = 0; $sampleIndex -lt $speedSamples; ++$sampleIndex) {
            [void](Invoke-CescRequestWithRetry $port 5 10 $nextSequence $speedPayload)
            ++$nextSequence
            Start-Sleep -Milliseconds 100
            $controlAfter = Invoke-CescRequestWithRetry $port 5 15 $nextSequence ([byte[]]@())
            ++$nextSequence
            if ($controlAfter.Length -lt 46) { throw "Control-status response is truncated" }
            $stateNow = [int]$controlAfter[0]
            $flagsNow = [int]$controlAfter[1]
            $faultsNow = [int]$controlAfter[2] -bor ([int]$controlAfter[3] -shl 8)
            $controlMode = [int]$controlAfter[8]
            $speedCurrentFoc = [int]$controlAfter[9]
            $speedTargetNow = [BitConverter]::ToInt32([byte[]]$controlAfter[10..13], 0)
            $speedReferenceNow = [BitConverter]::ToInt32([byte[]]$controlAfter[14..17], 0)
            $speedActualNow = [BitConverter]::ToInt32([byte[]]$controlAfter[18..21], 0)
            $iqTargetMa = [BitConverter]::ToInt32([byte[]]$controlAfter[30..33], 0)
            $iqActualMa = [BitConverter]::ToInt32([byte[]]$controlAfter[26..29], 0)
            $predictedElectricalRaw = [BitConverter]::ToUInt16([byte[]]$controlAfter[34..35], 0)
            $encoderSampleAgeMs = [BitConverter]::ToUInt32([byte[]]$controlAfter[36..39], 0)
            $predictionErrorRaw = [BitConverter]::ToInt16([byte[]]$controlAfter[40..41], 0)
            if ($controlAfter.Length -ge 78) {
                $runtimeFaultMagic = [BitConverter]::ToUInt32([byte[]]$controlAfter[46..49], 0)
                if ($runtimeFaultMagic -eq 0x43534654) {
                    throw "CPU fault record present during speed test"
                }
            }
            $voltageQCounts = if ($controlAfter.Length -ge 87) {
                [BitConverter]::ToInt32([byte[]]$controlAfter[78..81], 0)
            } else { 0 }
            $voltageLimitCounts = if ($controlAfter.Length -ge 87) {
                [BitConverter]::ToInt32([byte[]]$controlAfter[82..85], 0)
            } else { 0 }
            $voltageCurrentLimited = if ($controlAfter.Length -ge 87) {
                [int]$controlAfter[86]
            } else { 0 }
            $observerPhaseRaw = if ($controlAfter.Length -ge 96) {
                [BitConverter]::ToUInt16([byte[]]$controlAfter[87..88], 0)
            } else { 0 }
            $observerErrorRaw = if ($controlAfter.Length -ge 96) {
                [BitConverter]::ToInt16([byte[]]$controlAfter[89..90], 0)
            } else { 0 }
            $observerErpm = if ($controlAfter.Length -ge 96) {
                [BitConverter]::ToInt32([byte[]]$controlAfter[91..94], 0)
            } else { 0 }
            $observerUsingEncoder = if ($controlAfter.Length -ge 96) {
                [int]$controlAfter[95]
            } else { 1 }
            $actualSpeedSumMdps += $speedActualNow
            if ($sampleIndex -ge $steadySampleStart) {
                $steadySpeedSumMdps += $speedActualNow
                ++$steadySpeedSamples
            }
            if ($stateNow -ne 3 -or ($flagsNow -band 0x03) -ne 0x03 -or
                $faultsNow -ne 0 -or $controlMode -ne 2 -or
                $speedTargetNow -ne $SpeedTargetMdps -or
                [Math]::Abs($iqTargetMa) -gt 300) {
                throw "Speed control invalid state=$stateNow flags=$flagsNow faults=$faultsNow mode=$controlMode target=$speedTargetNow actual=$speedActualNow iqTarget=$iqTargetMa"
            }
            if (($sampleIndex % $SpeedLogEvery) -eq 0 -or
                $sampleIndex -eq ($speedSamples - 1)) {
                Write-Output "Speed sample=$sampleIndex target=${speedTargetNow}mdps reference=${speedReferenceNow}mdps actual=${speedActualNow}mdps Iq=${iqActualMa}mA Vq=$voltageQCounts/$voltageLimitCounts currentLimited=$voltageCurrentLimited iqTarget=${iqTargetMa}mA currentFoc=$speedCurrentFoc predictedElectricalRaw=$predictedElectricalRaw predictionErrorRaw=$predictionErrorRaw encoderAge=${encoderSampleAgeMs}ms observerRaw=$observerPhaseRaw observerErrorRaw=$observerErrorRaw observerErpm=$observerErpm observerEncoder=$observerUsingEncoder"
            }
        }
        $powerAfter = Invoke-CescRequestWithRetry $port 5 0 $nextSequence ([byte[]]@())
        ++$nextSequence
        $speedTransformSamples = [BitConverter]::ToUInt32([byte[]]$powerAfter[110..113], 0)
        $speedIdSumMa = [BitConverter]::ToInt64([byte[]]$powerAfter[114..121], 0)
        $speedIqSumMa = [BitConverter]::ToInt64([byte[]]$powerAfter[122..129], 0)
        $speedIqTargetSumMa = [BitConverter]::ToInt64([byte[]]$powerAfter[146..153], 0)
        $speedVoltageSaturated = [BitConverter]::ToUInt32([byte[]]$powerAfter[162..165], 0)
        $speedIdAverageMa = if ($speedTransformSamples -gt 0) { [Math]::Round($speedIdSumMa / [double]$speedTransformSamples, 1) } else { 0.0 }
        $speedIqAverageMa = if ($speedTransformSamples -gt 0) { [Math]::Round($speedIqSumMa / [double]$speedTransformSamples, 1) } else { 0.0 }
        $speedIqTargetAverageMa = if ($speedTransformSamples -gt 0) { [Math]::Round($speedIqTargetSumMa / [double]$speedTransformSamples, 1) } else { 0.0 }
        Write-Output "Speed aggregate transforms=$speedTransformSamples IdAvg=${speedIdAverageMa}mA IqAvg=${speedIqAverageMa}mA IqTargetAvg=${speedIqTargetAverageMa}mA voltageSaturated=$speedVoltageSaturated"
        [void](Assert-CescResponse (Send-CescRequest $port 5 2 $nextSequence ([byte[]]@())) 5 2 $nextSequence)
        ++$nextSequence
        Start-Sleep -Milliseconds 100
        $powerAfter = Invoke-CescRequestWithRetry $port 5 0 $nextSequence ([byte[]]@())
        ++$nextSequence
        $sampleAfter = Invoke-CescRequestWithRetry $port 2 1 $nextSequence ([byte[]]@(0))
        ++$nextSequence
        if ([int]$powerAfter[0] -ne 2 -or ([int]$powerAfter[1] -band 0x03) -ne 0 -or
            [int]$powerAfter[305] -ne 0) {
            throw "STOP did not release speed control"
        }
        $speedPositionAfter = [BitConverter]::ToSingle([byte[]]$sampleAfter[22..25], 0)
        $speedPositionDelta = [Math]::Round($speedPositionAfter - $speedPositionStart, 3)
        $speedAverageMdps = [Math]::Round($actualSpeedSumMdps / [double]$speedSamples, 1)
        $speedSteadyAverageMdps = [Math]::Round($steadySpeedSumMdps / [double]$steadySpeedSamples, 1)
        $speedTargetRpmResult = [Math]::Round($SpeedTargetMdps / 6000.0, 2)
        $speedAverageRpm = [Math]::Round($speedAverageMdps / 6000.0, 2)
        $speedSteadyAverageRpm = [Math]::Round($speedSteadyAverageMdps / 6000.0, 2)
        $motorResult = "speed targetMdps=$SpeedTargetMdps targetRpm=$speedTargetRpmResult actualAverageMdps=$speedAverageMdps actualAverageRpm=$speedAverageRpm steadyAverageMdps=$speedSteadyAverageMdps steadyAverageRpm=$speedSteadyAverageRpm idAverageMa=$speedIdAverageMa iqAverageMa=$speedIqAverageMa iqTargetAverageMa=$speedIqTargetAverageMa positionDeltaDegrees=$speedPositionDelta stopVerified=true"
    }
    if ($RunPositionTest) {
        if ([Math]::Abs($RelativePositionTargetMdeg) -lt 500) {
            throw "RelativePositionTargetMdeg must have an absolute value of at least 500"
        }
        [int]$positionStartMdeg = [Math]::Round($positionDegrees * 1000.0)
        [int]$positionTargetMdeg = $positionStartMdeg + $RelativePositionTargetMdeg
        [byte[]]$positionPayload = [BitConverter]::GetBytes($positionTargetMdeg)
        [int]$positionFeedbackMdeg = $positionStartMdeg
        [int]$positionSpeedMdps = 0
        [int]$positionIqTargetMa = 0
        for ($sampleIndex = 0; $sampleIndex -lt 30; ++$sampleIndex) {
            [void](Assert-CescResponse (Send-CescRequest $port 5 11 $nextSequence $positionPayload) 5 11 $nextSequence)
            ++$nextSequence
            Start-Sleep -Milliseconds 100
            $powerAfter = Invoke-CescRequestWithRetry $port 5 0 $nextSequence ([byte[]]@())
            ++$nextSequence
            if ($powerAfter.Length -lt 338) { throw "Power-stage response lacks position diagnostics" }
            $stateNow = [int]$powerAfter[0]
            $flagsNow = [int]$powerAfter[1]
            $faultsNow = [int]$powerAfter[2] -bor ([int]$powerAfter[3] -shl 8)
            $controlMode = [int]$powerAfter[305]
            $positionIqTargetMa = [BitConverter]::ToInt32([byte[]]$powerAfter[314..317], 0)
            $positionSpeedMdps = [BitConverter]::ToInt32([byte[]]$powerAfter[326..329], 0)
            $targetNow = [BitConverter]::ToInt32([byte[]]$powerAfter[330..333], 0)
            $positionFeedbackMdeg = [BitConverter]::ToInt32([byte[]]$powerAfter[334..337], 0)
            if ($stateNow -ne 3 -or ($flagsNow -band 0x03) -ne 0x03 -or
                $faultsNow -ne 0 -or $controlMode -ne 3 -or
                $targetNow -ne $positionTargetMdeg -or
                [Math]::Abs($positionIqTargetMa) -gt 300) {
                throw "Position control invalid state=$stateNow flags=$flagsNow faults=$faultsNow mode=$controlMode target=$targetNow actual=$positionFeedbackMdeg speed=$positionSpeedMdps iqTarget=$positionIqTargetMa"
            }
            Write-Output "Position sample=$sampleIndex target=${targetNow}mdeg actual=${positionFeedbackMdeg}mdeg speed=${positionSpeedMdps}mdps iqTarget=${positionIqTargetMa}mA"
        }
        [void](Assert-CescResponse (Send-CescRequest $port 5 2 $nextSequence ([byte[]]@())) 5 2 $nextSequence)
        ++$nextSequence
        Start-Sleep -Milliseconds 100
        $powerAfter = Invoke-CescRequestWithRetry $port 5 0 $nextSequence ([byte[]]@())
        ++$nextSequence
        if ([int]$powerAfter[0] -ne 2 -or ([int]$powerAfter[1] -band 0x03) -ne 0 -or
            [int]$powerAfter[305] -ne 0) { throw "STOP did not release position control" }
        $positionErrorMdeg = $positionTargetMdeg - $positionFeedbackMdeg
        if ([Math]::Abs($positionErrorMdeg) -gt 1000) {
            throw "Position target not reached target=$positionTargetMdeg actual=$positionFeedbackMdeg error=$positionErrorMdeg"
        }
        $motorResult = "position relativeTargetMdeg=$RelativePositionTargetMdeg finalErrorMdeg=$positionErrorMdeg finalSpeedMdps=$positionSpeedMdps finalIqTargetMa=$positionIqTargetMa stopVerified=true"
    }
    if ($RunPositionProfileTest) {
        if ([Math]::Abs($RelativePositionTargetMdeg) -lt 500) {
            throw "RelativePositionTargetMdeg must have an absolute value of at least 500"
        }
        [int]$profileStartMdeg = [Math]::Round($positionDegrees * 1000.0)
        [int]$profileTargetMdeg = $profileStartMdeg + $RelativePositionTargetMdeg
        if ($profileTargetMdeg -lt -3600000 -or $profileTargetMdeg -gt 3600000) {
            throw "Absolute profile target is outside the firmware +/-10 turn range"
        }
        [byte[]]$profilePayload = [BitConverter]::GetBytes($profileTargetMdeg) +
            [BitConverter]::GetBytes([int]$PositionProfileSpeedMdps) +
            [BitConverter]::GetBytes([int]$PositionProfileAccelerationMdps2) +
            [BitConverter]::GetBytes([int]$PositionProfileDecelerationMdps2)
        [int]$profileFeedbackMdeg = $profileStartMdeg
        [int]$profileActualSpeedMdps = 0
        [int]$profileIqTargetMa = 0
        [double]$profileDistanceMdeg = [Math]::Abs($RelativePositionTargetMdeg)
        [double]$profileMaximumSpeed = $PositionProfileSpeedMdps
        [double]$profileAcceleration = $PositionProfileAccelerationMdps2
        [double]$profileDeceleration = $PositionProfileDecelerationMdps2
        [double]$profileAccelerationTime =
            $profileMaximumSpeed / $profileAcceleration
        [double]$profileDecelerationTime =
            $profileMaximumSpeed / $profileDeceleration
        [double]$profileRampDistance =
            0.5 * $profileMaximumSpeed *
            ($profileAccelerationTime + $profileDecelerationTime)
        [double]$profileMotionSeconds = if ($profileDistanceMdeg -gt $profileRampDistance) {
            $profileAccelerationTime + $profileDecelerationTime +
                ($profileDistanceMdeg - $profileRampDistance) /
                    $profileMaximumSpeed
        } else {
            [double]$profilePeakSpeed = [Math]::Sqrt(
                2.0 * $profileDistanceMdeg /
                (1.0 / $profileAcceleration + 1.0 / $profileDeceleration))
            $profilePeakSpeed / $profileAcceleration +
                $profilePeakSpeed / $profileDeceleration
        }
        [int]$profileDurationMs = [Math]::Min(30000,
            [Math]::Max(2000, [int](1000.0 * $profileMotionSeconds) + 3000))
        [int]$profileSamples = [int][Math]::Ceiling($profileDurationMs / 100.0)
        for ($sampleIndex = 0; $sampleIndex -lt $profileSamples; ++$sampleIndex) {
            [void](Assert-CescResponse (Send-CescRequest $port 5 12 $nextSequence $profilePayload) 5 12 $nextSequence)
            ++$nextSequence
            Start-Sleep -Milliseconds 100
            $powerAfter = Invoke-CescRequestWithRetry $port 5 0 $nextSequence ([byte[]]@())
            ++$nextSequence
            if ($powerAfter.Length -lt 338) { throw "Power-stage response lacks profile diagnostics" }
            $stateNow = [int]$powerAfter[0]
            $flagsNow = [int]$powerAfter[1]
            $faultsNow = [int]$powerAfter[2] -bor ([int]$powerAfter[3] -shl 8)
            $controlMode = [int]$powerAfter[305]
            $profileIqTargetMa = [BitConverter]::ToInt32([byte[]]$powerAfter[314..317], 0)
            $profileActualSpeedMdps = [BitConverter]::ToInt32([byte[]]$powerAfter[326..329], 0)
            $targetNow = [BitConverter]::ToInt32([byte[]]$powerAfter[330..333], 0)
            $profileFeedbackMdeg = [BitConverter]::ToInt32([byte[]]$powerAfter[334..337], 0)
            $profileControl = Invoke-CescRequestWithRetry $port 5 15 $nextSequence ([byte[]]@())
            ++$nextSequence
            if ($profileControl.Length -lt 87) {
                throw "Control-status response is truncated during profile test"
            }
            $profileSpeedTargetMdps = [BitConverter]::ToInt32(
                [byte[]]$profileControl[10..13], 0)
            $profileSpeedReferenceMdps = [BitConverter]::ToInt32(
                [byte[]]$profileControl[14..17], 0)
            $profileCurrentFoc = [int]$profileControl[9]
            $profileIqActualMa = [BitConverter]::ToInt32(
                [byte[]]$profileControl[26..29], 0)
            $profileVoltageQCounts = [BitConverter]::ToInt32(
                [byte[]]$profileControl[78..81], 0)
            $profileVoltageLimitCounts = [BitConverter]::ToInt32(
                [byte[]]$profileControl[82..85], 0)
            $profileCurrentLimited = [int]$profileControl[86]
            if ($stateNow -ne 3 -or ($flagsNow -band 0x03) -ne 0x03 -or
                $faultsNow -ne 0 -or $controlMode -ne 4 -or
                $targetNow -ne $profileTargetMdeg -or
                [Math]::Abs($profileIqTargetMa) -gt 300) {
                throw "Position profile invalid state=$stateNow flags=$flagsNow faults=$faultsNow mode=$controlMode target=$targetNow actual=$profileFeedbackMdeg speed=$profileActualSpeedMdps iqTarget=$profileIqTargetMa"
            }
            Write-Output "Profile sample=$sampleIndex target=${targetNow}mdeg actual=${profileFeedbackMdeg}mdeg speed=${profileActualSpeedMdps}mdps speedTarget=${profileSpeedTargetMdps}mdps speedReference=${profileSpeedReferenceMdps}mdps Vq=$profileVoltageQCounts/$profileVoltageLimitCounts currentLimited=$profileCurrentLimited currentFoc=$profileCurrentFoc Iq=${profileIqActualMa}mA iqTarget=${profileIqTargetMa}mA"
        }
        [void](Assert-CescResponse (Send-CescRequest $port 5 2 $nextSequence ([byte[]]@())) 5 2 $nextSequence)
        ++$nextSequence
        Start-Sleep -Milliseconds 100
        $powerAfter = Invoke-CescRequestWithRetry $port 5 0 $nextSequence ([byte[]]@())
        ++$nextSequence
        if ([int]$powerAfter[0] -ne 2 -or ([int]$powerAfter[1] -band 0x03) -ne 0 -or
            [int]$powerAfter[305] -ne 0) { throw "STOP did not release position profile" }
        $profileErrorMdeg = $profileTargetMdeg - $profileFeedbackMdeg
        if ([Math]::Abs($profileErrorMdeg) -gt 1500) {
            throw "Position profile target not reached target=$profileTargetMdeg actual=$profileFeedbackMdeg error=$profileErrorMdeg"
        }
        $motorResult = "position-profile relativeTargetMdeg=$RelativePositionTargetMdeg maximumSpeedMdps=$PositionProfileSpeedMdps accelerationMdps2=$PositionProfileAccelerationMdps2 decelerationMdps2=$PositionProfileDecelerationMdps2 finalErrorMdeg=$profileErrorMdeg finalSpeedMdps=$profileActualSpeedMdps finalIqTargetMa=$profileIqTargetMa stopVerified=true"
    }
    if ($RunHapticTest) {
        [int]$hapticCenterMdeg = [Math]::Round($positionDegrees * 1000.0)
        [int]$hapticMinimumMdeg = [Math]::Max(-3600000, $hapticCenterMdeg - 180000)
        [int]$hapticMaximumMdeg = [Math]::Min(3600000, $hapticCenterMdeg + 180000)
        [byte[]]$hapticPayload = [BitConverter]::GetBytes([int]$HapticDetentSpacingMdeg) +
            [BitConverter]::GetBytes([int]$HapticDetentStrengthMa) +
            [BitConverter]::GetBytes([int]$HapticDampingMaPerDps) +
            [BitConverter]::GetBytes($hapticMinimumMdeg) +
            [BitConverter]::GetBytes($hapticMaximumMdeg)
        for ($sampleIndex = 0; $sampleIndex -lt 30; ++$sampleIndex) {
            [void](Assert-CescResponse (Send-CescRequest $port 5 13 $nextSequence $hapticPayload) 5 13 $nextSequence)
            ++$nextSequence
            Start-Sleep -Milliseconds 100
            $powerAfter = Invoke-CescRequestWithRetry $port 5 0 $nextSequence ([byte[]]@())
            ++$nextSequence
            $stateNow = [int]$powerAfter[0]
            $flagsNow = [int]$powerAfter[1]
            $faultsNow = [int]$powerAfter[2] -bor ([int]$powerAfter[3] -shl 8)
            $controlMode = [int]$powerAfter[305]
            $hapticIqTargetMa = [BitConverter]::ToInt32([byte[]]$powerAfter[314..317], 0)
            $hapticSpeedMdps = [BitConverter]::ToInt32([byte[]]$powerAfter[326..329], 0)
            $hapticPositionMdeg = [BitConverter]::ToInt32([byte[]]$powerAfter[334..337], 0)
            if ($stateNow -ne 3 -or ($flagsNow -band 0x03) -ne 0x03 -or
                $faultsNow -ne 0 -or $controlMode -ne 5 -or
                [Math]::Abs($hapticIqTargetMa) -gt 300) {
                throw "Haptic control invalid state=$stateNow flags=$flagsNow faults=$faultsNow mode=$controlMode position=$hapticPositionMdeg speed=$hapticSpeedMdps iqTarget=$hapticIqTargetMa"
            }
            Write-Output "Haptic sample=$sampleIndex position=${hapticPositionMdeg}mdeg speed=${hapticSpeedMdps}mdps iqTarget=${hapticIqTargetMa}mA"
        }
        [void](Assert-CescResponse (Send-CescRequest $port 5 2 $nextSequence ([byte[]]@())) 5 2 $nextSequence)
        ++$nextSequence
        Start-Sleep -Milliseconds 100
        $powerAfter = Invoke-CescRequestWithRetry $port 5 0 $nextSequence ([byte[]]@())
        ++$nextSequence
        if ([int]$powerAfter[0] -ne 2 -or ([int]$powerAfter[1] -band 0x03) -ne 0 -or
            [int]$powerAfter[305] -ne 0) { throw "STOP did not release haptic control" }
        $motorResult = "haptic spacingMdeg=$HapticDetentSpacingMdeg strengthMa=$HapticDetentStrengthMa dampingMaPerDps=$HapticDampingMaPerDps limitsMdeg=$hapticMinimumMdeg..$hapticMaximumMdeg stopVerified=true"
    }
    if ($RunResistanceTest -or $RunInductanceTest -or $RunFluxTest) {
        [void](Assert-CescResponse (Send-CescRequest $port 5 6 $nextSequence ([byte[]]@())) 5 6 $nextSequence)
        ++$nextSequence
        $deadline = [DateTime]::UtcNow.AddSeconds(8)
        do {
            Start-Sleep -Milliseconds 100
            $powerAfter = Invoke-CescRequestWithRetry $port 5 0 $nextSequence ([byte[]]@())
            ++$nextSequence
            $testStateNow = [int]$powerAfter[38]
            $phase = [int]$powerAfter[204]
            $valid = [int]$powerAfter[205]
            $forwardSamples = [BitConverter]::ToUInt32([byte[]]$powerAfter[206..209], 0)
            $reverseSamples = [BitConverter]::ToUInt32([byte[]]$powerAfter[210..213], 0)
            $idMa = [BitConverter]::ToInt32([byte[]]$powerAfter[214..217], 0)
            $iqMa = [BitConverter]::ToInt32([byte[]]$powerAfter[218..221], 0)
            $vdMv = [BitConverter]::ToInt32([byte[]]$powerAfter[222..225], 0)
            $vqMv = [BitConverter]::ToInt32([byte[]]$powerAfter[226..229], 0)
            $liveMilliohms = [BitConverter]::ToInt32([byte[]]$powerAfter[230..233], 0)
            $forwardMilliohms = [BitConverter]::ToInt32([byte[]]$powerAfter[234..237], 0)
            $reverseMilliohms = [BitConverter]::ToInt32([byte[]]$powerAfter[238..241], 0)
            $averageMilliohms = [BitConverter]::ToInt32([byte[]]$powerAfter[242..245], 0)
            $adcCurrent = for ($phaseIndex = 0; $phaseIndex -lt 3; ++$phaseIndex) {
                [BitConverter]::ToInt32([byte[]]$powerAfter[(18 + 8 * $phaseIndex)..(21 + 8 * $phaseIndex)], 0)
            }
            Write-Output ("Resistance phase={0} Id={1:F3}A Iq={2:F3}A Vd={3:F3}V Vq={4:F3}V ADC=[{5}] R={6:F3}ohm samples={7}/{8}" -f
                $phase, ($idMa / 1000.0), ($iqMa / 1000.0), ($vdMv / 1000.0),
                ($vqMv / 1000.0), ($adcCurrent -join ','), ($liveMilliohms / 1000.0),
                $forwardSamples, $reverseSamples)
        } while ($testStateNow -eq 1 -and [DateTime]::UtcNow -lt $deadline)
        $stateAfter = [int]$powerAfter[0]
        $flagsAfter = [int]$powerAfter[1]
        $faultsAfter = [int]$powerAfter[2] -bor ([int]$powerAfter[3] -shl 8)
        $testState = [int]$powerAfter[38]
        if ($stateAfter -ne 2 -or ($flagsAfter -band 0x03) -ne 0 -or
            $faultsAfter -ne 0 -or $testState -ne 2 -or
            $valid -ne 1 -or $forwardSamples -lt 1000 -or $reverseSamples -ne 0 -or
            $forwardMilliohms -le 0 -or $reverseMilliohms -ne 0 -or $averageMilliohms -le 0) {
            throw "Resistance measurement failed state=$stateAfter flags=$flagsAfter faults=$faultsAfter testState=$testState valid=$valid samples=$forwardSamples/$reverseSamples resistanceMilliohms=$forwardMilliohms/$reverseMilliohms/$averageMilliohms"
        }
        $motorResult = "resistance samples=$forwardSamples vescLockedOhm=$($forwardMilliohms / 1000.0) phaseResistanceOhm=$($averageMilliohms / 1000.0)"
    }
    if ($RunInductanceTest) {
        [void](Assert-CescResponse (Send-CescRequest $port 5 7 $nextSequence ([byte[]]@())) 5 7 $nextSequence)
        ++$nextSequence
        $deadline = [DateTime]::UtcNow.AddSeconds(5)
        do {
            Start-Sleep -Milliseconds 100
            $powerAfter = Invoke-CescRequestWithRetry $port 5 0 $nextSequence ([byte[]]@())
            ++$nextSequence
            $testStateNow = [int]$powerAfter[38]
            $inductancePhase = [int]$powerAfter[246]
            $inductanceValid = [int]$powerAfter[247]
            $inductanceForwardSamples = [BitConverter]::ToUInt32([byte[]]$powerAfter[248..251], 0)
            $inductanceReverseSamples = [BitConverter]::ToUInt32([byte[]]$powerAfter[252..255], 0)
            $deltaCurrentMa = [BitConverter]::ToInt32([byte[]]$powerAfter[256..259], 0)
            $inductiveVoltageMv = [BitConverter]::ToInt32([byte[]]$powerAfter[260..263], 0)
            $forwardUh = [BitConverter]::ToUInt32([byte[]]$powerAfter[264..267], 0)
            $reverseUh = [BitConverter]::ToUInt32([byte[]]$powerAfter[268..271], 0)
            $averageUh = [BitConverter]::ToUInt32([byte[]]$powerAfter[272..275], 0)
            Write-Output ("Inductance phase={0} dI={1:F3}A Vind={2:F3}V samples={3}/{4} L={5}/{6}/{7}uH" -f
                $inductancePhase, ($deltaCurrentMa / 1000.0), ($inductiveVoltageMv / 1000.0),
                $inductanceForwardSamples, $inductanceReverseSamples, $forwardUh, $reverseUh, $averageUh)
        } while ($testStateNow -eq 1 -and [DateTime]::UtcNow -lt $deadline)
        $stateAfter = [int]$powerAfter[0]
        $flagsAfter = [int]$powerAfter[1]
        $faultsAfter = [int]$powerAfter[2] -bor ([int]$powerAfter[3] -shl 8)
        $testState = [int]$powerAfter[38]
        if ($stateAfter -ne 2 -or ($flagsAfter -band 0x03) -ne 0 -or
            $faultsAfter -ne 0 -or $testState -ne 2 -or $inductanceValid -ne 1) {
            throw "Inductance measurement failed state=$stateAfter flags=$flagsAfter faults=$faultsAfter testState=$testState valid=$inductanceValid samples=$inductanceForwardSamples/$inductanceReverseSamples L_uH=$forwardUh/$reverseUh/$averageUh"
        }
        $motorResult += " inductance samples=$inductanceForwardSamples/$inductanceReverseSamples forwardUh=$forwardUh reverseUh=$reverseUh phaseInductanceUh=$averageUh"
    }
    if ($RunFluxTest) {
        [byte]$directionByte = if ($CommissioningDirection -gt 0) { 1 } else { 255 }
        [void](Assert-CescResponse (Send-CescRequest $port 5 8 $nextSequence ([byte[]]@($directionByte))) 5 8 $nextSequence)
        ++$nextSequence
        $deadline = [DateTime]::UtcNow.AddSeconds(12)
        do {
            Start-Sleep -Milliseconds 200
            $powerAfter = Invoke-CescRequestWithRetry $port 5 0 $nextSequence ([byte[]]@())
            ++$nextSequence
            $testStateNow = [int]$powerAfter[38]
            $fluxValid = [int]$powerAfter[276]
            $fluxSamples = [BitConverter]::ToUInt32([byte[]]$powerAfter[277..280], 0)
            $speedMdps = [BitConverter]::ToInt32([byte[]]$powerAfter[281..284], 0)
            $fluxIqMa = [BitConverter]::ToInt32([byte[]]$powerAfter[285..288], 0)
            $fluxVqMv = [BitConverter]::ToInt32([byte[]]$powerAfter[289..292], 0)
            $linkageUwb = [BitConverter]::ToUInt32([byte[]]$powerAfter[293..296], 0)
            $keUv = [BitConverter]::ToUInt32([byte[]]$powerAfter[297..300], 0)
            $kvMilli = [BitConverter]::ToUInt32([byte[]]$powerAfter[301..304], 0)
            Write-Output ("Flux speed={0:F2}dps Iq={1:F3}A Vq={2:F3}V samples={3} lambda={4:F6}Wb Ke={5:F6}V/(rad/s) KV={6:F2}rpm/V" -f
                ($speedMdps / 1000.0), ($fluxIqMa / 1000.0), ($fluxVqMv / 1000.0),
                $fluxSamples, ($linkageUwb / 1000000.0), ($keUv / 1000000.0), ($kvMilli / 1000.0))
        } while ($testStateNow -eq 1 -and [DateTime]::UtcNow -lt $deadline)
        $stateAfter = [int]$powerAfter[0]
        $flagsAfter = [int]$powerAfter[1]
        $faultsAfter = [int]$powerAfter[2] -bor ([int]$powerAfter[3] -shl 8)
        $testState = [int]$powerAfter[38]
        if ($stateAfter -ne 2 -or ($flagsAfter -band 0x03) -ne 0 -or
            $faultsAfter -ne 0 -or $testState -ne 2 -or $fluxValid -ne 1) {
            throw "Flux measurement failed state=$stateAfter flags=$flagsAfter faults=$faultsAfter testState=$testState valid=$fluxValid samples=$fluxSamples"
        }
        $motorResult += " flux samples=$fluxSamples lambdaWb=$($linkageUwb / 1000000.0) Ke=$($keUv / 1000000.0) KV=$($kvMilli / 1000.0)"
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
        Start-Sleep -Milliseconds 9300
        $sampleHeld = Assert-CescResponse (Send-CescRequest $port 2 1 $nextSequence ([byte[]]@(0))) 2 1 $nextSequence
        ++$nextSequence
        Start-Sleep -Milliseconds 300
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
        $motorResult = "encoder-vesc-bidirectional-sweep zeroRaw=$zeroAfter finalRaw=$alignedAfter finalDegrees=$alignedDegreesAfter mechanicalDelta=$alignmentMechanicalDelta"
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
    # Unsubscribe is idempotent from the host's perspective. If its success
    # response is lost, retrying returns NOT_READY because the stream is
    # already gone, so do not route this command through the generic retry.
    [void](Send-CescRequest $port 3 2 $nextSequence ([byte[]]@(
        ($streamId -band 0xff), ($streamId -shr 8))) 100)
    ++$nextSequence
    $commStatsAfter = Invoke-CescRequestWithRetry $port 0 4 $nextSequence ([byte[]]@())
    ++$nextSequence
    [uint32]$usbMutexAfter = [BitConverter]::ToUInt32([byte[]]$commStatsAfter[36..39], 0)
    [uint32]$usbSubmitAfter = [BitConverter]::ToUInt32([byte[]]$commStatsAfter[40..43], 0)
    [uint32]$usbCompleteAfter = [BitConverter]::ToUInt32([byte[]]$commStatsAfter[44..47], 0)
    [uint32]$usbDropAfter = [BitConverter]::ToUInt32([byte[]]$commStatsAfter[48..51], 0)
    $usbDiagnosticText = "mutex=$usbMutexAfter(+$($usbMutexAfter-$usbMutexBefore)) submit=$usbSubmitAfter(+$($usbSubmitAfter-$usbSubmitBefore)) complete=$usbCompleteAfter(+$($usbCompleteAfter-$usbCompleteBefore)) drops=$usbDropAfter(+$($usbDropAfter-$usbDropBefore))"
    if ($streamFrames -lt 5) { throw "Expected telemetry frames, got $streamFrames" }
    Write-Output "PASS port=$PortName sensorStatus=$sensorStatus angleRaw=$raw angle=$angle positionCounts=$positionCounts positionDegrees=$positionDegrees electricalRaw=$electricalRaw electricalDegrees=$electricalDegrees polePairs=$polePairs angleFlags=$angleFlags electricalZeroRaw=$electricalZeroRaw alignedElectricalRaw=$alignedElectricalRaw alignedElectricalDegrees=$alignedElectricalDegrees powerState=$powerState flags=$powerFlags drvFaults=$drvFaults busRaw=$busRaw busMv=$busMv currentSeq=$currentSequence phases(raw/offset/centered)=$($phaseText -join ',') cpuFault=[$cpuFaultText] motorTest=[$motorResult] usb=[$usbDiagnosticText] streamId=$streamId frames=$streamFrames"
} finally {
    if ($port.IsOpen) {
        try { [void](Send-CescRequest $port 5 2 0xffff ([byte[]]@()) 20) } catch {}
    }
    if ($port.IsOpen) {
        try { $port.Close() }
        catch { Write-Warning "Serial close failed: $($_.Exception.Message)" }
    }
    $port.Dispose()
}
