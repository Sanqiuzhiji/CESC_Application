param(
    [string]$PortName = "COM15",
    [int]$BaudRate = 115200,
    [int]$StreamDurationMs = 500
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

$port = [System.IO.Ports.SerialPort]::new(
    $PortName, $BaudRate, "None", 8, "One")
$port.ReadTimeout = 200
$port.WriteTimeout = 1000
try {
    $port.Open()
    $port.DiscardInBuffer()
    $hello = Assert-CescResponse (Send-CescRequest $port 0 0 1 ([byte[]]@(1,1,0,0,0,0))) 0 0 1
    $sample = Assert-CescResponse (Send-CescRequest $port 2 1 2 ([byte[]]@(0))) 2 1 2
    $raw = [int]$sample[4] -bor ([int]$sample[5] -shl 8)
    $angle = [BitConverter]::ToSingle([byte[]]$sample[6..9], 0)
    [byte[]]$subscription = @(0x10,0x27,0,0,1,3,0,0,1,0,2,0)
    $reply = Assert-CescResponse (Send-CescRequest $port 3 1 3 $subscription) 3 1 3
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
    [void](Send-CescRequest $port 3 2 4 ([byte[]]@(
        ($streamId -band 0xff), ($streamId -shr 8))))
    if ($streamFrames -lt 5) { throw "Expected telemetry frames, got $streamFrames" }
    Write-Output "PASS port=$PortName raw=$raw angle=$angle streamId=$streamId frames=$streamFrames"
} finally {
    if ($port.IsOpen) { $port.Close() }
    $port.Dispose()
}
