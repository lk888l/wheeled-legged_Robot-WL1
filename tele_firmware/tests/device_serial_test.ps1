param(
    [Parameter(Mandatory = $true)][string]$Port,
    [string]$LogPath = 'build/uart-radio/hardware-test.log'
)

$ErrorActionPreference = 'Stop'
$serial = [System.IO.Ports.SerialPort]::new($Port, 115200,
    [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.Handshake = [System.IO.Ports.Handshake]::None
$serial.ReadBufferSize = 65536
$serial.WriteTimeout = 1000
$transcript = [Text.StringBuilder]::new()

function Read-Reply([int]$Milliseconds = 250) {
    $reply = [Text.StringBuilder]::new()
    $timer = [Diagnostics.Stopwatch]::StartNew()
    while ($timer.ElapsedMilliseconds -lt $Milliseconds) {
        [void]$reply.Append($serial.ReadExisting())
        Start-Sleep -Milliseconds 10
    }
    $text = $reply.ToString()
    [void]$transcript.Append($text)
    return $text
}

function Check-Reply([string]$Command, [string]$Expected, [int]$Milliseconds = 250) {
    [void]$transcript.AppendLine('> ' + $Command.TrimEnd())
    $serial.Write($Command)
    $reply = Read-Reply $Milliseconds
    if (-not $reply.Contains($Expected)) {
        throw "Expected '$Expected' after '$Command'; received: $reply"
    }
    Write-Output ('PASS: ' + $Command.TrimEnd())
}

try {
    $serial.Open()
    [void](Read-Reply 100)
    Check-Reply "joystick off`r`n" 'joystick: off'
    Check-Reply "help`r`n" 'commands: nrfsend'
    Check-Reply "status`r`n" 'status: joystick=off'

    # Only neutral control frames: these tests do not request wheel motion.
    Check-Reply "nrfsend R 0 0 0 61.5`r`n" 'nRF: send success [uart]: R 0 0 0 61.5'
    Check-Reply "R 0 0 0 61.5`n" 'nRF: send success [uart]: R 0 0 0 61.5'
    Check-Reply 'nrfsend R 0 0 0 61.5' 'nRF: send success [uart]: R 0 0 0 61.5'

    [void]$transcript.AppendLine('> fragmented nrfsend R 0 0 0 61.5')
    $serial.Write('nrf')
    Start-Sleep -Milliseconds 10
    $serial.Write('send R 0')
    Start-Sleep -Milliseconds 10
    $serial.Write(" 0 0 61.5`r`n")
    $reply = Read-Reply 350
    if ([regex]::Matches($reply, 'nRF: send success \[uart\]').Count -ne 1) {
        throw "Fragmented command must produce exactly one ACK: $reply"
    }
    Write-Output 'PASS: fragmented command'

    [void]$transcript.AppendLine('> four commands in one write')
    $serial.Write(("nrfsend R 0 0 0 61.5`r`n" * 4))
    $reply = Read-Reply 600
    if ([regex]::Matches($reply, 'nRF: send success \[uart\]').Count -ne 4) {
        throw "Expected four ACKs from the command burst: $reply"
    }
    Write-Output 'PASS: four-command burst'

    $fullPayload = 'x' * 32
    Check-Reply ("nrfsend $fullPayload`r`n") ("nRF: send success [uart]: $fullPayload")
    Check-Reply ("nrfsend " + ('x' * 33) + "`r`n") 'uart: command too long'
    Check-Reply ("nrfsend " + ('x' * 150) + "`r`n") 'uart: command too long'
    Check-Reply "status`r`n" 'status: joystick=off'

    Check-Reply "nrfsend nrfshow -mr 0`r`n" 'nRF: send success [uart]: nrfshow -mr 0' 500
    $telemetry = Read-Reply 1200
    if (-not $telemetry.Contains('receive: ')) {
        throw 'No car telemetry received after enabling nrfshow'
    }
    Write-Output 'PASS: radio reply forwarded to UART'
    Check-Reply "nrfsend nrfshow -nn`r`n" 'nRF: send success [uart]: nrfshow -nn' 400

    Check-Reply "joystick on`r`n" 'nRF: send success [joystick]' 1000
    Check-Reply "status`r`n" 'rx_errors=0 dropped_logs=0'
    Write-Output 'PASS: hardware UART/radio regression'
} finally {
    if ($serial.IsOpen) {
        # Restore the normal remote mode even when an assertion fails.
        $serial.Write("nrfsend nrfshow -nn`r`n")
        [void](Read-Reply 250)
        $serial.Write("joystick on`r`n")
        [void](Read-Reply 250)
    }
    $serial.Dispose()
    $directory = Split-Path -Parent $LogPath
    if ($directory) { [void](New-Item -ItemType Directory -Force -Path $directory) }
    [IO.File]::WriteAllText([IO.Path]::GetFullPath($LogPath), $transcript.ToString())
}
