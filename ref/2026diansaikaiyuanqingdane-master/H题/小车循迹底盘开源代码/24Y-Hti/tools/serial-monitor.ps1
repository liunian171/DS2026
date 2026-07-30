param(
    [string]$Port = 'COM11',
    [int]$BaudRate = 115200,
    [int]$Seconds = 10
)

$ErrorActionPreference = 'Stop'
$serial = [System.IO.Ports.SerialPort]::new(
    $Port,
    $BaudRate,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One
)
$serial.Handshake = [System.IO.Ports.Handshake]::None
$deadline = [DateTime]::UtcNow.AddSeconds($Seconds)

try {
    $serial.Open()
    Write-Host "Listening on $Port at $BaudRate baud for $Seconds seconds..."

    while ([DateTime]::UtcNow -lt $deadline) {
        $text = $serial.ReadExisting()
        if ($text.Length -gt 0) {
            Write-Host -NoNewline $text
        }
        Start-Sleep -Milliseconds 20
    }
}
finally {
    if ($serial.IsOpen) {
        $serial.Close()
    }
}
