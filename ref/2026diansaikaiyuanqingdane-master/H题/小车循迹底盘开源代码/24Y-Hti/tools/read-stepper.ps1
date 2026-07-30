$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$firmware = Join-Path $projectRoot 'build\Debug\HtiCar.elf'
$openOcdConfig = Join-Path $projectRoot 'openocd\openocd.cfg'
. (Join-Path $PSScriptRoot 'setup-env.ps1')

$symbols = & arm-none-eabi-nm.exe --defined-only $firmware

function Get-SymbolAddress([string]$name) {
    $match = $symbols | Select-String -Pattern "^([0-9a-fA-F]+)\s+[A-Za-z]\s+$name$" |
        Select-Object -First 1
    if (-not $match) {
        throw "Symbol not found: $name"
    }
    return [Convert]::ToUInt64($match.Matches[0].Groups[1].Value, 16)
}

function Convert-ToSigned([uint32]$value) {
    if (($value -band [Convert]::ToUInt32('80000000', 16)) -ne 0) {
        return [int64]$value - 4294967296
    }
    return [int64]$value
}

$names = @(
    'g_zdt_stepper_test_state',
    'g_zdt_stepper_test_elapsed_ms',
    'g_zdt_stepper_test_move_sent',
    'g_zdt_stepper_test_position_requested',
    'g_stepper_uart_tx_frame_count',
    'g_stepper_uart_rx_byte_count',
    'g_stepper_uart_rx_overflow_count',
    'g_stepper_uart_error_count',
    'g_zdt_stepper_tx_command_count',
    'g_zdt_stepper_rx_frame_count',
    'g_zdt_stepper_rx_error_count',
    'g_zdt_stepper_position_valid',
    'g_zdt_stepper_last_function',
    'g_zdt_stepper_last_status',
    'g_zdt_stepper_position_raw',
    'g_zdt_stepper_position_pulses',
    'g_zdt_stepper_angle_mdeg'
)

$addresses = @{}
$arguments = @('-f', $openOcdConfig, '-c', 'init')
foreach ($name in $names) {
    $addresses[$name] = Get-SymbolAddress $name
    $arguments += @('-c', ('mdw 0x{0:X8} 1' -f $addresses[$name]))
}
$arguments += @('-c', 'shutdown')

$savedPreference = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
$output = & $env:HTI_OPENOCD_EXE @arguments 2>&1
$exitCode = $LASTEXITCODE
$ErrorActionPreference = $savedPreference
if ($exitCode -ne 0) {
    $output | ForEach-Object { Write-Host $_ }
    throw "OpenOCD stepper read failed with exit code $exitCode."
}

$values = @{}
$addressToName = @{}
foreach ($name in $names) {
    $addressToName[$addresses[$name].ToString('X8')] = $name
}
foreach ($item in $output) {
    if ($item.ToString() -match '^0x([0-9a-fA-F]+):\s+([0-9a-fA-F]{8})') {
        $key = ([Convert]::ToUInt64($Matches[1], 16)).ToString('X8')
        if ($addressToName.ContainsKey($key)) {
            $values[$addressToName[$key]] =
                [Convert]::ToUInt32($Matches[2], 16)
        }
    }
}

$stateNames = @('unavailable', 'waiting', 'enable-settle', 'moving',
                'waiting-position', 'finished', 'position-timeout')
$state = [int]$values['g_zdt_stepper_test_state']
$stateName = if ($state -ge 0 -and $state -lt $stateNames.Count) {
    $stateNames[$state]
} else { "unknown-$state" }

Write-Host ("Stepper test: state={0}, elapsed={1} ms, move_sent={2}, query_sent={3}" -f
    $stateName,
    $values['g_zdt_stepper_test_elapsed_ms'],
    $values['g_zdt_stepper_test_move_sent'],
    $values['g_zdt_stepper_test_position_requested'])
Write-Host ("UART: tx_frames={0}, rx_bytes={1}, overflow={2}, errors={3}" -f
    $values['g_stepper_uart_tx_frame_count'],
    $values['g_stepper_uart_rx_byte_count'],
    $values['g_stepper_uart_rx_overflow_count'],
    $values['g_stepper_uart_error_count'])
Write-Host ("ZDT: tx_commands={0}, rx_frames={1}, parse_errors={2}, position_valid={3}" -f
    $values['g_zdt_stepper_tx_command_count'],
    $values['g_zdt_stepper_rx_frame_count'],
    $values['g_zdt_stepper_rx_error_count'],
    $values['g_zdt_stepper_position_valid'])
Write-Host ("Last reply: function=0x{0:X2}, status=0x{1:X2}" -f
    $values['g_zdt_stepper_last_function'],
    $values['g_zdt_stepper_last_status'])
Write-Host ("Position: raw={0}, pulses={1}, angle={2:N3} deg" -f
    (Convert-ToSigned $values['g_zdt_stepper_position_raw']),
    (Convert-ToSigned $values['g_zdt_stepper_position_pulses']),
    ((Convert-ToSigned $values['g_zdt_stepper_angle_mdeg']) / 1000.0))
