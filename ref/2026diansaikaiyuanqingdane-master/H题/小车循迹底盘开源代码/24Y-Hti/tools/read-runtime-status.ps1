param(
    [uint32]$DelayBeforeReadMs = 0,
    [switch]$ResumeWithoutReset
)

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

function Get-AddressKey([uint64]$address) {
    return $address.ToString('X8')
}

function Convert-ToSigned([uint64]$value, [int]$bits) {
    $sign = [uint64]1 -shl ($bits - 1)
    $range = [uint64]1 -shl $bits
    if (($value -band $sign) -ne 0) {
        return [int64]$value - [int64]$range
    }
    return [int64]$value
}

$items = @(
    @{ Name = 'cyz_requests'; Symbol = 'g_cyz_i2c_request_count'; Read = 'mdw'; Bits = 32; Signed = $false },
    @{ Name = 'cyz_valid'; Symbol = 'g_cyz_telemetry_frame_count'; Read = 'mdw'; Bits = 32; Signed = $false },
    @{ Name = 'cyz_i2c_errors'; Symbol = 'g_cyz_i2c_error_count'; Read = 'mdw'; Bits = 32; Signed = $false },
    @{ Name = 'cyz_crc_errors'; Symbol = 'g_cyz_crc_error_count'; Read = 'mdw'; Bits = 32; Signed = $false },
    @{ Name = 'cyz_format_errors'; Symbol = 'g_cyz_format_error_count'; Read = 'mdw'; Bits = 32; Signed = $false },
    @{ Name = 'cyz_sample_hz'; Symbol = 'g_cyz_i2c_sample_hz'; Read = 'mdh'; Bits = 16; Signed = $false },
    @{ Name = 'cyz_status'; Symbol = 'g_cyz_i2c_status'; Read = 'mdh'; Bits = 16; Signed = $false },
    @{ Name = 'cyz_busy'; Symbol = 'g_cyz_i2c_busy'; Read = 'mdb'; Bits = 8; Signed = $false },
    @{ Name = 'cyz_angle_mdeg'; Symbol = 'g_cyz_angle_mdeg'; Read = 'mdw'; Bits = 32; Signed = $true },
    @{ Name = 'cyz_gyro_mdps'; Symbol = 'g_cyz_gyro_mdps'; Read = 'mdw'; Bits = 32; Signed = $true },
    @{ Name = 'line_valid'; Symbol = 'g_line_valid_frame_count'; Read = 'mdw'; Bits = 32; Signed = $false },
    @{ Name = 'line_lost'; Symbol = 'g_line_lost'; Read = 'mdw'; Bits = 32; Signed = $false },
    @{ Name = 'left_encoder_count'; Symbol = 'g_encoder_left_count'; Read = 'mdw'; Bits = 32; Signed = $true },
    @{ Name = 'right_encoder_count'; Symbol = 'g_encoder_right_count'; Read = 'mdw'; Bits = 32; Signed = $true },
    @{ Name = 'left_a_edges'; Symbol = 'g_encoder_left_a_transition_count'; Read = 'mdw'; Bits = 32; Signed = $false },
    @{ Name = 'left_b_edges'; Symbol = 'g_encoder_left_b_transition_count'; Read = 'mdw'; Bits = 32; Signed = $false },
    @{ Name = 'right_a_edges'; Symbol = 'g_encoder_right_a_transition_count'; Read = 'mdw'; Bits = 32; Signed = $false },
    @{ Name = 'right_b_edges'; Symbol = 'g_encoder_right_b_transition_count'; Read = 'mdw'; Bits = 32; Signed = $false },
    @{ Name = 'left_a_state'; Symbol = 'g_encoder_left_a_state'; Read = 'mdb'; Bits = 8; Signed = $false },
    @{ Name = 'left_b_state'; Symbol = 'g_encoder_left_b_state'; Read = 'mdb'; Bits = 8; Signed = $false },
    @{ Name = 'right_a_state'; Symbol = 'g_encoder_right_a_state'; Read = 'mdb'; Bits = 8; Signed = $false },
    @{ Name = 'right_b_state'; Symbol = 'g_encoder_right_b_state'; Read = 'mdb'; Bits = 8; Signed = $false },
    @{ Name = 'left_cps'; Symbol = 'g_wheel_speed_left_cps'; Read = 'mdw'; Bits = 32; Signed = $true },
    @{ Name = 'right_cps'; Symbol = 'g_wheel_speed_right_cps'; Read = 'mdw'; Bits = 32; Signed = $true },
    @{ Name = 'left_peak_cps'; Symbol = 'g_wheel_speed_left_peak_abs_cps'; Read = 'mdw'; Bits = 32; Signed = $true },
    @{ Name = 'right_peak_cps'; Symbol = 'g_wheel_speed_right_peak_abs_cps'; Read = 'mdw'; Bits = 32; Signed = $true },
    @{ Name = 'left_end_count'; Symbol = 'g_speed_test_left_end_count'; Read = 'mdw'; Bits = 32; Signed = $true },
    @{ Name = 'right_end_count'; Symbol = 'g_speed_test_right_end_count'; Read = 'mdw'; Bits = 32; Signed = $true },
    @{ Name = 'left_target_cps'; Symbol = 'g_chassis_left_target_cps'; Read = 'mdw'; Bits = 32; Signed = $true },
    @{ Name = 'right_target_cps'; Symbol = 'g_chassis_right_target_cps'; Read = 'mdw'; Bits = 32; Signed = $true },
    @{ Name = 'profile_speed_mm_s'; Symbol = 'g_vehicle_profile_speed_mm_s'; Read = 'mdw'; Bits = 32; Signed = $true },
    @{ Name = 'left_output'; Symbol = 'g_pi_left_output_permille'; Read = 'mdw'; Bits = 32; Signed = $true },
    @{ Name = 'right_output'; Symbol = 'g_pi_right_output_permille'; Read = 'mdw'; Bits = 32; Signed = $true },
    @{ Name = 'fault'; Symbol = 'g_pi_fault_code'; Read = 'mdw'; Bits = 32; Signed = $false },
    @{ Name = 'finished'; Symbol = 'g_line_follow_finished'; Read = 'mdw'; Bits = 32; Signed = $false }
)

$openOcdArguments = @('-f', $openOcdConfig, '-c', 'init')
if ($DelayBeforeReadMs -gt 0) {
    $openOcdArguments += @(
        '-c', 'reset run',
        '-c', ('sleep {0}' -f $DelayBeforeReadMs),
        '-c', 'halt'
    )
} else {
    $openOcdArguments += @('-c', 'halt')
}
foreach ($item in $items) {
    $item.Address = Get-SymbolAddress $item.Symbol
    $openOcdArguments += @(
        '-c', ('{0} 0x{1:X8} 1' -f $item.Read, [uint64]$item.Address)
    )
}
if ($ResumeWithoutReset) {
    $openOcdArguments += @('-c', 'resume', '-c', 'shutdown')
} else {
    $openOcdArguments += @('-c', 'reset run', '-c', 'shutdown')
}

$savedPreference = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
$openOcdOutput = & $env:HTI_OPENOCD_EXE @openOcdArguments 2>&1
$openOcdExitCode = $LASTEXITCODE
$ErrorActionPreference = $savedPreference
if ($openOcdExitCode -ne 0) {
    $openOcdOutput | ForEach-Object { Write-Host $_ }
    throw "OpenOCD runtime read failed with exit code $openOcdExitCode."
}

$memory = @{}
foreach ($outputLine in $openOcdOutput) {
    if ($outputLine.ToString() -match '^0x([0-9a-fA-F]+):\s+([0-9a-fA-F]+)') {
        $memory[(Get-AddressKey ([Convert]::ToUInt64($Matches[1], 16)))] =
            [Convert]::ToUInt64($Matches[2], 16)
    }
}

foreach ($item in $items) {
    $key = Get-AddressKey ([uint64]$item.Address)
    if (-not $memory.ContainsKey($key)) {
        throw ('No value returned for {0} at 0x{1:X8}' -f
               $item.Name, [uint64]$item.Address)
    }
    $raw = [uint64]$memory[$key]
    $value = if ($item.Signed) {
        Convert-ToSigned $raw $item.Bits
    } else {
        $raw
    }
    Write-Host ('{0}={1}' -f $item.Name, $value)
}
