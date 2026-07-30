$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$firmware = Join-Path $projectRoot 'build\Debug\HtiCar.elf'
$openOcdConfig = Join-Path $projectRoot 'openocd\openocd.cfg'
$csvPath = Join-Path $projectRoot 'build\Debug\motion-curve.csv'
. (Join-Path $PSScriptRoot 'setup-env.ps1')

if (-not (Test-Path -LiteralPath $firmware)) {
    throw "Firmware not found: $firmware. Build the Debug preset first."
}

$symbols = & arm-none-eabi-nm.exe --defined-only $firmware

function Get-SymbolAddress([string]$name) {
    $match = $symbols | Select-String -Pattern "^([0-9a-fA-F]+)\s+[A-Za-z]\s+$name$" |
        Select-Object -First 1
    if (-not $match) {
        throw "Symbol not found: $name"
    }
    return [Convert]::ToUInt64($match.Matches[0].Groups[1].Value, 16)
}

function Get-Key([uint64]$address) {
    return $address.ToString('X8')
}

function Convert-ToSigned([uint32]$value) {
    if (($value -band [Convert]::ToUInt32('80000000', 16)) -ne 0) {
        return [int64]$value - [int64]4294967296
    }
    return [int64]$value
}

$capacity = 192
$wordsPerSample = 16
$scalarNames = @(
    'g_pi_fault_code',
    'g_line_follow_finished',
    'g_lap_stop_line_armed',
    'g_lap_stop_line_detected',
    'g_lap_line_active_count_max',
    'g_lap_stop_candidate_max_frames',
    'g_lap_elapsed_ms',
    'g_lap_distance_um',
    'g_lap_heading_progress_mdeg',
    'g_lap_encoder_heading_progress_mdeg',
    'g_lap_gyro_heading_signed_mdeg',
    'g_lap_gyro_valid',
    'g_lap_stop_line_distance_um',
    'g_vehicle_acceleration_max_mm_s2',
    'g_vehicle_acceleration_min_mm_s2',
    'g_motion_log_count',
    'g_motion_log_write_index',
    'g_motion_log_overflow_count',
    'g_cruise_straight_sample_count',
    'g_cruise_straight_speed_min_mm_s',
    'g_cruise_straight_speed_max_mm_s',
    'g_cruise_straight_accel_min_mm_s2',
    'g_cruise_straight_accel_max_mm_s2',
    'g_cruise_turn_sample_count',
    'g_cruise_turn_speed_min_mm_s',
    'g_cruise_turn_speed_max_mm_s',
    'g_cruise_turn_accel_min_mm_s2',
    'g_cruise_turn_accel_max_mm_s2'
)

$addresses = @{}
foreach ($name in $scalarNames) {
    $addresses[$name] = Get-SymbolAddress $name
}
$logAddress = Get-SymbolAddress 'g_motion_log'

$openOcdArguments = @('-f', $openOcdConfig, '-c', 'init')
foreach ($name in $scalarNames) {
    $openOcdArguments += @(
        '-c', ('mdw 0x{0:X8} 1' -f ([uint64]($addresses[$name])))
    )
}
$openOcdArguments += @(
    '-c', ('mdw 0x{0:X8} {1}' -f $logAddress,
            ($capacity * $wordsPerSample)),
    '-c', 'shutdown'
)

$savedErrorActionPreference = $ErrorActionPreference
$ErrorActionPreference = 'Continue'
$openOcdOutput = & $env:HTI_OPENOCD_EXE @openOcdArguments 2>&1
$openOcdExitCode = $LASTEXITCODE
$ErrorActionPreference = $savedErrorActionPreference
if ($openOcdExitCode -ne 0) {
    $openOcdOutput | ForEach-Object { Write-Host $_ }
    throw "OpenOCD motion-curve read failed with exit code $openOcdExitCode."
}

$memory = @{}
foreach ($outputLine in $openOcdOutput) {
    $line = $outputLine.ToString()
    if ($line -match '^0x([0-9a-fA-F]+):\s+(.+)$') {
        $baseAddress = [Convert]::ToUInt64($Matches[1], 16)
        $wordTokens = @(
            $Matches[2] -split '\s+' |
                Where-Object { $_ -match '^[0-9a-fA-F]{8}$' }
        )
        for ($index = 0; $index -lt $wordTokens.Count; $index++) {
            $address = $baseAddress + [uint64](4 * $index)
            $memory[(Get-Key $address)] =
                [Convert]::ToUInt32($wordTokens[$index], 16)
        }
    }
}

function Get-WordAt([uint64]$address) {
    $key = Get-Key $address
    if (-not $memory.ContainsKey($key)) {
        throw ('Memory word was not returned at 0x{0:X8}' -f $address)
    }
    return [uint32]($memory[$key])
}

function Get-Scalar([string]$name) {
    $scalarAddress = [uint64]($addresses[$name])
    return (Get-WordAt $scalarAddress)
}

$count = [int](Get-Scalar 'g_motion_log_count')
$writeIndex = [int](Get-Scalar 'g_motion_log_write_index')
$overflowCount = [uint32](Get-Scalar 'g_motion_log_overflow_count')
$startIndex = if ($count -lt $capacity) { 0 } else { $writeIndex }
$phaseNames = @('waiting', 'accelerating', 'cruising', 'approaching',
                'stopping', 'finished')
$rows = @()

for ($sampleNumber = 0; $sampleNumber -lt $count; $sampleNumber++) {
    $sampleIndex = ($startIndex + $sampleNumber) % $capacity
    $baseAddress = $logAddress + [uint64]($sampleIndex * $wordsPerSample * 4)
    $phase = [int](Get-WordAt ($baseAddress + 4))
    $phaseName = if (($phase -ge 0) -and ($phase -lt $phaseNames.Count)) {
        $phaseNames[$phase]
    } else {
        "unknown-$phase"
    }
    $rows += [pscustomobject]@{
        elapsed_ms = [uint32](Get-WordAt $baseAddress)
        phase = $phaseName
        profile_speed_mm_s = Convert-ToSigned (Get-WordAt ($baseAddress + 8))
        measured_speed_mm_s = Convert-ToSigned (Get-WordAt ($baseAddress + 12))
        acceleration_command_mm_s2 = Convert-ToSigned (Get-WordAt ($baseAddress + 16))
        acceleration_measured_mm_s2 = Convert-ToSigned (Get-WordAt ($baseAddress + 20))
        yaw_command_mdeg_s = Convert-ToSigned (Get-WordAt ($baseAddress + 24))
        gyro_heading_progress_mdeg = Convert-ToSigned (Get-WordAt ($baseAddress + 28))
        encoder_heading_progress_mdeg = Convert-ToSigned (Get-WordAt ($baseAddress + 32))
        left_output_permille = Convert-ToSigned (Get-WordAt ($baseAddress + 36))
        right_output_permille = Convert-ToSigned (Get-WordAt ($baseAddress + 40))
        left_measured_cps = Convert-ToSigned (Get-WordAt ($baseAddress + 44))
        right_measured_cps = Convert-ToSigned (Get-WordAt ($baseAddress + 48))
        left_target_cps = Convert-ToSigned (Get-WordAt ($baseAddress + 52))
        right_target_cps = Convert-ToSigned (Get-WordAt ($baseAddress + 56))
        line_position_x1000 = Convert-ToSigned (Get-WordAt ($baseAddress + 60))
    }
}

try {
    $rows | Export-Csv -LiteralPath $csvPath -NoTypeInformation -Encoding UTF8
}
catch [System.IO.IOException] {
    $csvPath = Join-Path $projectRoot 'build\Debug\motion-curve-latest.csv'
    $rows | Export-Csv -LiteralPath $csvPath -NoTypeInformation -Encoding UTF8
}

Write-Host ("Motion samples: {0}, overwritten samples: {1}" -f
            $count, $overflowCount)
Write-Host ("Curve CSV: {0}" -f $csvPath)
Write-Host (("Run status: fault={0}, finished={1}, gyro_valid={2}, " +
             "stop_armed={3}, stop_detected={4}") -f
    (Get-Scalar 'g_pi_fault_code'),
    (Get-Scalar 'g_line_follow_finished'),
    (Get-Scalar 'g_lap_gyro_valid'),
    (Get-Scalar 'g_lap_stop_line_armed'),
    (Get-Scalar 'g_lap_stop_line_detected'))
Write-Host (("Lap: elapsed={0} ms, distance={1} mm, " +
             "stop_line={2} mm, roll_after_line={3} mm") -f
    (Get-Scalar 'g_lap_elapsed_ms'),
    ((Convert-ToSigned (Get-Scalar 'g_lap_distance_um')) / 1000.0),
    ((Convert-ToSigned (Get-Scalar 'g_lap_stop_line_distance_um')) /
        1000.0),
    (((Convert-ToSigned (Get-Scalar 'g_lap_distance_um')) -
      (Convert-ToSigned (Get-Scalar 'g_lap_stop_line_distance_um'))) /
        1000.0))
Write-Host (("Heading: gyro={0} deg, signed={1} deg, encoder={2} deg; " +
             "marker active max={3}, stable frames={4}") -f
    ((Convert-ToSigned (Get-Scalar 'g_lap_heading_progress_mdeg')) /
        1000.0),
    ((Convert-ToSigned (Get-Scalar 'g_lap_gyro_heading_signed_mdeg')) /
        1000.0),
    ((Convert-ToSigned (Get-Scalar 'g_lap_encoder_heading_progress_mdeg')) /
        1000.0),
    (Get-Scalar 'g_lap_line_active_count_max'),
    (Get-Scalar 'g_lap_stop_candidate_max_frames'))
Write-Host ("Measured acceleration extrema: {0}..{1} mm/s^2" -f
    (Convert-ToSigned (Get-Scalar 'g_vehicle_acceleration_min_mm_s2')),
    (Convert-ToSigned (Get-Scalar 'g_vehicle_acceleration_max_mm_s2')))
Write-Host ("Straight cruise: n={0}, speed={1}..{2} mm/s, accel={3}..{4} mm/s^2" -f
    (Get-Scalar 'g_cruise_straight_sample_count'),
    (Convert-ToSigned (Get-Scalar 'g_cruise_straight_speed_min_mm_s')),
    (Convert-ToSigned (Get-Scalar 'g_cruise_straight_speed_max_mm_s')),
    (Convert-ToSigned (Get-Scalar 'g_cruise_straight_accel_min_mm_s2')),
    (Convert-ToSigned (Get-Scalar 'g_cruise_straight_accel_max_mm_s2')))
Write-Host ("Turn cruise: n={0}, speed={1}..{2} mm/s, accel={3}..{4} mm/s^2" -f
    (Get-Scalar 'g_cruise_turn_sample_count'),
    (Convert-ToSigned (Get-Scalar 'g_cruise_turn_speed_min_mm_s')),
    (Convert-ToSigned (Get-Scalar 'g_cruise_turn_speed_max_mm_s')),
    (Convert-ToSigned (Get-Scalar 'g_cruise_turn_accel_min_mm_s2')),
    (Convert-ToSigned (Get-Scalar 'g_cruise_turn_accel_max_mm_s2')))
