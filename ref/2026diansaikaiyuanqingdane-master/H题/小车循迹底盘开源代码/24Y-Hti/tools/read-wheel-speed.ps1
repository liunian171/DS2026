$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$firmware = Join-Path $projectRoot 'build\Debug\HtiCar.elf'
$openOcdConfig = Join-Path $projectRoot 'openocd\openocd.cfg'
. (Join-Path $PSScriptRoot 'setup-env.ps1')

if (-not (Test-Path -LiteralPath $firmware)) {
    throw "Firmware not found: $firmware. Run tools/build.ps1 first."
}

$symbols = & arm-none-eabi-nm.exe --defined-only $firmware

function Get-SymbolAddress([string]$name) {
    $match = $symbols | Select-String -Pattern "^([0-9a-fA-F]+)\s+[A-Za-z]\s+$name$" |
        Select-Object -First 1
    if (-not $match) {
        throw "Symbol not found: $name"
    }
    return $match.Matches[0].Groups[1].Value
}

$names = @(
    'g_wheel_speed_sample_count',
    'g_wheel_speed_left_cps',
    'g_wheel_speed_right_cps',
    'g_wheel_speed_left_rpm_x10',
    'g_wheel_speed_right_rpm_x10',
    'g_wheel_speed_left_peak_abs_cps',
    'g_wheel_speed_right_peak_abs_cps',
    'g_speed_test_left_end_count',
    'g_speed_test_right_end_count',
    'g_pi_target_cps',
    'g_pi_left_output_permille',
    'g_pi_right_output_permille',
    'g_pi_fault_code',
    'g_pi_settled_sample_count',
    'g_pi_left_average_cps',
    'g_pi_right_average_cps',
    'g_pi_left_average_output_permille',
    'g_pi_right_average_output_permille',
    'g_chassis_translation_command_cps',
    'g_chassis_turn_command_cps',
    'g_chassis_left_target_cps',
    'g_chassis_right_target_cps',
    'g_chassis_linear_command_mm_s',
    'g_chassis_yaw_command_mdeg_s',
    'g_vehicle_linear_speed_mm_s',
    'g_vehicle_acceleration_raw_mm_s2',
    'g_vehicle_acceleration_mm_s2',
    'g_vehicle_acceleration_max_mm_s2',
    'g_vehicle_acceleration_min_mm_s2',
    'g_vehicle_profile_speed_mm_s',
    'g_vehicle_acceleration_command_mm_s2',
    'g_odometry_distance_um',
    'g_odometry_heading_mdeg',
    'g_distance_target_mm',
    'g_distance_remaining_um',
    'g_distance_finished',
    'g_angle_target_mdeg',
    'g_angle_remaining_mdeg',
    'g_angle_finished',
    'g_gyro_uart_rx_count',
    'g_gyro_uart_overflow_count',
    'g_gyro_uart_error_count',
    'g_cyz_valid_frame_count',
    'g_cyz_telemetry_frame_count',
    'g_cyz_crc_error_count',
    'g_cyz_format_error_count',
    'g_cyz_sequence_drop_count',
    'g_cyz_last_update_ms',
    'g_cyz_angle_mdeg',
    'g_cyz_gyro_mdps',
    'g_cyz_ack_count',
    'g_cyz_last_ack_command',
    'g_cyz_last_ack_result',
    'g_cyz_zero_test_status',
    'g_cyz_chassis_angle_mdeg',
    'g_fused_heading_mdeg',
    'g_heading_hold_target_mdeg',
    'g_heading_hold_error_mdeg',
    'g_heading_hold_correction_mdeg_s',
    'g_arc_radius_mm',
    'g_arc_remaining_mdeg',
    'g_arc_expected_length_um',
    'g_arc_actual_length_um',
    'g_arc_finished',
    'g_line_uart_rx_count',
    'g_line_uart_overflow_count',
    'g_line_uart_error_count',
    'g_line_valid_frame_count',
    'g_line_checksum_error_count',
    'g_line_format_error_count',
    'g_line_last_update_ms',
    'g_line_channel_mask',
    'g_line_detected_mask',
    'g_line_active_count',
    'g_line_lost',
    'g_line_position_x1000',
    'g_line_detection_event_count',
    'g_line_last_detected_mask',
    'g_line_last_detected_position_x1000',
    'g_line_last_detection_ms',
    'g_line_d1',
    'g_line_d2',
    'g_line_d3',
    'g_line_d4',
    'g_line_d5',
    'g_line_d6',
    'g_line_d7',
    'g_line_d8',
    'g_line_follow_yaw_mdeg_s',
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
    'g_line_motion_phase',
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
    'g_cruise_turn_accel_max_mm_s2',
    'g_h_task_number',
    'g_h_task_elapsed_ms',
    'g_h_task_finished',
    'g_h_task_start_distance_um',
    'g_h_task_final_distance_um',
    'g_h_task_final_fused_heading_mdeg',
    'g_h_task_final_odometry_heading_mdeg',
    'g_h_task_phase',
    'g_h_task_phase_progress_mdeg',
    'g_h_task_phase_distance_um',
    'g_h_task_bc_exit_fused_heading_mdeg',
    'g_h_task_bc_exit_odometry_heading_mdeg',
    'g_h_task_bc_exit_progress_mdeg',
    'g_h_task_bc_exit_distance_um',
    'g_h_task_line_centered',
    'g_h_task_arc_gyro_takeover',
    'g_h_task_cross_completed_laps',
    'g_h_task_stall_consecutive_samples',
    'g_h_task_arc_entry_centered',
    'g_h_task_pre_entry_active',
    'g_h_task_preview_yaw_mdeg_s',
    'g_h_task_cross_cb_exit_progress_mdeg',
    'g_h_task_cross_cb_exit_distance_um',
    'g_h_task_cross_da_exit_progress_mdeg',
    'g_h_task_cross_da_exit_distance_um',
    'g_h_task_cross_cb_entry_heading_error_mdeg',
    'g_h_task_cross_da_entry_heading_error_mdeg',
    'g_h_task_cross_cb_entry_line_position_x1000',
    'g_h_task_cross_da_entry_line_position_x1000',
    'g_h_task_arc_takeover_progress_mdeg',
    'g_h_task_arc_tail_distance_um',
    'g_h_task_arc_tail_target_um',
    'g_h_task_arc_exit_reason',
    'g_h_task_diagonal_endpoint_captured',
    'g_h_task_diagonal_compensation_um',
    'g_h_task_da_exit_fused_heading_mdeg',
    'g_h_task_da_exit_odometry_heading_mdeg',
    'g_h_task_da_exit_progress_mdeg',
    'g_h_task_da_exit_distance_um',
    'g_h_task_arc_tail_expected_progress_mdeg',
    'g_h_task_arc_tail_error_mdeg',
    'g_h_task_arc_tail_yaw_mdeg_s',
    'g_h_task_arc_tail_distance_hold',
    'g_h_task_diagonal_tail_slowdown'
)

$byteNames = @(
    'g_cyz_last_ack_command',
    'g_cyz_last_ack_result'
)

$addresses = @{}
foreach ($name in $names) {
    $addresses[$name] = Get-SymbolAddress $name
    Write-Host ("{0,-36} 0x{1}" -f $name, $addresses[$name])
}

$openOcdArguments = @('-f', $openOcdConfig, '-c', 'init')
foreach ($name in $names) {
    if ($byteNames -contains $name) {
        $openOcdArguments += @('-c', "mdb 0x$($addresses[$name]) 1")
    }
    else {
        $openOcdArguments += @('-c', "mdw 0x$($addresses[$name]) 1")
    }
}
$openOcdArguments += @('-c', 'shutdown')

& $env:HTI_OPENOCD_EXE @openOcdArguments
if ($LASTEXITCODE -ne 0) {
    throw "OpenOCD wheel-speed read failed with exit code $LASTEXITCODE."
}
