#include "app_main.h"

#include "app_config.h"
#include "arc_profile.h"
#include "angle_profile.h"
#include "bsp_debug_uart.h"
#include "bsp_encoder.h"
#include "bsp_encoder_config.h"
#include "bsp_gyro_uart.h"
#include "bsp_led.h"
#include "bsp_line_uart.h"
#include "bsp_motor.h"
#include "bsp_stepper_uart.h"
#include "chassis_config.h"
#include "chassis_kinematics.h"
#include "chassis_velocity.h"
#include "cyz_sensor.h"
#include "distance_profile.h"
#include "heading_estimator.h"
#include "heading_controller.h"
#include "main.h"
#include "line_sensor.h"
#include "line_follower.h"
#include "wheel_speed.h"
#include "wheel_speed_pi.h"
#include "zdt_stepper.h"

#define APP_H_TRACK_TASK_ENABLED \
    (APP_H_TASK1_ENABLED || APP_H_TASK2_ENABLED || APP_H_TASK3_ENABLED)

#if ((APP_SPEED_TEST_ENABLED + APP_CLOSED_LOOP_TEST_ENABLED + \
      APP_DIRECTION_DIAGNOSTIC_ENABLED + APP_CHASSIS_MIX_TEST_ENABLED + \
      APP_GROUND_STRAIGHT_TEST_ENABLED + APP_DISTANCE_TEST_ENABLED + \
      APP_ANGLE_TEST_ENABLED + APP_HEADING_STRAIGHT_TEST_ENABLED + \
      APP_ARC_TEST_ENABLED + APP_LINE_FOLLOW_TEST_ENABLED + \
      APP_H_TASK1_ENABLED + APP_H_TASK2_ENABLED + APP_H_TASK3_ENABLED + \
      APP_LINE_POLARITY_TEST_ENABLED + \
      APP_ZDT_STEPPER_TEST_ENABLED) > 1U)
#error "Only one physical motor test may be enabled at a time."
#endif

#if APP_H_TASK3_ENABLED
#define APP_H_ACTIVE_TASK_NUMBER           3U
#define APP_H_ACTIVE_START_DELAY_MS         APP_H_TASK3_START_DELAY_MS
#define APP_H_ACTIVE_TARGET_MM              APP_H_TASK3_AC_APPROACH_MM
#define APP_H_ACTIVE_MAX_SPEED_MM_S         APP_H_TASK3_MAX_SPEED_MM_S
#define APP_H_ACTIVE_MIN_SPEED_MM_S         APP_H_TASK3_MIN_SPEED_MM_S
#define APP_H_ACTIVE_SLOWDOWN_MM            APP_H_TASK3_SLOWDOWN_MM
#define APP_H_ACTIVE_TOLERANCE_MM           APP_H_TASK3_TOLERANCE_MM
#define APP_H_ACTIVE_POST_STOP_MS           APP_H_TASK3_POST_STOP_MS
#define APP_H_ACTIVE_TIMEOUT_MS             APP_H_TASK3_TIMEOUT_MS
#define APP_H_ACTIVE_LINE_KP                APP_H_TASK3_LINE_KP
#define APP_H_ACTIVE_MAX_YAW_MDEG_S         APP_H_TASK3_MAX_LINE_YAW_MDEG_S
#elif APP_H_TASK2_ENABLED
#define APP_H_ACTIVE_TASK_NUMBER           2U
#define APP_H_ACTIVE_START_DELAY_MS         APP_H_TASK2_START_DELAY_MS
#define APP_H_ACTIVE_TARGET_MM              APP_H_TASK2_TARGET_MM
#define APP_H_ACTIVE_MAX_SPEED_MM_S         APP_H_TASK2_MAX_SPEED_MM_S
#define APP_H_ACTIVE_MIN_SPEED_MM_S         APP_H_TASK2_MIN_SPEED_MM_S
#define APP_H_ACTIVE_SLOWDOWN_MM            APP_H_TASK2_SLOWDOWN_MM
#define APP_H_ACTIVE_TOLERANCE_MM           APP_H_TASK2_TOLERANCE_MM
#define APP_H_ACTIVE_POST_STOP_MS           APP_H_TASK2_POST_STOP_MS
#define APP_H_ACTIVE_TIMEOUT_MS             APP_H_TASK2_TIMEOUT_MS
#define APP_H_ACTIVE_LINE_KP                APP_H_TASK2_LINE_KP
#define APP_H_ACTIVE_MAX_YAW_MDEG_S         APP_H_TASK2_MAX_YAW_MDEG_S
#elif APP_H_TASK1_ENABLED
#define APP_H_ACTIVE_TASK_NUMBER           1U
#define APP_H_ACTIVE_START_DELAY_MS         APP_H_TASK1_START_DELAY_MS
#define APP_H_ACTIVE_TARGET_MM              APP_H_TASK1_TARGET_MM
#define APP_H_ACTIVE_MAX_SPEED_MM_S         APP_H_TASK1_MAX_SPEED_MM_S
#define APP_H_ACTIVE_MIN_SPEED_MM_S         APP_H_TASK1_MIN_SPEED_MM_S
#define APP_H_ACTIVE_SLOWDOWN_MM            APP_H_TASK1_SLOWDOWN_MM
#define APP_H_ACTIVE_TOLERANCE_MM           APP_H_TASK1_TOLERANCE_MM
#define APP_H_ACTIVE_POST_STOP_MS           APP_H_TASK1_POST_STOP_MS
#define APP_H_ACTIVE_TIMEOUT_MS             APP_H_TASK1_TIMEOUT_MS
#define APP_H_ACTIVE_LINE_KP                APP_H_TASK1_LINE_KP
#define APP_H_ACTIVE_MAX_YAW_MDEG_S         APP_H_TASK1_MAX_YAW_MDEG_S
#endif

#if APP_H_TRACK_TASK_ENABLED
#define APP_ACTIVE_DISTANCE_DELAY_MS       APP_H_ACTIVE_START_DELAY_MS
#define APP_ACTIVE_DISTANCE_TARGET_MM      APP_H_ACTIVE_TARGET_MM
#define APP_ACTIVE_DISTANCE_MAX_SPEED_MM_S APP_H_ACTIVE_MAX_SPEED_MM_S
#define APP_ACTIVE_DISTANCE_MIN_SPEED_MM_S APP_H_ACTIVE_MIN_SPEED_MM_S
#define APP_ACTIVE_DISTANCE_SLOWDOWN_MM    APP_H_ACTIVE_SLOWDOWN_MM
#define APP_ACTIVE_DISTANCE_TOLERANCE_MM   APP_H_ACTIVE_TOLERANCE_MM
#define APP_ACTIVE_DISTANCE_POST_STOP_MS   APP_H_ACTIVE_POST_STOP_MS
#elif APP_HEADING_STRAIGHT_TEST_ENABLED
#define APP_ACTIVE_DISTANCE_DELAY_MS       APP_HEADING_STRAIGHT_TEST_DELAY_MS
#define APP_ACTIVE_DISTANCE_TARGET_MM      APP_HEADING_STRAIGHT_TARGET_MM
#define APP_ACTIVE_DISTANCE_MAX_SPEED_MM_S APP_HEADING_STRAIGHT_MAX_SPEED_MM_S
#define APP_ACTIVE_DISTANCE_MIN_SPEED_MM_S APP_HEADING_STRAIGHT_MIN_SPEED_MM_S
#define APP_ACTIVE_DISTANCE_SLOWDOWN_MM    APP_HEADING_STRAIGHT_SLOWDOWN_MM
#define APP_ACTIVE_DISTANCE_TOLERANCE_MM   APP_HEADING_STRAIGHT_TOLERANCE_MM
#define APP_ACTIVE_DISTANCE_POST_STOP_MS   APP_HEADING_STRAIGHT_POST_STOP_MS
#else
#define APP_ACTIVE_DISTANCE_DELAY_MS       APP_DISTANCE_TEST_DELAY_MS
#define APP_ACTIVE_DISTANCE_TARGET_MM      APP_DISTANCE_TEST_TARGET_MM
#define APP_ACTIVE_DISTANCE_MAX_SPEED_MM_S APP_DISTANCE_TEST_MAX_SPEED_MM_S
#define APP_ACTIVE_DISTANCE_MIN_SPEED_MM_S APP_DISTANCE_TEST_MIN_SPEED_MM_S
#define APP_ACTIVE_DISTANCE_SLOWDOWN_MM    APP_DISTANCE_TEST_SLOWDOWN_MM
#define APP_ACTIVE_DISTANCE_TOLERANCE_MM   APP_DISTANCE_TEST_TOLERANCE_MM
#define APP_ACTIVE_DISTANCE_POST_STOP_MS   APP_DISTANCE_TEST_POST_STOP_MS
#endif

#if APP_H_TRACK_TASK_ENABLED
#define APP_ACTIVE_LINE_KP             APP_H_ACTIVE_LINE_KP
#define APP_ACTIVE_LINE_MAX_YAW_MDEG_S APP_H_ACTIVE_MAX_YAW_MDEG_S
#else
#define APP_ACTIVE_LINE_KP             APP_LINE_FOLLOW_KP
#define APP_ACTIVE_LINE_MAX_YAW_MDEG_S APP_LINE_FOLLOW_MAX_YAW_MDEG_S
#endif

#if APP_GROUND_STRAIGHT_TEST_ENABLED
#define APP_ACTIVE_MOTION_DELAY_MS  APP_GROUND_TEST_DELAY_MS
#define APP_ACTIVE_MOTION_RUN_MS    APP_GROUND_TEST_RUN_MS
#define APP_ACTIVE_MOTION_STOP_MS   APP_GROUND_TEST_STOP_MS
#define APP_ACTIVE_MOTION_SETTLE_MS APP_GROUND_TEST_SETTLE_MS
#elif APP_CHASSIS_MIX_TEST_ENABLED
#define APP_ACTIVE_MOTION_DELAY_MS  APP_CHASSIS_MIX_TEST_DELAY_MS
#define APP_ACTIVE_MOTION_RUN_MS    APP_CHASSIS_MIX_TEST_RUN_MS
#define APP_ACTIVE_MOTION_STOP_MS   APP_CHASSIS_MIX_TEST_STOP_MS
#define APP_ACTIVE_MOTION_SETTLE_MS APP_CLOSED_LOOP_SETTLE_MS
#endif

typedef enum
{
    APP_TEST_WAITING = 0,
    APP_TEST_RUNNING,
    APP_TEST_FINISHED
} App_TestState;

typedef enum
{
    APP_LINE_MOTION_WAITING = 0,
    APP_LINE_MOTION_ACCELERATING,
    APP_LINE_MOTION_CRUISING,
    APP_LINE_MOTION_APPROACHING,
    APP_LINE_MOTION_STOPPING,
    APP_LINE_MOTION_FINISHED
} App_LineMotionPhase;

typedef struct
{
    uint32_t elapsed_ms;
    uint32_t phase;
    int32_t profile_speed_mm_s;
    int32_t measured_speed_mm_s;
    int32_t acceleration_command_mm_s2;
    int32_t acceleration_measured_mm_s2;
    int32_t yaw_command_mdeg_s;
    int32_t gyro_heading_progress_mdeg;
    int32_t encoder_heading_progress_mdeg;
    int32_t left_output_permille;
    int32_t right_output_permille;
    int32_t left_measured_cps;
    int32_t right_measured_cps;
    int32_t left_target_cps;
    int32_t right_target_cps;
    int32_t line_position_x1000;
} App_MotionLogSample;

typedef enum
{
    APP_H2_PHASE_IDLE = 0,
    APP_H2_PHASE_STRAIGHT_AB,
    APP_H2_PHASE_ACQUIRE_B,
    APP_H2_PHASE_ARC_BC,
    APP_H2_PHASE_STRAIGHT_CD,
    APP_H2_PHASE_ACQUIRE_D,
    APP_H2_PHASE_ARC_DA,
    APP_H2_PHASE_STOPPING
} App_H2Phase;

typedef enum
{
    APP_H3_PHASE_IDLE = 0,
    APP_H3_PHASE_ALIGN_A,
    APP_H3_PHASE_DIAGONAL_AC,
    APP_H3_PHASE_ALIGN_C,
    APP_H3_PHASE_ACQUIRE_C,
    APP_H3_PHASE_ARC_CB,
    APP_H3_PHASE_ALIGN_B,
    APP_H3_PHASE_DIAGONAL_BD,
    APP_H3_PHASE_ALIGN_D,
    APP_H3_PHASE_ACQUIRE_D,
    APP_H3_PHASE_ARC_DA,
    APP_H3_PHASE_STOPPING
} App_H3Phase;

static App_TestState test_state;
static uint32_t state_started_ms;
static uint32_t led_changed_ms;
static int32_t displayed_left_count;
static int32_t displayed_right_count;
static uint32_t encoder_activity_ms;
static bool encoder_led_active;
static WheelSpeedEstimator speed_estimator;
static WheelSpeedPI left_speed_controller;
static WheelSpeedPI right_speed_controller;
static ChassisVelocity chassis_velocity;
static ChassisKinematicsConfig chassis_kinematics;
static ChassisOdometry chassis_odometry;
static DistanceProfile distance_profile;
static uint32_t distance_finished_ms;
static AngleProfile angle_profile;
static ArcProfile arc_profile;
static HeadingEstimator heading_estimator;
static HeadingController heading_controller;
static LineFollower line_follower;
static uint32_t angle_finished_ms;
static int32_t arc_start_distance_um;
static int32_t vehicle_previous_speed_mm_s;
static bool vehicle_acceleration_sample_valid;
static uint32_t lap_line_checked_frame_count;
static uint32_t lap_line_clear_stable_frame_count;
static uint32_t lap_stop_line_stable_frame_count;
static uint32_t lap_stopped_stable_sample_count;
static uint32_t lap_line_lost_started_ms;
static int32_t lap_start_distance_um;
static int32_t lap_start_heading_mdeg;
static int32_t lap_gyro_previous_mdeg;
static int32_t lap_gyro_accumulated_mdeg;
static int32_t line_yaw_command_mdeg_s;
#if APP_LINE_FOLLOW_TEST_ENABLED
static int32_t line_profile_speed_mm_s;
static int32_t line_profile_start_speed_mm_s;
static int32_t line_profile_target_speed_mm_s;
static int32_t line_profile_acceleration_mm_s2;
static uint32_t line_profile_elapsed_ms;
static uint32_t line_profile_duration_ms;
#endif
static uint32_t motion_log_last_ms;
static uint32_t lap_gyro_checked_frame_count;
static bool lap_line_lost_active;
static bool lap_stop_requested;
static App_H2Phase h_task2_phase;
static uint32_t h_task2_phase_started_ms;
static uint32_t h_task2_line_checked_frame_count;
static uint32_t h_task2_line_stable_frame_count;
static uint32_t h_task2_line_candidate_started_ms;
static uint32_t h_task2_line_lost_started_ms;
static uint32_t h_task2_center_checked_frame_count;
static uint32_t h_task2_center_stable_frame_count;
static bool h_task2_arc_entry_centered;
static bool h_task2_arc_gyro_takeover;
static int32_t h_task2_initial_heading_mdeg;
static int32_t h_task2_arc_start_heading_mdeg;
static int32_t h_task2_arc_start_distance_um;
static int32_t h_task2_arc_tail_start_distance_um;
static int32_t h_task2_arc_tail_target_um;
static int32_t h_task2_arc_takeover_progress_mdeg;
static App_H3Phase h_task3_phase;
static uint32_t h_task3_phase_started_ms;
static uint32_t h_task3_line_checked_frame_count;
static uint32_t h_task3_line_stable_frame_count;
static uint32_t h_task3_line_lost_started_ms;
static uint32_t h_task3_center_checked_frame_count;
static uint32_t h_task3_center_stable_frame_count;
static uint32_t h_task3_completed_laps;
static bool h_task3_arc_entry_centered;
static bool h_task3_arc_gyro_takeover;
static bool h_task3_endpoint_line_captured;
static int32_t h_task3_endpoint_line_distance_um;
static uint32_t h_task3_endpoint_checked_frame_count;
static uint32_t h_task3_endpoint_stable_frame_count;
static int32_t h_task3_initial_heading_mdeg;
static int32_t h_task3_arc_start_heading_mdeg;
static int32_t h_task3_arc_start_distance_um;
static int32_t h_task3_arc_tail_start_distance_um;
static int32_t h_task3_arc_tail_target_um;
static int32_t h_task3_arc_takeover_progress_mdeg;
static int64_t pi_left_cps_sum;
static int64_t pi_right_cps_sum;
static int64_t pi_left_output_sum;
static int64_t pi_right_output_sum;
#if APP_ZDT_STEPPER_ENABLED && APP_ZDT_STEPPER_TEST_ENABLED
static uint32_t zdt_stepper_test_phase_started_ms;
#endif

/* These telemetry symbols are intentionally global for OpenOCD inspection. */
volatile uint32_t g_wheel_speed_sample_count;
volatile int32_t g_wheel_speed_left_cps;
volatile int32_t g_wheel_speed_right_cps;
volatile int32_t g_wheel_speed_left_rpm_x10;
volatile int32_t g_wheel_speed_right_rpm_x10;
volatile int32_t g_wheel_speed_left_peak_abs_cps;
volatile int32_t g_wheel_speed_right_peak_abs_cps;
volatile int32_t g_speed_test_left_end_count;
volatile int32_t g_speed_test_right_end_count;
volatile uint32_t g_wheel_speed_elapsed_ms;
volatile int32_t g_pi_target_cps;
volatile int32_t g_pi_left_output_permille;
volatile int32_t g_pi_right_output_permille;
volatile uint32_t g_pi_fault_code;
volatile uint32_t g_pi_settled_sample_count;
volatile int32_t g_pi_left_average_cps;
volatile int32_t g_pi_right_average_cps;
volatile int32_t g_pi_left_average_output_permille;
volatile int32_t g_pi_right_average_output_permille;
volatile int32_t g_chassis_translation_command_cps;
volatile int32_t g_chassis_turn_command_cps;
volatile int32_t g_chassis_left_target_cps;
volatile int32_t g_chassis_right_target_cps;
volatile int32_t g_chassis_linear_command_mm_s;
volatile int32_t g_chassis_yaw_command_mdeg_s;
volatile int32_t g_vehicle_linear_speed_mm_s;
volatile int32_t g_vehicle_acceleration_raw_mm_s2;
volatile int32_t g_vehicle_acceleration_mm_s2;
volatile int32_t g_vehicle_acceleration_max_mm_s2;
volatile int32_t g_vehicle_acceleration_min_mm_s2;
volatile int32_t g_vehicle_profile_speed_mm_s;
volatile int32_t g_vehicle_acceleration_command_mm_s2;
volatile int32_t g_odometry_distance_um;
volatile int32_t g_odometry_heading_mdeg;
volatile int32_t g_distance_target_mm;
volatile int32_t g_distance_remaining_um;
volatile uint32_t g_distance_finished;
volatile int32_t g_angle_target_mdeg;
volatile int32_t g_angle_remaining_mdeg;
volatile uint32_t g_angle_finished;
volatile uint32_t g_cyz_zero_test_status;
volatile int32_t g_cyz_chassis_angle_mdeg;
volatile int32_t g_fused_heading_mdeg;
volatile int32_t g_heading_hold_target_mdeg;
volatile int32_t g_heading_hold_error_mdeg;
volatile int32_t g_heading_hold_correction_mdeg_s;
volatile int32_t g_arc_radius_mm;
volatile int32_t g_arc_remaining_mdeg;
volatile int32_t g_arc_expected_length_um;
volatile int32_t g_arc_actual_length_um;
volatile uint32_t g_arc_finished;
volatile int32_t g_line_follow_yaw_mdeg_s;
volatile uint32_t g_line_follow_finished;
volatile uint32_t g_lap_stop_line_armed;
volatile uint32_t g_lap_stop_line_detected;
volatile uint32_t g_lap_line_active_count_max;
volatile uint32_t g_lap_stop_candidate_max_frames;
volatile uint32_t g_lap_elapsed_ms;
volatile int32_t g_lap_distance_um;
volatile int32_t g_lap_heading_progress_mdeg;
volatile int32_t g_lap_encoder_heading_progress_mdeg;
volatile int32_t g_lap_gyro_heading_signed_mdeg;
volatile uint32_t g_lap_gyro_valid;
volatile int32_t g_lap_stop_line_distance_um;
volatile uint32_t g_line_motion_phase;
volatile App_MotionLogSample g_motion_log[APP_MOTION_LOG_CAPACITY];
volatile uint32_t g_motion_log_count;
volatile uint32_t g_motion_log_write_index;
volatile uint32_t g_motion_log_overflow_count;
volatile uint32_t g_cruise_straight_sample_count;
volatile int32_t g_cruise_straight_speed_min_mm_s;
volatile int32_t g_cruise_straight_speed_max_mm_s;
volatile int32_t g_cruise_straight_accel_min_mm_s2;
volatile int32_t g_cruise_straight_accel_max_mm_s2;
volatile uint32_t g_cruise_turn_sample_count;
volatile int32_t g_cruise_turn_speed_min_mm_s;
volatile int32_t g_cruise_turn_speed_max_mm_s;
volatile int32_t g_cruise_turn_accel_min_mm_s2;
volatile int32_t g_cruise_turn_accel_max_mm_s2;
volatile uint32_t g_zdt_stepper_test_state;
volatile uint32_t g_zdt_stepper_test_elapsed_ms;
volatile uint32_t g_zdt_stepper_test_move_sent;
volatile uint32_t g_zdt_stepper_test_position_requested;
volatile uint32_t g_h_task_number;
volatile uint32_t g_h_task_elapsed_ms;
volatile uint32_t g_h_task_finished;
volatile int32_t g_h_task_start_distance_um;
volatile int32_t g_h_task_final_distance_um;
volatile int32_t g_h_task_final_fused_heading_mdeg;
volatile int32_t g_h_task_final_odometry_heading_mdeg;
volatile uint32_t g_h_task_phase;
volatile int32_t g_h_task_phase_progress_mdeg;
volatile int32_t g_h_task_phase_distance_um;
volatile int32_t g_h_task_bc_exit_fused_heading_mdeg;
volatile int32_t g_h_task_bc_exit_odometry_heading_mdeg;
volatile int32_t g_h_task_bc_exit_progress_mdeg;
volatile int32_t g_h_task_bc_exit_distance_um;
volatile uint32_t g_h_task_line_centered;
volatile uint32_t g_h_task_arc_gyro_takeover;
volatile uint32_t g_h_task_cross_completed_laps;
volatile uint32_t g_h_task_stall_consecutive_samples;
volatile uint32_t g_h_task_arc_entry_centered;
volatile uint32_t g_h_task_pre_entry_active;
volatile int32_t g_h_task_preview_yaw_mdeg_s;
volatile int32_t g_h_task_cross_cb_exit_progress_mdeg;
volatile int32_t g_h_task_cross_cb_exit_distance_um;
volatile int32_t g_h_task_cross_da_exit_progress_mdeg;
volatile int32_t g_h_task_cross_da_exit_distance_um;
volatile int32_t g_h_task_cross_cb_entry_heading_error_mdeg;
volatile int32_t g_h_task_cross_da_entry_heading_error_mdeg;
volatile int32_t g_h_task_cross_cb_entry_line_position_x1000;
volatile int32_t g_h_task_cross_da_entry_line_position_x1000;
volatile int32_t g_h_task_arc_takeover_progress_mdeg;
volatile int32_t g_h_task_arc_tail_distance_um;
volatile int32_t g_h_task_arc_tail_target_um;
volatile uint32_t g_h_task_arc_exit_reason;
volatile uint32_t g_h_task_diagonal_endpoint_captured;
volatile int32_t g_h_task_diagonal_compensation_um;
volatile int32_t g_h_task_da_exit_fused_heading_mdeg;
volatile int32_t g_h_task_da_exit_odometry_heading_mdeg;
volatile int32_t g_h_task_da_exit_progress_mdeg;
volatile int32_t g_h_task_da_exit_distance_um;
volatile int32_t g_h_task_arc_tail_expected_progress_mdeg;
volatile int32_t g_h_task_arc_tail_error_mdeg;
volatile int32_t g_h_task_arc_tail_yaw_mdeg_s;
volatile uint32_t g_h_task_arc_tail_distance_hold;
volatile uint32_t g_h_task_diagonal_tail_slowdown;

static int32_t App_AbsInt32(int32_t value)
{
    return (value < 0) ? -value : value;
}

#if APP_LINE_FOLLOW_TEST_ENABLED
#define APP_LINE_HALF_TURN_MDEG 180000
#define APP_LINE_FULL_TURN_MDEG 360000

static int32_t App_LineWrappedAngleDelta(int32_t current_mdeg,
                                         int32_t previous_mdeg)
{
    int64_t delta_mdeg = (int64_t)current_mdeg - previous_mdeg;

    while (delta_mdeg > APP_LINE_HALF_TURN_MDEG)
    {
        delta_mdeg -= APP_LINE_FULL_TURN_MDEG;
    }
    while (delta_mdeg < -APP_LINE_HALF_TURN_MDEG)
    {
        delta_mdeg += APP_LINE_FULL_TURN_MDEG;
    }
    return (int32_t)delta_mdeg;
}

static int32_t App_LineYawStep(int32_t current_mdeg_s,
                               int32_t target_mdeg_s,
                               uint32_t elapsed_ms)
{
    int32_t maximum_step_mdeg_s = (int32_t)(
        ((int64_t)APP_LINE_FOLLOW_YAW_SLEW_MDEG_S2 * elapsed_ms) /
        1000LL);

    if (maximum_step_mdeg_s < 1)
    {
        maximum_step_mdeg_s = 1;
    }
    if (target_mdeg_s > current_mdeg_s + maximum_step_mdeg_s)
    {
        return current_mdeg_s + maximum_step_mdeg_s;
    }
    if (target_mdeg_s < current_mdeg_s - maximum_step_mdeg_s)
    {
        return current_mdeg_s - maximum_step_mdeg_s;
    }
    return target_mdeg_s;
}

static void App_LineProfileReset(void)
{
    line_profile_speed_mm_s = 0;
    line_profile_start_speed_mm_s = 0;
    line_profile_target_speed_mm_s = 0;
    line_profile_acceleration_mm_s2 = 0;
    line_profile_elapsed_ms = 0U;
    line_profile_duration_ms = 0U;
}

static void App_LineProfileSetTarget(int32_t target_mm_s,
                                     int32_t maximum_accel_mm_s2)
{
    int32_t speed_change_mm_s;

    if (target_mm_s == line_profile_target_speed_mm_s)
    {
        return;
    }
    line_profile_start_speed_mm_s = line_profile_speed_mm_s;
    line_profile_target_speed_mm_s = target_mm_s;
    line_profile_elapsed_ms = 0U;
    speed_change_mm_s = App_AbsInt32(
        target_mm_s - line_profile_start_speed_mm_s);
    if ((speed_change_mm_s == 0) || (maximum_accel_mm_s2 <= 0))
    {
        line_profile_speed_mm_s = target_mm_s;
        line_profile_acceleration_mm_s2 = 0;
        line_profile_duration_ms = 0U;
        return;
    }

    /* Cubic smoothstep has a peak acceleration of 1.5 * delta_v / T. */
    line_profile_duration_ms = (uint32_t)(
        ((int64_t)1500 * speed_change_mm_s +
         maximum_accel_mm_s2 - 1) /
        maximum_accel_mm_s2);
    if (line_profile_duration_ms == 0U)
    {
        line_profile_duration_ms = 1U;
    }
}

static int32_t App_LineProfileUpdate(uint32_t elapsed_ms)
{
    int64_t elapsed;
    int64_t duration;
    int64_t duration_cubed;
    int64_t smooth_numerator;
    int64_t speed_change;

    if (line_profile_duration_ms == 0U)
    {
        line_profile_speed_mm_s = line_profile_target_speed_mm_s;
        line_profile_acceleration_mm_s2 = 0;
        return line_profile_speed_mm_s;
    }

    if (elapsed_ms >= line_profile_duration_ms -
                      line_profile_elapsed_ms)
    {
        line_profile_elapsed_ms = line_profile_duration_ms;
    }
    else
    {
        line_profile_elapsed_ms += elapsed_ms;
    }

    elapsed = line_profile_elapsed_ms;
    duration = line_profile_duration_ms;
    duration_cubed = duration * duration * duration;
    speed_change = (int64_t)line_profile_target_speed_mm_s -
                   line_profile_start_speed_mm_s;
    smooth_numerator = elapsed * elapsed *
                       (3LL * duration - 2LL * elapsed);
    line_profile_speed_mm_s = line_profile_start_speed_mm_s +
        (int32_t)((speed_change * smooth_numerator) / duration_cubed);
    line_profile_acceleration_mm_s2 = (int32_t)(
        (speed_change * 6000LL * elapsed * (duration - elapsed)) /
        duration_cubed);

    if (line_profile_elapsed_ms >= line_profile_duration_ms)
    {
        line_profile_speed_mm_s = line_profile_target_speed_mm_s;
        line_profile_acceleration_mm_s2 = 0;
        line_profile_duration_ms = 0U;
    }
    return line_profile_speed_mm_s;
}

static bool App_LineProfileIsFinished(void)
{
    return line_profile_duration_ms == 0U;
}

static int32_t App_LineFollowerUpdate(int32_t line_position_x1000)
{
    int32_t position_magnitude = App_AbsInt32(line_position_x1000);
    int32_t scheduled_kp;
    int32_t yaw_mdeg_s;

    if (position_magnitude > APP_LINE_FOLLOW_GAIN_FULL_SCALE_X1000)
    {
        position_magnitude = APP_LINE_FOLLOW_GAIN_FULL_SCALE_X1000;
    }
    scheduled_kp = APP_LINE_FOLLOW_CENTER_KP + (int32_t)(
        ((int64_t)(APP_LINE_FOLLOW_KP - APP_LINE_FOLLOW_CENTER_KP) *
         position_magnitude) /
        APP_LINE_FOLLOW_GAIN_FULL_SCALE_X1000);
    yaw_mdeg_s = -scheduled_kp * line_position_x1000;
    if (yaw_mdeg_s > APP_LINE_FOLLOW_MAX_YAW_MDEG_S)
    {
        yaw_mdeg_s = APP_LINE_FOLLOW_MAX_YAW_MDEG_S;
    }
    else if (yaw_mdeg_s < -APP_LINE_FOLLOW_MAX_YAW_MDEG_S)
    {
        yaw_mdeg_s = -APP_LINE_FOLLOW_MAX_YAW_MDEG_S;
    }
    return yaw_mdeg_s;
}

static void App_UpdateMotionRange(
    volatile uint32_t *sample_count,
    volatile int32_t *speed_min_mm_s,
    volatile int32_t *speed_max_mm_s,
    volatile int32_t *accel_min_mm_s2,
    volatile int32_t *accel_max_mm_s2)
{
    if (*sample_count == 0U)
    {
        *speed_min_mm_s = g_vehicle_linear_speed_mm_s;
        *speed_max_mm_s = g_vehicle_linear_speed_mm_s;
        *accel_min_mm_s2 = g_vehicle_acceleration_mm_s2;
        *accel_max_mm_s2 = g_vehicle_acceleration_mm_s2;
    }
    else
    {
        if (g_vehicle_linear_speed_mm_s < *speed_min_mm_s)
        {
            *speed_min_mm_s = g_vehicle_linear_speed_mm_s;
        }
        if (g_vehicle_linear_speed_mm_s > *speed_max_mm_s)
        {
            *speed_max_mm_s = g_vehicle_linear_speed_mm_s;
        }
        if (g_vehicle_acceleration_mm_s2 < *accel_min_mm_s2)
        {
            *accel_min_mm_s2 = g_vehicle_acceleration_mm_s2;
        }
        if (g_vehicle_acceleration_mm_s2 > *accel_max_mm_s2)
        {
            *accel_max_mm_s2 = g_vehicle_acceleration_mm_s2;
        }
    }
    (*sample_count)++;
}

static void App_UpdateMotionTelemetry(uint32_t now_ms,
                                      bool speed_sample_ready)
{
    uint32_t write_index;

    if (test_state != APP_TEST_RUNNING)
    {
        return;
    }

    if (speed_sample_ready &&
        (g_line_motion_phase == APP_LINE_MOTION_CRUISING))
    {
        if (App_AbsInt32(g_line_follow_yaw_mdeg_s) >=
            APP_LINE_TURN_YAW_THRESHOLD_MDEG_S)
        {
            App_UpdateMotionRange(
                &g_cruise_turn_sample_count,
                &g_cruise_turn_speed_min_mm_s,
                &g_cruise_turn_speed_max_mm_s,
                &g_cruise_turn_accel_min_mm_s2,
                &g_cruise_turn_accel_max_mm_s2);
        }
        else
        {
            App_UpdateMotionRange(
                &g_cruise_straight_sample_count,
                &g_cruise_straight_speed_min_mm_s,
                &g_cruise_straight_speed_max_mm_s,
                &g_cruise_straight_accel_min_mm_s2,
                &g_cruise_straight_accel_max_mm_s2);
        }
    }

    if ((uint32_t)(now_ms - motion_log_last_ms) <
        APP_MOTION_LOG_INTERVAL_MS)
    {
        return;
    }
    motion_log_last_ms = now_ms;
    write_index = g_motion_log_write_index;
    g_motion_log[write_index].elapsed_ms = g_lap_elapsed_ms;
    g_motion_log[write_index].phase = g_line_motion_phase;
    g_motion_log[write_index].profile_speed_mm_s =
        g_vehicle_profile_speed_mm_s;
    g_motion_log[write_index].measured_speed_mm_s =
        g_vehicle_linear_speed_mm_s;
    g_motion_log[write_index].acceleration_command_mm_s2 =
        g_vehicle_acceleration_command_mm_s2;
    g_motion_log[write_index].acceleration_measured_mm_s2 =
        g_vehicle_acceleration_mm_s2;
    g_motion_log[write_index].yaw_command_mdeg_s =
        g_line_follow_yaw_mdeg_s;
    g_motion_log[write_index].gyro_heading_progress_mdeg =
        g_lap_heading_progress_mdeg;
    g_motion_log[write_index].encoder_heading_progress_mdeg =
        g_lap_encoder_heading_progress_mdeg;
    g_motion_log[write_index].left_output_permille =
        g_pi_left_output_permille;
    g_motion_log[write_index].right_output_permille =
        g_pi_right_output_permille;
    g_motion_log[write_index].left_measured_cps =
        g_wheel_speed_left_cps;
    g_motion_log[write_index].right_measured_cps =
        g_wheel_speed_right_cps;
    g_motion_log[write_index].left_target_cps =
        g_chassis_left_target_cps;
    g_motion_log[write_index].right_target_cps =
        g_chassis_right_target_cps;
    g_motion_log[write_index].line_position_x1000 =
        (g_line_lost != 0U) ?
        g_line_last_detected_position_x1000 : g_line_position_x1000;

    write_index++;
    if (write_index >= APP_MOTION_LOG_CAPACITY)
    {
        write_index = 0U;
    }
    g_motion_log_write_index = write_index;
    if (g_motion_log_count < APP_MOTION_LOG_CAPACITY)
    {
        g_motion_log_count++;
    }
    else
    {
        g_motion_log_overflow_count++;
    }
}
#endif

#if APP_H_TRACK_TASK_ENABLED
static int32_t App_ClampInt32(int32_t value,
                              int32_t minimum,
                              int32_t maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static int32_t App_GetAcquireSweepYawMdegS(uint32_t elapsed_ms,
                                           int32_t direction,
                                           int32_t nominal_yaw_mdeg_s)
{
    int32_t yaw_magnitude = nominal_yaw_mdeg_s;

    if ((elapsed_ms >= APP_H_ACQUIRE_SWEEP_SHALLOW_START_MS) &&
        (elapsed_ms < APP_H_ACQUIRE_SWEEP_SHARP_START_MS))
    {
        yaw_magnitude = APP_H_ACQUIRE_SWEEP_SHALLOW_YAW_MDEG_S;
    }
    else if ((elapsed_ms >= APP_H_ACQUIRE_SWEEP_SHARP_START_MS) &&
             (elapsed_ms < APP_H_ACQUIRE_SWEEP_NOMINAL_START_MS))
    {
        yaw_magnitude = APP_H_ACQUIRE_SWEEP_SHARP_YAW_MDEG_S;
    }

    return ((direction < 0) ? -1 : 1) * App_AbsInt32(yaw_magnitude);
}

static bool App_HasRecentLineDetection(uint32_t now_ms)
{
    return (g_line_detection_event_count > 0U) &&
           LineSensor_IsFresh(now_ms,
                              APP_LINE_SENSOR_MAXIMUM_AGE_MS) &&
           ((uint32_t)(now_ms - g_line_last_detection_ms) <=
            APP_H_ENTRY_PRECAPTURE_MAX_AGE_MS);
}

static int32_t App_GetArcTailTargetUm(int32_t takeover_progress_mdeg,
                                      int32_t end_progress_mdeg)
{
    int32_t remaining_mdeg =
        end_progress_mdeg - takeover_progress_mdeg;

    if (remaining_mdeg <= 0)
    {
        return 0;
    }

    /* s = r * theta，使用 3.1416 的整数近似，结果单位为 um。 */
    return (int32_t)(((int64_t)APP_H_ARC_NOMINAL_RADIUS_MM *
                      remaining_mdeg * 31416LL) / 1800000LL);
}

static uint32_t App_GetArcExitReason(int32_t progress_mdeg,
                                     int32_t end_progress_mdeg,
                                     int32_t tail_distance_um,
                                     int32_t tail_target_um)
{
    if ((progress_mdeg >= end_progress_mdeg) &&
        (tail_distance_um >= tail_target_um))
    {
        return 1U;
    }
    return 0U;
}

static int32_t App_GetArcTailYawMagnitude(
    int32_t takeover_progress_mdeg,
    int32_t end_progress_mdeg,
    int32_t tail_distance_um,
    int32_t tail_target_um,
    int32_t nominal_yaw_mdeg_s)
{
    int32_t expected_progress_mdeg;
    int32_t progress_error_mdeg;
    int32_t yaw_magnitude;

    if (tail_target_um <= 0)
    {
        expected_progress_mdeg = end_progress_mdeg;
    }
    else if (tail_distance_um >= tail_target_um)
    {
        expected_progress_mdeg = end_progress_mdeg;
    }
    else
    {
        expected_progress_mdeg = takeover_progress_mdeg +
            (int32_t)(((int64_t)(end_progress_mdeg -
                                 takeover_progress_mdeg) *
                       tail_distance_um) / tail_target_um);
    }

    progress_error_mdeg = expected_progress_mdeg -
        g_h_task_phase_progress_mdeg;
    if (progress_error_mdeg < 0)
    {
        progress_error_mdeg = 0;
    }

    yaw_magnitude = App_AbsInt32(nominal_yaw_mdeg_s) +
        progress_error_mdeg * APP_H_ARC_TAIL_HEADING_KP;
    yaw_magnitude = App_ClampInt32(
        yaw_magnitude,
        App_AbsInt32(nominal_yaw_mdeg_s),
        APP_H_ARC_TAIL_MAX_YAW_MDEG_S);

    g_h_task_arc_tail_expected_progress_mdeg =
        expected_progress_mdeg;
    g_h_task_arc_tail_error_mdeg = progress_error_mdeg;
    g_h_task_arc_tail_yaw_mdeg_s = yaw_magnitude;
    return yaw_magnitude;
}
#endif

static bool App_UpdateWheelSpeed(uint32_t now_ms,
                                 int32_t left_count,
                                 int32_t right_count)
{
    WheelSpeedSample sample;
    int32_t left_abs_cps;
    int32_t right_abs_cps;
    int32_t acceleration_raw_mm_s2;

    if (!WheelSpeed_Update(&speed_estimator,
                           left_count,
                           right_count,
                           now_ms,
                           &sample))
    {
        return false;
    }

    g_wheel_speed_left_cps = sample.left_counts_per_second;
    g_wheel_speed_right_cps = sample.right_counts_per_second;
    g_wheel_speed_left_rpm_x10 = sample.left_rpm_x10;
    g_wheel_speed_right_rpm_x10 = sample.right_rpm_x10;
    g_wheel_speed_elapsed_ms = sample.elapsed_ms;
    g_wheel_speed_sample_count++;
    g_vehicle_linear_speed_mm_s =
        ChassisKinematics_WheelCpsToLinearMmS(
            &chassis_kinematics,
            sample.left_counts_per_second,
            sample.right_counts_per_second);
    if (vehicle_acceleration_sample_valid)
    {
        acceleration_raw_mm_s2 = (int32_t)(
            ((int64_t)(g_vehicle_linear_speed_mm_s -
                       vehicle_previous_speed_mm_s) * 1000LL) /
            sample.elapsed_ms);
        g_vehicle_acceleration_raw_mm_s2 = acceleration_raw_mm_s2;
        g_vehicle_acceleration_mm_s2 +=
            (acceleration_raw_mm_s2 - g_vehicle_acceleration_mm_s2) /
            APP_ACCELERATION_FILTER_DIVISOR;
        if (g_vehicle_acceleration_mm_s2 >
            g_vehicle_acceleration_max_mm_s2)
        {
            g_vehicle_acceleration_max_mm_s2 =
                g_vehicle_acceleration_mm_s2;
        }
        if (g_vehicle_acceleration_mm_s2 <
            g_vehicle_acceleration_min_mm_s2)
        {
            g_vehicle_acceleration_min_mm_s2 =
                g_vehicle_acceleration_mm_s2;
        }
    }
    else
    {
        g_vehicle_acceleration_raw_mm_s2 = 0;
        g_vehicle_acceleration_mm_s2 = 0;
        vehicle_acceleration_sample_valid = true;
    }
    vehicle_previous_speed_mm_s = g_vehicle_linear_speed_mm_s;
    ChassisOdometry_Update(&chassis_kinematics,
                           &chassis_odometry,
                           sample.left_delta_counts,
                           sample.right_delta_counts);
    g_odometry_distance_um = chassis_odometry.distance_um;
    g_odometry_heading_mdeg = chassis_odometry.heading_mdeg;

    left_abs_cps = App_AbsInt32(sample.left_counts_per_second);
    right_abs_cps = App_AbsInt32(sample.right_counts_per_second);
    if (left_abs_cps > g_wheel_speed_left_peak_abs_cps)
    {
        g_wheel_speed_left_peak_abs_cps = left_abs_cps;
    }
    if (right_abs_cps > g_wheel_speed_right_peak_abs_cps)
    {
        g_wheel_speed_right_peak_abs_cps = right_abs_cps;
    }
    return true;
}

#if APP_CLOSED_LOOP_TEST_ENABLED
static int32_t App_SpeedAlongTargetDirection(int32_t measured_cps)
{
    return (APP_CLOSED_LOOP_TARGET_CPS < 0) ? -measured_cps : measured_cps;
}

static void App_FinishClosedLoopTest(uint32_t fault_code,
                                     int32_t left_count,
                                     int32_t right_count)
{
    BSP_Motor_CoastAll();
    BSP_LED_Set(false);
    g_pi_left_output_permille = 0;
    g_pi_right_output_permille = 0;
    g_pi_fault_code = fault_code;
    g_speed_test_left_end_count = left_count;
    g_speed_test_right_end_count = right_count;

    if (g_pi_settled_sample_count > 0U)
    {
        g_pi_left_average_cps =
            (int32_t)(pi_left_cps_sum / g_pi_settled_sample_count);
        g_pi_right_average_cps =
            (int32_t)(pi_right_cps_sum / g_pi_settled_sample_count);
        g_pi_left_average_output_permille =
            (int32_t)(pi_left_output_sum / g_pi_settled_sample_count);
        g_pi_right_average_output_permille =
            (int32_t)(pi_right_output_sum / g_pi_settled_sample_count);
    }

    test_state = APP_TEST_FINISHED;
    BSP_DebugUART_Write((fault_code == 0U) ?
                        "Closed-loop speed test finished\r\n" :
                        "Closed-loop speed test stopped by protection\r\n");
}
#endif

#if APP_CHASSIS_MIX_TEST_ENABLED || APP_GROUND_STRAIGHT_TEST_ENABLED || \
    APP_ANGLE_TEST_ENABLED || APP_ARC_TEST_ENABLED || \
    APP_LINE_FOLLOW_TEST_ENABLED || APP_H_TRACK_TASK_ENABLED
static int32_t App_SpeedAlongWheelTarget(int32_t measured_cps,
                                         int32_t target_cps)
{
    return (target_cps < 0) ? -measured_cps : measured_cps;
}
#endif

#if APP_CHASSIS_MIX_TEST_ENABLED || APP_GROUND_STRAIGHT_TEST_ENABLED || \
    APP_DISTANCE_TEST_ENABLED || APP_ANGLE_TEST_ENABLED || \
    APP_HEADING_STRAIGHT_TEST_ENABLED || APP_ARC_TEST_ENABLED || \
    APP_LINE_FOLLOW_TEST_ENABLED || APP_H_TRACK_TASK_ENABLED
static void App_FinishChassisTest(uint32_t fault_code,
                                  int32_t left_count,
                                  int32_t right_count)
{
    BSP_Motor_CoastAll();
    BSP_LED_Set(false);
    g_pi_left_output_permille = 0;
    g_pi_right_output_permille = 0;
    g_pi_fault_code = fault_code;
    g_speed_test_left_end_count = left_count;
    g_speed_test_right_end_count = right_count;

    if (g_pi_settled_sample_count > 0U)
    {
        g_pi_left_average_cps =
            (int32_t)(pi_left_cps_sum / g_pi_settled_sample_count);
        g_pi_right_average_cps =
            (int32_t)(pi_right_cps_sum / g_pi_settled_sample_count);
        g_pi_left_average_output_permille =
            (int32_t)(pi_left_output_sum / g_pi_settled_sample_count);
        g_pi_right_average_output_permille =
            (int32_t)(pi_right_output_sum / g_pi_settled_sample_count);
    }
#if APP_H_TRACK_TASK_ENABLED
    g_h_task_elapsed_ms = HAL_GetTick() - state_started_ms;
    g_h_task_final_distance_um = chassis_odometry.distance_um;
    g_h_task_final_fused_heading_mdeg = g_fused_heading_mdeg;
    g_h_task_final_odometry_heading_mdeg =
        chassis_odometry.heading_mdeg;
    g_h_task_finished = (fault_code == 0U) ? 1U : 0U;
#endif
    test_state = APP_TEST_FINISHED;
}
#endif

#if APP_H_TASK2_ENABLED && APP_H_CHAINED_CROSS_ENABLED
static bool App_UpdateChainedCrossRoute(uint32_t now_ms,
                                        int32_t left_count,
                                        int32_t right_count,
                                        int32_t *linear_mm_s,
                                        int32_t *yaw_mdeg_s)
{
    if (h_task3_phase == APP_H3_PHASE_ALIGN_A)
    {
        int32_t target_heading_mdeg =
            h_task3_initial_heading_mdeg -
            APP_H_TASK3_AC_ENTRY_OFFSET_MDEG;

        if (g_fused_heading_mdeg <=
            target_heading_mdeg + APP_H_TASK3_PIVOT_TOLERANCE_MDEG)
        {
            heading_controller.target_heading_mdeg =
                target_heading_mdeg;
            g_heading_hold_target_mdeg =
                heading_controller.target_heading_mdeg;
            DistanceProfile_Start(&distance_profile,
                                  chassis_odometry.distance_um,
                                  APP_H_TASK3_AC_APPROACH_MM);
            h_task3_endpoint_line_captured = false;
            h_task3_endpoint_line_distance_um = 0;
            h_task3_endpoint_checked_frame_count =
                g_line_valid_frame_count;
            h_task3_endpoint_stable_frame_count = 0U;
            g_h_task_diagonal_endpoint_captured = 0U;
            g_h_task_diagonal_compensation_um = 0;
            h_task3_phase = APP_H3_PHASE_DIAGONAL_AC;
            h_task3_phase_started_ms = now_ms;
        }
        else
        {
            *linear_mm_s = APP_H_TASK3_PIVOT_LINEAR_MM_S;
            *yaw_mdeg_s = -APP_H_TASK3_PIVOT_YAW_MDEG_S;
        }
    }
    else if ((h_task3_phase == APP_H3_PHASE_DIAGONAL_AC) ||
             (h_task3_phase == APP_H3_PHASE_DIAGONAL_BD))
    {
        *yaw_mdeg_s = HeadingController_Update(
            &heading_controller, g_fused_heading_mdeg);
        g_heading_hold_error_mdeg = HeadingController_GetErrorMdeg(
            &heading_controller, g_fused_heading_mdeg);
        g_heading_hold_correction_mdeg_s = *yaw_mdeg_s;

        if (h_task3_endpoint_line_captured)
        {
            int32_t direction =
                (h_task3_phase == APP_H3_PHASE_DIAGONAL_AC) ? 1 : -1;
            int32_t line_position_x1000 =
                (g_line_lost == 0U) ?
                g_line_position_x1000 :
                g_line_last_detected_position_x1000;
            int32_t target_heading_mdeg =
                h_task3_initial_heading_mdeg +
                ((direction > 0) ?
                 APP_H_TASK3_CORNER_ANGLE_MDEG :
                 (180000 + APP_H_TASK3_CORNER_ANGLE_MDEG));

            g_h_task_diagonal_compensation_um =
                chassis_odometry.distance_um -
                h_task3_endpoint_line_distance_um;
            *linear_mm_s = APP_H_TASK3_MAX_SPEED_MM_S;
            g_distance_remaining_um =
                APP_H_LINE_SENSOR_FORWARD_OFFSET_MM * 1000 -
                g_h_task_diagonal_compensation_um;
            /* 扫到 C/D 圆弧后立即由循迹接管，不再直行到车轴交点。 */
            *yaw_mdeg_s = App_ClampInt32(
                direction * APP_H_TASK3_ARC_BASE_YAW_MDEG_S +
                    LineFollower_Update(
                        &line_follower, line_position_x1000),
                -APP_H_TASK3_ARC_MAX_YAW_MDEG_S,
                APP_H_TASK3_ARC_MAX_YAW_MDEG_S);
            g_line_follow_yaw_mdeg_s = *yaw_mdeg_s;

            if (g_distance_remaining_um <= 0)
            {
                /*
                 * 车轴抵达交点时已经处于循迹状态，直接进入圆弧阶段；
                 * 理论切线仍作为陀螺仪弧段进度的零点。
                 */
                h_task3_arc_start_heading_mdeg = target_heading_mdeg;
                h_task3_arc_start_distance_um =
                    chassis_odometry.distance_um;
                h_task3_arc_tail_start_distance_um = 0;
                h_task3_arc_tail_target_um = 0;
                h_task3_arc_takeover_progress_mdeg = 0;
                h_task3_line_lost_started_ms = 0U;
                h_task3_center_checked_frame_count =
                    g_line_valid_frame_count;
                h_task3_center_stable_frame_count = 0U;
                h_task3_arc_entry_centered = false;
                h_task3_arc_gyro_takeover = false;
                g_h_task_line_centered = 0U;
                g_h_task_arc_gyro_takeover = 0U;
                g_h_task_arc_entry_centered = 0U;
                g_h_task_arc_takeover_progress_mdeg = 0;
                g_h_task_arc_tail_distance_um = 0;
                g_h_task_arc_tail_target_um = 0;
                g_h_task_arc_exit_reason = 0U;
                g_h_task_arc_tail_expected_progress_mdeg = 0;
                g_h_task_arc_tail_error_mdeg = 0;
                g_h_task_arc_tail_yaw_mdeg_s = 0;
                g_h_task_arc_tail_distance_hold = 0U;
                h_task3_phase = (direction > 0) ?
                    APP_H3_PHASE_ARC_CB : APP_H3_PHASE_ARC_DA;
                h_task3_phase_started_ms = now_ms;
            }
        }
        else
        {
            *linear_mm_s = DistanceProfile_Update(
                &distance_profile, chassis_odometry.distance_um);
            g_distance_remaining_um = DistanceProfile_GetRemainingUm(
                &distance_profile, chassis_odometry.distance_um);

            if ((g_distance_remaining_um <=
                 APP_H_TASK3_PRE_ENTRY_DISTANCE_MM * 1000) &&
                (*linear_mm_s > APP_H_PRE_ENTRY_SPEED_MM_S))
            {
                *linear_mm_s = APP_H_PRE_ENTRY_SPEED_MM_S;
                g_h_task_pre_entry_active = 1U;
            }

            if (g_distance_remaining_um <=
                APP_H_TASK3_DIAGONAL_TAIL_DISTANCE_MM * 1000)
            {
                *linear_mm_s =
                    APP_H_TASK3_DIAGONAL_TAIL_SPEED_MM_S;
                g_h_task_pre_entry_active = 1U;
                g_h_task_diagonal_tail_slowdown = 1U;
            }

            if ((g_distance_remaining_um <=
                 APP_H_DIAGONAL_LINE_ARM_REMAINING_MM * 1000) &&
                LineSensor_IsFresh(now_ms,
                                   APP_LINE_SENSOR_MAXIMUM_AGE_MS) &&
                (g_line_valid_frame_count !=
                 h_task3_endpoint_checked_frame_count))
            {
                h_task3_endpoint_checked_frame_count =
                    g_line_valid_frame_count;
                if ((g_line_lost == 0U) &&
                    (g_line_detected_mask != 0U))
                {
                    h_task3_endpoint_stable_frame_count++;
                }
                else
                {
                    h_task3_endpoint_stable_frame_count = 0U;
                }
            }

            if ((g_distance_remaining_um <=
                 APP_H_DIAGONAL_LINE_ARM_REMAINING_MM * 1000) &&
                (h_task3_endpoint_stable_frame_count >=
                 APP_H_DIAGONAL_CAPTURE_STABLE_FRAMES))
            {
                /* 先记录传感器过线位置，再补偿到轮轴中心到达端点。 */
                h_task3_endpoint_line_captured = true;
                h_task3_endpoint_line_distance_um =
                    chassis_odometry.distance_um;
                g_h_task_diagonal_endpoint_captured = 1U;
                g_h_task_diagonal_compensation_um = 0;
                g_distance_remaining_um =
                    APP_H_LINE_SENSOR_FORWARD_OFFSET_MM * 1000;

                {
                    int32_t direction =
                        (h_task3_phase == APP_H3_PHASE_DIAGONAL_AC) ?
                        1 : -1;
                    int32_t target_heading_mdeg =
                        h_task3_initial_heading_mdeg +
                        ((direction > 0) ?
                         APP_H_TASK3_CORNER_ANGLE_MDEG :
                         (180000 + APP_H_TASK3_CORNER_ANGLE_MDEG));

                    /* 稳定扫到圆弧的当前点就是统一的圆弧计量起点。 */
                    g_distance_remaining_um = 0;
                    h_task3_arc_start_heading_mdeg =
                        target_heading_mdeg;
                    h_task3_arc_start_distance_um =
                        chassis_odometry.distance_um;
                    h_task3_arc_tail_start_distance_um = 0;
                    h_task3_arc_tail_target_um = 0;
                    h_task3_arc_takeover_progress_mdeg = 0;
                    h_task3_line_lost_started_ms = 0U;
                    h_task3_center_checked_frame_count =
                        g_line_valid_frame_count;
                    h_task3_center_stable_frame_count = 0U;
                    h_task3_arc_entry_centered = false;
                    h_task3_arc_gyro_takeover = false;
                    g_h_task_line_centered = 0U;
                    g_h_task_arc_gyro_takeover = 0U;
                    g_h_task_arc_entry_centered = 0U;
                    g_h_task_arc_takeover_progress_mdeg = 0;
                    g_h_task_arc_tail_distance_um = 0;
                    g_h_task_arc_tail_target_um = 0;
                    g_h_task_arc_exit_reason = 0U;
                    g_h_task_arc_tail_expected_progress_mdeg = 0;
                    g_h_task_arc_tail_error_mdeg = 0;
                    g_h_task_arc_tail_yaw_mdeg_s = 0;
                    g_h_task_arc_tail_distance_hold = 0U;
                    if (direction > 0)
                    {
                        g_h_task_cross_cb_entry_heading_error_mdeg =
                            g_fused_heading_mdeg - target_heading_mdeg;
                        g_h_task_cross_cb_entry_line_position_x1000 =
                            g_line_position_x1000;
                    }
                    else
                    {
                        g_h_task_cross_da_entry_heading_error_mdeg =
                            g_fused_heading_mdeg - target_heading_mdeg;
                        g_h_task_cross_da_entry_line_position_x1000 =
                            g_line_position_x1000;
                    }
                    *linear_mm_s = APP_H_TASK3_ARC_SPEED_MM_S;
                    *yaw_mdeg_s = App_ClampInt32(
                        direction * APP_H_TASK3_ARC_BASE_YAW_MDEG_S +
                            LineFollower_Update(
                                &line_follower,
                                g_line_position_x1000),
                        -APP_H_TASK3_ARC_MAX_YAW_MDEG_S,
                        APP_H_TASK3_ARC_MAX_YAW_MDEG_S);
                    g_line_follow_yaw_mdeg_s = *yaw_mdeg_s;
                    h_task3_phase = (direction > 0) ?
                        APP_H3_PHASE_ARC_CB : APP_H3_PHASE_ARC_DA;
                    h_task3_phase_started_ms = now_ms;
                }
            }
            else if (DistanceProfile_IsFinished(&distance_profile))
            {
                h_task3_phase =
                    (h_task3_phase == APP_H3_PHASE_DIAGONAL_AC) ?
                    APP_H3_PHASE_ALIGN_C : APP_H3_PHASE_ALIGN_D;
                h_task3_phase_started_ms = now_ms;
            }
        }
    }
    else if (h_task3_phase == APP_H3_PHASE_ALIGN_C)
    {
        int32_t target_heading = h_task3_initial_heading_mdeg +
            APP_H_TASK3_CORNER_ANGLE_MDEG;

        if (g_fused_heading_mdeg >=
            target_heading - APP_H_TASK3_PIVOT_TOLERANCE_MDEG)
        {
            h_task3_phase_started_ms = now_ms;
            /* 使用理论切线航向作为圆弧零点，消除转向容差带来的累计误差。 */
            h_task3_arc_start_heading_mdeg = target_heading;
            h_task3_arc_start_distance_um =
                chassis_odometry.distance_um;
            h_task3_arc_tail_start_distance_um = 0;
            h_task3_arc_tail_target_um = 0;
            h_task3_arc_takeover_progress_mdeg = 0;
            g_h_task_arc_takeover_progress_mdeg = 0;
            g_h_task_arc_tail_distance_um = 0;
            g_h_task_arc_tail_target_um = 0;
            g_h_task_arc_exit_reason = 0U;
            g_h_task_arc_tail_expected_progress_mdeg = 0;
            g_h_task_arc_tail_error_mdeg = 0;
            g_h_task_arc_tail_yaw_mdeg_s = 0;
            g_h_task_arc_tail_distance_hold = 0U;
            h_task3_line_lost_started_ms = 0U;
            if (App_HasRecentLineDetection(now_ms))
            {
                h_task3_phase = APP_H3_PHASE_ARC_CB;
                h_task3_center_checked_frame_count =
                    g_line_valid_frame_count;
                h_task3_center_stable_frame_count = 0U;
                h_task3_arc_entry_centered = false;
                h_task3_arc_gyro_takeover = false;
                g_h_task_line_centered = 0U;
                g_h_task_arc_gyro_takeover = 0U;
                g_h_task_arc_entry_centered = 0U;
                *linear_mm_s = APP_H_ARC_ENTRY_SPEED_MM_S;
                *yaw_mdeg_s = APP_H_TASK3_ARC_BASE_YAW_MDEG_S +
                    LineFollower_Update(
                        &line_follower,
                        g_line_last_detected_position_x1000);
            }
            else
            {
                h_task3_phase = APP_H3_PHASE_ACQUIRE_C;
                h_task3_line_checked_frame_count =
                    g_line_detection_event_count;
                h_task3_line_stable_frame_count = 0U;
                *linear_mm_s = APP_H_TASK3_ACQUIRE_SPEED_MM_S;
                *yaw_mdeg_s = APP_H_TASK3_ACQUIRE_YAW_MDEG_S;
            }
        }
        else
        {
            *linear_mm_s = APP_H_TASK3_PIVOT_LINEAR_MM_S;
            *yaw_mdeg_s = APP_H_TASK3_PIVOT_YAW_MDEG_S;
        }
    }
    else if (h_task3_phase == APP_H3_PHASE_ALIGN_B)
    {
        int32_t target_heading = h_task3_initial_heading_mdeg +
            180000 + (2 * APP_H_TASK3_CORNER_ANGLE_MDEG);

        if (g_fused_heading_mdeg >=
            target_heading - APP_H_TASK3_PIVOT_TOLERANCE_MDEG)
        {
            DistanceProfile_Start(&distance_profile,
                                  chassis_odometry.distance_um,
                                  APP_H_TASK3_BD_APPROACH_MM);
            h_task3_endpoint_line_captured = false;
            h_task3_endpoint_line_distance_um = 0;
            h_task3_endpoint_checked_frame_count =
                g_line_valid_frame_count;
            h_task3_endpoint_stable_frame_count = 0U;
            g_h_task_diagonal_endpoint_captured = 0U;
            g_h_task_diagonal_compensation_um = 0;
            h_task3_phase = APP_H3_PHASE_DIAGONAL_BD;
            h_task3_phase_started_ms = now_ms;
            heading_controller.target_heading_mdeg = target_heading;
            g_heading_hold_target_mdeg = target_heading;
        }
        else
        {
            *linear_mm_s = APP_H_TASK3_PIVOT_LINEAR_MM_S;
            *yaw_mdeg_s = APP_H_TASK3_PIVOT_YAW_MDEG_S;
        }
    }
    else if (h_task3_phase == APP_H3_PHASE_ALIGN_D)
    {
        int32_t target_heading = h_task3_initial_heading_mdeg +
            180000 + APP_H_TASK3_CORNER_ANGLE_MDEG;

        if (g_fused_heading_mdeg <=
            target_heading + APP_H_TASK3_PIVOT_TOLERANCE_MDEG)
        {
            h_task3_phase_started_ms = now_ms;
            /* C->B 与 D->A 使用同一理论切线参考，保证镜像圆弧一致。 */
            h_task3_arc_start_heading_mdeg = target_heading;
            h_task3_arc_start_distance_um =
                chassis_odometry.distance_um;
            h_task3_arc_tail_start_distance_um = 0;
            h_task3_arc_tail_target_um = 0;
            h_task3_arc_takeover_progress_mdeg = 0;
            g_h_task_arc_takeover_progress_mdeg = 0;
            g_h_task_arc_tail_distance_um = 0;
            g_h_task_arc_tail_target_um = 0;
            g_h_task_arc_exit_reason = 0U;
            g_h_task_arc_tail_expected_progress_mdeg = 0;
            g_h_task_arc_tail_error_mdeg = 0;
            g_h_task_arc_tail_yaw_mdeg_s = 0;
            g_h_task_arc_tail_distance_hold = 0U;
            h_task3_line_lost_started_ms = 0U;
            if (App_HasRecentLineDetection(now_ms))
            {
                h_task3_phase = APP_H3_PHASE_ARC_DA;
                h_task3_center_checked_frame_count =
                    g_line_valid_frame_count;
                h_task3_center_stable_frame_count = 0U;
                h_task3_arc_entry_centered = false;
                h_task3_arc_gyro_takeover = false;
                g_h_task_line_centered = 0U;
                g_h_task_arc_gyro_takeover = 0U;
                g_h_task_arc_entry_centered = 0U;
                *linear_mm_s = APP_H_ARC_ENTRY_SPEED_MM_S;
                *yaw_mdeg_s = -APP_H_TASK3_ARC_BASE_YAW_MDEG_S +
                    LineFollower_Update(
                        &line_follower,
                        g_line_last_detected_position_x1000);
            }
            else
            {
                h_task3_phase = APP_H3_PHASE_ACQUIRE_D;
                h_task3_line_checked_frame_count =
                    g_line_detection_event_count;
                h_task3_line_stable_frame_count = 0U;
                *linear_mm_s = APP_H_TASK3_ACQUIRE_SPEED_MM_S;
                *yaw_mdeg_s = -APP_H_TASK3_ACQUIRE_YAW_MDEG_S;
            }
        }
        else
        {
            *linear_mm_s = APP_H_TASK3_PIVOT_LINEAR_MM_S;
            *yaw_mdeg_s = -APP_H_TASK3_PIVOT_YAW_MDEG_S;
        }
    }
    else if ((h_task3_phase == APP_H3_PHASE_ACQUIRE_C) ||
             (h_task3_phase == APP_H3_PHASE_ACQUIRE_D))
    {
        int32_t direction =
            (h_task3_phase == APP_H3_PHASE_ACQUIRE_C) ? 1 : -1;

        *linear_mm_s = APP_H_TASK3_ACQUIRE_SPEED_MM_S;
        *yaw_mdeg_s = App_GetAcquireSweepYawMdegS(
            (uint32_t)(now_ms - h_task3_phase_started_ms),
            direction,
            APP_H_TASK3_ACQUIRE_YAW_MDEG_S);
        g_h_task_phase_progress_mdeg = direction *
            (g_fused_heading_mdeg - h_task3_arc_start_heading_mdeg);
        g_h_task_phase_distance_um = chassis_odometry.distance_um -
            h_task3_arc_start_distance_um;

        if (LineSensor_IsFresh(now_ms,
                               APP_LINE_SENSOR_MAXIMUM_AGE_MS))
        {
            if (g_line_detection_event_count !=
                h_task3_line_checked_frame_count)
            {
                h_task3_line_checked_frame_count =
                    g_line_detection_event_count;
                h_task3_line_stable_frame_count++;
            }

            if (h_task3_line_stable_frame_count >=
                APP_H_TASK3_LINE_STABLE_FRAMES)
            {
                h_task3_phase =
                    (h_task3_phase == APP_H3_PHASE_ACQUIRE_C) ?
                    APP_H3_PHASE_ARC_CB : APP_H3_PHASE_ARC_DA;
                h_task3_phase_started_ms = now_ms;
                h_task3_line_stable_frame_count = 0U;
                h_task3_line_lost_started_ms = 0U;
                h_task3_center_checked_frame_count =
                    g_line_valid_frame_count;
                h_task3_center_stable_frame_count = 0U;
                h_task3_arc_entry_centered = false;
                h_task3_arc_gyro_takeover = false;
                g_h_task_line_centered = 0U;
                g_h_task_arc_gyro_takeover = 0U;
                g_h_task_arc_entry_centered = 0U;
                *linear_mm_s = APP_H_ARC_ENTRY_SPEED_MM_S;
                *yaw_mdeg_s = direction *
                    APP_H_TASK3_ARC_BASE_YAW_MDEG_S +
                    LineFollower_Update(&line_follower,
                                        g_line_last_detected_position_x1000);
            }
        }
        else
        {
            h_task3_line_checked_frame_count =
                g_line_detection_event_count;
            h_task3_line_stable_frame_count = 0U;
        }

        if (((h_task3_phase == APP_H3_PHASE_ACQUIRE_C) ||
             (h_task3_phase == APP_H3_PHASE_ACQUIRE_D)) &&
            ((uint32_t)(now_ms - h_task3_phase_started_ms) >=
             APP_H_TASK3_ACQUIRE_TIMEOUT_MS))
        {
            App_FinishChassisTest(5U, left_count, right_count);
            return false;
        }
    }
    else if ((h_task3_phase == APP_H3_PHASE_ARC_CB) ||
             (h_task3_phase == APP_H3_PHASE_ARC_DA))
    {
        int32_t direction =
            (h_task3_phase == APP_H3_PHASE_ARC_CB) ? 1 : -1;
        bool arc_complete = false;
        bool line_available = true;

        g_h_task_phase_progress_mdeg = direction *
            (g_fused_heading_mdeg - h_task3_arc_start_heading_mdeg);
        g_h_task_phase_distance_um = chassis_odometry.distance_um -
            h_task3_arc_start_distance_um;

        if (!h_task3_arc_entry_centered &&
            LineSensor_IsFresh(now_ms,
                               APP_LINE_SENSOR_MAXIMUM_AGE_MS) &&
            (g_line_lost == 0U) &&
            (g_line_detected_mask != 0U) &&
            ((g_line_detected_mask &
              ~APP_H_TASK2_CENTER_DETECTED_MASK) == 0U))
        {
            h_task3_arc_entry_centered = true;
        }
        g_h_task_arc_entry_centered =
            h_task3_arc_entry_centered ? 1U : 0U;

        if ((g_h_task_phase_progress_mdeg >=
             APP_H_TASK2_CENTERING_START_MDEG) &&
            LineSensor_IsFresh(now_ms,
                               APP_LINE_SENSOR_MAXIMUM_AGE_MS) &&
            (g_line_lost == 0U))
        {
            bool center_detected_now =
                (g_line_detected_mask != 0U) &&
                ((g_line_detected_mask &
                  ~APP_H_TASK2_CENTER_DETECTED_MASK) == 0U);

            if (g_line_valid_frame_count !=
                h_task3_center_checked_frame_count)
            {
                h_task3_center_checked_frame_count =
                    g_line_valid_frame_count;
                if (center_detected_now)
                {
                    h_task3_center_stable_frame_count++;
                }
                else
                {
                    h_task3_center_stable_frame_count = 0U;
                    if (!h_task3_arc_gyro_takeover)
                    {
                        g_h_task_line_centered = 0U;
                    }
                }
            }
        }
        else if (g_h_task_phase_progress_mdeg >=
                 APP_H_TASK2_CENTERING_START_MDEG)
        {
            h_task3_center_stable_frame_count = 0U;
        }

        if (h_task3_center_stable_frame_count >=
            APP_H_TASK2_CENTERING_STABLE_FRAMES)
        {
            g_h_task_line_centered = 1U;
        }
        if (!h_task3_arc_gyro_takeover &&
            (g_h_task_phase_progress_mdeg >=
             APP_H_TASK3_ARC_GYRO_TAKEOVER_MDEG) &&
            (g_h_task_line_centered != 0U))
        {
            h_task3_arc_gyro_takeover = true;
            h_task3_arc_tail_start_distance_um =
                chassis_odometry.distance_um;
            h_task3_arc_takeover_progress_mdeg =
                g_h_task_phase_progress_mdeg;
            h_task3_arc_tail_target_um = App_GetArcTailTargetUm(
                h_task3_arc_takeover_progress_mdeg,
                APP_H_TASK3_ARC_END_PROGRESS_MDEG);
            g_h_task_arc_takeover_progress_mdeg =
                h_task3_arc_takeover_progress_mdeg;
            g_h_task_arc_tail_target_um =
                h_task3_arc_tail_target_um;
        }
        g_h_task_arc_gyro_takeover =
            h_task3_arc_gyro_takeover ? 1U : 0U;

        if (h_task3_arc_gyro_takeover)
        {
            g_h_task_arc_tail_distance_um =
                chassis_odometry.distance_um -
                h_task3_arc_tail_start_distance_um;
            g_h_task_arc_exit_reason = App_GetArcExitReason(
                g_h_task_phase_progress_mdeg,
                APP_H_TASK3_ARC_END_PROGRESS_MDEG,
                g_h_task_arc_tail_distance_um,
                h_task3_arc_tail_target_um);
            arc_complete = (g_h_task_arc_exit_reason != 0U);
        }
        if (!arc_complete && !h_task3_arc_gyro_takeover &&
                 (g_h_task_phase_progress_mdeg >=
                  APP_H_TASK2_CENTERING_MAX_MDEG))
        {
            App_FinishChassisTest(5U, left_count, right_count);
            return false;
        }
        else if (h_task3_arc_gyro_takeover)
        {
            line_available = false;
            h_task3_line_lost_started_ms = 0U;
        }
        else if (!LineSensor_IsFresh(now_ms,
                                     APP_LINE_SENSOR_MAXIMUM_AGE_MS))
        {
            App_FinishChassisTest(4U, left_count, right_count);
            return false;
        }
        else if (g_line_lost != 0U)
        {
            if (h_task3_line_lost_started_ms == 0U)
            {
                h_task3_line_lost_started_ms = now_ms;
            }
            else if ((uint32_t)(now_ms -
                      h_task3_line_lost_started_ms) >=
                     (((uint32_t)(now_ms -
                        h_task3_phase_started_ms) <
                       APP_H_ENTRY_CAPTURE_PHASE_MS) ?
                      APP_H_ENTRY_CAPTURE_LOST_GRACE_MS :
                      APP_H_TASK3_LINE_LOST_GRACE_MS))
            {
                App_FinishChassisTest(5U, left_count, right_count);
                return false;
            }
        }
        else
        {
            h_task3_line_lost_started_ms = 0U;
        }

        if (arc_complete)
        {
            if (h_task3_phase == APP_H3_PHASE_ARC_CB)
            {
                g_h_task_cross_cb_exit_progress_mdeg =
                    g_h_task_phase_progress_mdeg;
                g_h_task_cross_cb_exit_distance_um =
                    g_h_task_phase_distance_um;
                h_task3_phase = APP_H3_PHASE_ALIGN_B;
                h_task3_phase_started_ms = now_ms;
                DistanceProfile_Start(&distance_profile,
                                      chassis_odometry.distance_um,
                                      APP_H_TASK3_BD_APPROACH_MM);
                *linear_mm_s = APP_H_TASK3_PIVOT_LINEAR_MM_S;
                *yaw_mdeg_s = APP_H_TASK3_PIVOT_YAW_MDEG_S;
            }
            else
            {
                g_h_task_cross_da_exit_progress_mdeg =
                    g_h_task_phase_progress_mdeg;
                g_h_task_cross_da_exit_distance_um =
                    g_h_task_phase_distance_um;
                h_task3_completed_laps++;
                g_h_task_cross_completed_laps =
                    h_task3_completed_laps;
                if (h_task3_completed_laps <
                    APP_H_CHAINED_CROSS_LOOPS)
                {
                    h_task3_phase = APP_H3_PHASE_ALIGN_A;
                    h_task3_phase_started_ms = now_ms;
                    *linear_mm_s = APP_H_TASK3_PIVOT_LINEAR_MM_S;
                    *yaw_mdeg_s = -APP_H_TASK3_PIVOT_YAW_MDEG_S;
                }
                else
                {
                    h_task3_phase = APP_H3_PHASE_STOPPING;
                    h_task3_phase_started_ms = now_ms;
                    g_distance_finished = 1U;
                    distance_finished_ms = now_ms;
                }
            }
        }
        else
        {
            int32_t line_correction_mdeg_s = 0;

            *linear_mm_s = line_available ?
                APP_H_TASK3_ARC_SPEED_MM_S :
                APP_H_TASK3_ACQUIRE_SPEED_MM_S;
            *yaw_mdeg_s = direction *
                (line_available ?
                 APP_H_TASK3_ARC_BASE_YAW_MDEG_S :
                 APP_H_TASK3_ACQUIRE_YAW_MDEG_S);
            if (line_available)
            {
                int32_t line_distance =
                    App_AbsInt32(g_line_position_x1000);

                if (line_distance >=
                    APP_H_TASK3_LINE_EDGE_POSITION_X1000)
                {
                    *linear_mm_s =
                        APP_H_TASK3_LINE_EDGE_SPEED_MM_S;
                }
                else if (line_distance >=
                         APP_H_TASK3_LINE_MID_POSITION_X1000)
                {
                    *linear_mm_s =
                        APP_H_TASK3_LINE_MID_SPEED_MM_S;
                }
                line_correction_mdeg_s = LineFollower_Update(
                    &line_follower, g_line_position_x1000);
                if ((g_h_task_phase_progress_mdeg >=
                     APP_H_TASK2_CENTERING_START_MDEG) &&
                    (g_h_task_line_centered == 0U))
                {
                    *linear_mm_s =
                        APP_H_TASK2_CENTERING_SPEED_MM_S;
                    line_correction_mdeg_s =
                        (line_correction_mdeg_s *
                         APP_H_TASK2_CENTERING_GAIN_NUMERATOR) /
                        APP_H_TASK2_CENTERING_GAIN_DENOMINATOR;
                }
                *yaw_mdeg_s += line_correction_mdeg_s;
                if (g_h_task_phase_progress_mdeg >=
                    APP_H_TASK2_CENTERING_START_MDEG)
                {
                    *linear_mm_s = APP_H_ARC_EXIT_SPEED_MM_S;
                }
                else if (h_task3_arc_entry_centered)
                {
                    *linear_mm_s = APP_H_TASK3_ARC_SPEED_MM_S;
                }
                else
                {
                    *linear_mm_s = APP_H_ARC_ENTRY_SPEED_MM_S;
                }
            }
            else
            {
                int32_t tail_yaw_magnitude =
                    App_GetArcTailYawMagnitude(
                        h_task3_arc_takeover_progress_mdeg,
                        APP_H_TASK3_ARC_END_PROGRESS_MDEG,
                        g_h_task_arc_tail_distance_um,
                        h_task3_arc_tail_target_um,
                        APP_H_TASK3_ACQUIRE_YAW_MDEG_S);

                if (g_h_task_arc_tail_distance_um >=
                    h_task3_arc_tail_target_um)
                {
                    *linear_mm_s = 0;
                    g_h_task_arc_tail_distance_hold = 1U;
                }
                else
                {
                    *linear_mm_s = APP_H_TASK3_ACQUIRE_SPEED_MM_S;
                }

                *yaw_mdeg_s =
                    (g_h_task_phase_progress_mdeg >=
                     APP_H_TASK3_ARC_END_PROGRESS_MDEG) ?
                    0 : direction * tail_yaw_magnitude;
            }

            /* At arc entry the line can still be outside D4/D5 after
               accumulated position drift. Allow the follower to steer
               briefly across the nominal arc direction until centered;
               then lock the turn direction for the rest of the arc. */
            if (!line_available)
            {
                /* 尾段允许线速度或角速度单独归零，使距离和航向分别收敛。 */
            }
            else if (!h_task3_arc_entry_centered)
            {
                *yaw_mdeg_s = App_ClampInt32(
                    *yaw_mdeg_s,
                    -APP_H_TASK3_ARC_MAX_YAW_MDEG_S,
                    APP_H_TASK3_ARC_MAX_YAW_MDEG_S);
            }
            else if (direction > 0)
            {
                *yaw_mdeg_s = App_ClampInt32(
                    *yaw_mdeg_s, 5000,
                    APP_H_TASK3_ARC_MAX_YAW_MDEG_S);
            }
            else
            {
                *yaw_mdeg_s = App_ClampInt32(
                    *yaw_mdeg_s,
                    -APP_H_TASK3_ARC_MAX_YAW_MDEG_S, -5000);
            }
        }
    }

    return true;
}
#endif

#if APP_ZDT_STEPPER_ENABLED && APP_ZDT_STEPPER_TEST_ENABLED
enum
{
    APP_ZDT_TEST_UNAVAILABLE = 0U,
    APP_ZDT_TEST_WAITING,
    APP_ZDT_TEST_ENABLE_SETTLE,
    APP_ZDT_TEST_MOVING,
    APP_ZDT_TEST_WAITING_POSITION,
    APP_ZDT_TEST_FINISHED,
    APP_ZDT_TEST_POSITION_TIMEOUT
};

static void App_ZDTStepperTestUpdate(uint32_t now_ms)
{
    g_zdt_stepper_test_elapsed_ms =
        now_ms - zdt_stepper_test_phase_started_ms;

    if (g_zdt_stepper_test_state == APP_ZDT_TEST_WAITING)
    {
        if ((g_zdt_stepper_test_elapsed_ms >=
             APP_ZDT_STEPPER_TEST_DELAY_MS) &&
            ZDT_Stepper_Enable(true))
        {
            g_zdt_stepper_test_state = APP_ZDT_TEST_ENABLE_SETTLE;
            zdt_stepper_test_phase_started_ms = now_ms;
            g_zdt_stepper_test_elapsed_ms = 0U;
        }
    }
    else if (g_zdt_stepper_test_state == APP_ZDT_TEST_ENABLE_SETTLE)
    {
        if ((g_zdt_stepper_test_elapsed_ms >=
             APP_ZDT_STEPPER_TEST_ENABLE_SETTLE_MS) &&
            ZDT_Stepper_MoveRelativePulses(
                APP_ZDT_STEPPER_TEST_RELATIVE_PULSES,
                APP_ZDT_STEPPER_TEST_SPEED_RPM,
                APP_ZDT_STEPPER_TEST_ACCELERATION))
        {
            g_zdt_stepper_test_state = APP_ZDT_TEST_MOVING;
            g_zdt_stepper_test_move_sent = 1U;
            zdt_stepper_test_phase_started_ms = now_ms;
            g_zdt_stepper_test_elapsed_ms = 0U;
        }
    }
    else if (g_zdt_stepper_test_state == APP_ZDT_TEST_MOVING)
    {
        if ((g_zdt_stepper_test_elapsed_ms >=
             APP_ZDT_STEPPER_TEST_POSITION_DELAY_MS) &&
            ZDT_Stepper_RequestPosition())
        {
            g_zdt_stepper_test_state = APP_ZDT_TEST_WAITING_POSITION;
            g_zdt_stepper_test_position_requested = 1U;
            zdt_stepper_test_phase_started_ms = now_ms;
            g_zdt_stepper_test_elapsed_ms = 0U;
        }
    }
    else if (g_zdt_stepper_test_state ==
             APP_ZDT_TEST_WAITING_POSITION)
    {
        if (g_zdt_stepper_position_valid != 0U)
        {
            g_zdt_stepper_test_state = APP_ZDT_TEST_FINISHED;
        }
        else if (g_zdt_stepper_test_elapsed_ms >=
                 APP_ZDT_STEPPER_TEST_REPLY_TIMEOUT_MS)
        {
            g_zdt_stepper_test_state = APP_ZDT_TEST_POSITION_TIMEOUT;
        }
    }
}
#endif

void App_Init(void)
{
    BSP_LED_Set(false);
    if (!BSP_Motor_Init())
    {
        BSP_Motor_CoastAll();
        Error_Handler();
    }
    if (!BSP_Encoder_Init())
    {
        BSP_Motor_CoastAll();
        Error_Handler();
    }
#if !APP_CYZ_USE_I2C
    if (!BSP_GyroUART_Init())
    {
        BSP_Motor_CoastAll();
        Error_Handler();
    }
#endif
    if (!CYZ_Sensor_Init(APP_CYZ_USE_I2C ?
                         CYZ_INTERFACE_I2C : CYZ_INTERFACE_UART))
    {
        BSP_Motor_CoastAll();
        Error_Handler();
    }
    if (!BSP_LineUART_Init())
    {
        BSP_Motor_CoastAll();
        Error_Handler();
    }
    if (!LineSensor_Init())
    {
        BSP_Motor_CoastAll();
        Error_Handler();
    }
#if APP_ZDT_STEPPER_ENABLED
    g_zdt_stepper_test_state = 0U;
    g_zdt_stepper_test_elapsed_ms = 0U;
    g_zdt_stepper_test_move_sent = 0U;
    g_zdt_stepper_test_position_requested = 0U;
    if (BSP_StepperUART_Init() &&
        ZDT_Stepper_Init(
            APP_ZDT_STEPPER_ADDRESS,
            APP_ZDT_STEPPER_PULSES_PER_REVOLUTION))
    {
#if APP_ZDT_STEPPER_TEST_ENABLED
        g_zdt_stepper_test_state = APP_ZDT_TEST_WAITING;
        zdt_stepper_test_phase_started_ms = HAL_GetTick();
#endif
    }
#endif

    test_state = APP_TEST_WAITING;
    state_started_ms = HAL_GetTick();
    led_changed_ms = state_started_ms;
    displayed_left_count = 0;
    displayed_right_count = 0;
    encoder_activity_ms = state_started_ms;
    encoder_led_active = false;
    g_wheel_speed_sample_count = 0U;
    g_wheel_speed_left_cps = 0;
    g_wheel_speed_right_cps = 0;
    g_wheel_speed_left_rpm_x10 = 0;
    g_wheel_speed_right_rpm_x10 = 0;
    g_wheel_speed_left_peak_abs_cps = 0;
    g_wheel_speed_right_peak_abs_cps = 0;
    g_speed_test_left_end_count = 0;
    g_speed_test_right_end_count = 0;
    g_wheel_speed_elapsed_ms = 0U;
    g_pi_target_cps = APP_CLOSED_LOOP_TARGET_CPS;
    g_pi_left_output_permille = 0;
    g_pi_right_output_permille = 0;
    g_pi_fault_code = 0U;
    g_pi_settled_sample_count = 0U;
    g_pi_left_average_cps = 0;
    g_pi_right_average_cps = 0;
    g_pi_left_average_output_permille = 0;
    g_pi_right_average_output_permille = 0;
    g_chassis_translation_command_cps = APP_CHASSIS_TEST_TRANSLATION_CPS;
    g_chassis_turn_command_cps = APP_CHASSIS_TEST_TURN_CPS;
    g_chassis_left_target_cps = 0;
    g_chassis_right_target_cps = 0;
    g_chassis_linear_command_mm_s = 0;
    g_chassis_yaw_command_mdeg_s = 0;
    g_vehicle_linear_speed_mm_s = 0;
    g_vehicle_acceleration_raw_mm_s2 = 0;
    g_vehicle_acceleration_mm_s2 = 0;
    g_vehicle_acceleration_max_mm_s2 = 0;
    g_vehicle_acceleration_min_mm_s2 = 0;
    g_vehicle_profile_speed_mm_s = 0;
    g_vehicle_acceleration_command_mm_s2 = 0;
    vehicle_previous_speed_mm_s = 0;
    vehicle_acceleration_sample_valid = false;
    g_odometry_distance_um = 0;
    g_odometry_heading_mdeg = 0;
    g_distance_target_mm = APP_ACTIVE_DISTANCE_TARGET_MM;
    g_distance_remaining_um = 0;
    g_distance_finished = 0U;
    distance_finished_ms = 0U;
    g_angle_target_mdeg = APP_ANGLE_TEST_TARGET_MDEG;
    g_angle_remaining_mdeg = 0;
    g_angle_finished = 0U;
    g_cyz_zero_test_status = 0U;
    g_cyz_chassis_angle_mdeg = 0;
    g_fused_heading_mdeg = 0;
    g_heading_hold_target_mdeg = 0;
    g_heading_hold_error_mdeg = 0;
    g_heading_hold_correction_mdeg_s = 0;
    g_arc_radius_mm = APP_ARC_TEST_RADIUS_MM;
    g_arc_remaining_mdeg = APP_ARC_TEST_TARGET_MDEG;
    g_arc_expected_length_um = 0;
    g_arc_actual_length_um = 0;
    g_arc_finished = 0U;
    g_line_follow_yaw_mdeg_s = 0;
    g_line_follow_finished = 0U;
    g_lap_stop_line_armed = 0U;
    g_lap_stop_line_detected = 0U;
    g_lap_line_active_count_max = 0U;
    g_lap_stop_candidate_max_frames = 0U;
    g_lap_elapsed_ms = 0U;
    g_lap_distance_um = 0;
    g_lap_heading_progress_mdeg = 0;
    g_lap_encoder_heading_progress_mdeg = 0;
    g_lap_gyro_heading_signed_mdeg = 0;
    g_lap_gyro_valid = 0U;
    g_lap_stop_line_distance_um = 0;
    g_line_motion_phase = APP_LINE_MOTION_WAITING;
    g_motion_log_count = 0U;
    g_motion_log_write_index = 0U;
    g_motion_log_overflow_count = 0U;
    g_cruise_straight_sample_count = 0U;
    g_cruise_straight_speed_min_mm_s = 0;
    g_cruise_straight_speed_max_mm_s = 0;
    g_cruise_straight_accel_min_mm_s2 = 0;
    g_cruise_straight_accel_max_mm_s2 = 0;
    g_cruise_turn_sample_count = 0U;
    g_cruise_turn_speed_min_mm_s = 0;
    g_cruise_turn_speed_max_mm_s = 0;
    g_cruise_turn_accel_min_mm_s2 = 0;
    g_cruise_turn_accel_max_mm_s2 = 0;
    lap_line_checked_frame_count = 0U;
    lap_line_clear_stable_frame_count = 0U;
    lap_stop_line_stable_frame_count = 0U;
    lap_stopped_stable_sample_count = 0U;
    lap_line_lost_started_ms = 0U;
    lap_start_distance_um = 0;
    lap_start_heading_mdeg = 0;
    lap_gyro_previous_mdeg = 0;
    lap_gyro_accumulated_mdeg = 0;
    line_yaw_command_mdeg_s = 0;
#if APP_LINE_FOLLOW_TEST_ENABLED
    App_LineProfileReset();
#endif
    motion_log_last_ms = state_started_ms;
    lap_gyro_checked_frame_count = 0U;
    lap_line_lost_active = false;
    lap_stop_requested = false;
#if APP_H_TRACK_TASK_ENABLED
    g_h_task_number = APP_H_ACTIVE_TASK_NUMBER;
#else
    g_h_task_number = 0U;
#endif
    g_h_task_elapsed_ms = 0U;
    g_h_task_finished = 0U;
    g_h_task_start_distance_um = 0;
    g_h_task_final_distance_um = 0;
    g_h_task_final_fused_heading_mdeg = 0;
    g_h_task_final_odometry_heading_mdeg = 0;
    g_h_task_phase = APP_H2_PHASE_IDLE;
    g_h_task_phase_progress_mdeg = 0;
    g_h_task_phase_distance_um = 0;
    g_h_task_bc_exit_fused_heading_mdeg = 0;
    g_h_task_bc_exit_odometry_heading_mdeg = 0;
    g_h_task_bc_exit_progress_mdeg = 0;
    g_h_task_bc_exit_distance_um = 0;
    g_h_task_cross_cb_exit_progress_mdeg = 0;
    g_h_task_cross_cb_exit_distance_um = 0;
    g_h_task_cross_da_exit_progress_mdeg = 0;
    g_h_task_cross_da_exit_distance_um = 0;
    g_h_task_cross_cb_entry_heading_error_mdeg = 0;
    g_h_task_cross_da_entry_heading_error_mdeg = 0;
    g_h_task_cross_cb_entry_line_position_x1000 = 0;
    g_h_task_cross_da_entry_line_position_x1000 = 0;
    g_h_task_line_centered = 0U;
    g_h_task_arc_gyro_takeover = 0U;
    g_h_task_cross_completed_laps = 0U;
    arc_start_distance_um = 0;
    h_task2_phase = APP_H2_PHASE_IDLE;
    h_task2_phase_started_ms = 0U;
    h_task2_line_checked_frame_count = 0U;
    h_task2_line_stable_frame_count = 0U;
    h_task2_line_candidate_started_ms = 0U;
    h_task2_line_lost_started_ms = 0U;
    h_task2_center_checked_frame_count = 0U;
    h_task2_center_stable_frame_count = 0U;
    h_task2_arc_entry_centered = false;
    h_task2_arc_gyro_takeover = false;
    h_task2_initial_heading_mdeg = 0;
    h_task2_arc_start_heading_mdeg = 0;
    h_task2_arc_start_distance_um = 0;
    h_task2_arc_tail_start_distance_um = 0;
    h_task2_arc_tail_target_um = 0;
    h_task2_arc_takeover_progress_mdeg = 0;
    h_task3_phase = APP_H3_PHASE_IDLE;
    h_task3_phase_started_ms = 0U;
    h_task3_line_checked_frame_count = 0U;
    h_task3_line_stable_frame_count = 0U;
    h_task3_line_lost_started_ms = 0U;
    h_task3_center_checked_frame_count = 0U;
    h_task3_center_stable_frame_count = 0U;
    h_task3_completed_laps = 0U;
    h_task3_arc_entry_centered = false;
    h_task3_arc_gyro_takeover = false;
    h_task3_endpoint_line_captured = false;
    h_task3_endpoint_line_distance_um = 0;
    h_task3_endpoint_checked_frame_count = 0U;
    h_task3_endpoint_stable_frame_count = 0U;
    h_task3_initial_heading_mdeg = 0;
    h_task3_arc_start_heading_mdeg = 0;
    h_task3_arc_start_distance_um = 0;
    h_task3_arc_tail_start_distance_um = 0;
    h_task3_arc_tail_target_um = 0;
    h_task3_arc_takeover_progress_mdeg = 0;
    g_h_task_stall_consecutive_samples = 0U;
    g_h_task_arc_entry_centered = 0U;
    g_h_task_pre_entry_active = 0U;
    g_h_task_preview_yaw_mdeg_s = 0;
    g_h_task_arc_takeover_progress_mdeg = 0;
    g_h_task_arc_tail_distance_um = 0;
    g_h_task_arc_tail_target_um = 0;
    g_h_task_arc_exit_reason = 0U;
    g_h_task_diagonal_endpoint_captured = 0U;
    g_h_task_diagonal_compensation_um = 0;
    g_h_task_da_exit_fused_heading_mdeg = 0;
    g_h_task_da_exit_odometry_heading_mdeg = 0;
    g_h_task_da_exit_progress_mdeg = 0;
    g_h_task_da_exit_distance_um = 0;
    g_h_task_arc_tail_expected_progress_mdeg = 0;
    g_h_task_arc_tail_error_mdeg = 0;
    g_h_task_arc_tail_yaw_mdeg_s = 0;
    g_h_task_arc_tail_distance_hold = 0U;
    g_h_task_diagonal_tail_slowdown = 0U;
    angle_finished_ms = 0U;
    pi_left_cps_sum = 0;
    pi_right_cps_sum = 0;
    pi_left_output_sum = 0;
    pi_right_output_sum = 0;
    ChassisKinematics_Init(&chassis_kinematics,
                           CHASSIS_WHEEL_DIAMETER_MM,
                           CHASSIS_TRACK_WIDTH_MM,
                           BSP_ENCODER_COUNTS_PER_WHEEL_REV);
    ChassisOdometry_Reset(&chassis_odometry);
    DistanceProfile_Init(&distance_profile,
                         APP_ACTIVE_DISTANCE_MAX_SPEED_MM_S,
                         APP_ACTIVE_DISTANCE_MIN_SPEED_MM_S,
                         APP_ACTIVE_DISTANCE_SLOWDOWN_MM,
                         APP_ACTIVE_DISTANCE_TOLERANCE_MM);
    AngleProfile_Init(&angle_profile,
                      APP_ANGLE_TEST_MAX_YAW_MDEG_S,
                      APP_ANGLE_TEST_MIN_YAW_MDEG_S,
                      APP_ANGLE_TEST_SLOWDOWN_MDEG,
                      APP_ANGLE_TEST_TOLERANCE_MDEG);
    ArcProfile_Init(&arc_profile,
                    APP_ARC_TEST_RADIUS_MM,
                    APP_ARC_TEST_MAX_SPEED_MM_S,
                    APP_ARC_TEST_MIN_SPEED_MM_S,
                    APP_ARC_TEST_SLOWDOWN_MDEG,
                    APP_ARC_TEST_TOLERANCE_MDEG);
    g_arc_expected_length_um = ArcProfile_GetExpectedLengthUm(
        &arc_profile, APP_ARC_TEST_TARGET_MDEG);
    HeadingEstimator_Init(&heading_estimator,
                          APP_CYZ_YAW_SIGN,
                          APP_HEADING_GYRO_WEIGHT_PERMILLE);
    HeadingController_Init(&heading_controller,
                           APP_HEADING_HOLD_KP_X1000,
                           APP_HEADING_HOLD_MAX_YAW_MDEG_S);
    LineFollower_Init(&line_follower,
                      APP_ACTIVE_LINE_KP,
                      APP_ACTIVE_LINE_MAX_YAW_MDEG_S);
    WheelSpeed_Init(&speed_estimator,
                    APP_SPEED_SAMPLE_PERIOD_MS,
                    BSP_ENCODER_COUNTS_PER_WHEEL_REV,
                    0,
                    0,
                    state_started_ms);
    WheelSpeedPI_Init(&left_speed_controller,
                      APP_SPEED_PI_KP_X1000,
                      APP_SPEED_PI_KI_X1000_PER_SECOND,
                      APP_SPEED_PI_FEEDFORWARD_GAIN_X1000,
                      APP_SPEED_PI_STATIC_FEEDFORWARD_PERMILLE,
                      APP_SPEED_PI_STATIC_FEEDFORWARD_CUTOFF_CPS,
                      APP_SPEED_PI_HIGH_SPEED_FEEDFORWARD_GAIN_X1000,
                      APP_SPEED_PI_HIGH_SPEED_FEEDFORWARD_START_CPS,
                      APP_SPEED_PI_OUTPUT_MIN_PERMILLE,
                      APP_SPEED_PI_OUTPUT_MAX_PERMILLE,
                      APP_SPEED_PI_INTEGRAL_LIMIT);
    WheelSpeedPI_Init(&right_speed_controller,
                      APP_SPEED_PI_KP_X1000,
                      APP_SPEED_PI_KI_X1000_PER_SECOND,
                      APP_SPEED_PI_FEEDFORWARD_GAIN_X1000,
                      APP_SPEED_PI_STATIC_FEEDFORWARD_PERMILLE,
                      APP_SPEED_PI_STATIC_FEEDFORWARD_CUTOFF_CPS,
                      APP_SPEED_PI_HIGH_SPEED_FEEDFORWARD_GAIN_X1000,
                      APP_SPEED_PI_HIGH_SPEED_FEEDFORWARD_START_CPS,
                      APP_SPEED_PI_OUTPUT_MIN_PERMILLE,
                      APP_SPEED_PI_OUTPUT_MAX_PERMILLE,
                      APP_SPEED_PI_INTEGRAL_LIMIT);
    ChassisVelocity_Init(&chassis_velocity,
                         APP_CHASSIS_MAX_WHEEL_CPS,
                         APP_CHASSIS_SLEW_CPS_PER_SECOND);

    BSP_DebugUART_Write("\r\nHtiCar wheel-speed control ready\r\n");
#if APP_LINE_POLARITY_TEST_ENABLED
    BSP_DebugUART_Write("Line polarity test: PC13 displays raw D4\r\n");
#elif APP_H_TASK3_ENABLED
    BSP_DebugUART_Write("H task 3 crossed route starts after delay\r\n");
#elif APP_H_TASK2_ENABLED
    BSP_DebugUART_Write("H task 2 segmented route starts after delay\r\n");
#elif APP_H_TASK1_ENABLED
    BSP_DebugUART_Write("H task 1 A-to-B run starts after delay\r\n");
#elif APP_LINE_FOLLOW_TEST_ENABLED
    BSP_DebugUART_Write(
        "One-lap line follow: leave start marker, stop on next marker\r\n");
#elif APP_ARC_TEST_ENABLED
    BSP_DebugUART_Write("400 mm radius arc test starts after delay\r\n");
#elif APP_HEADING_STRAIGHT_TEST_ENABLED
    BSP_DebugUART_Write("Heading-hold straight test starts after delay\r\n");
#elif APP_ANGLE_TEST_ENABLED
    BSP_DebugUART_Write("Angle test starts after delay\r\n");
#elif APP_DISTANCE_TEST_ENABLED
    BSP_DebugUART_Write("Distance test starts after delay\r\n");
#elif APP_GROUND_STRAIGHT_TEST_ENABLED
    BSP_DebugUART_Write("Ground straight test starts after delay\r\n");
#elif APP_CHASSIS_MIX_TEST_ENABLED
    BSP_DebugUART_Write("Chassis mixer test starts after delay\r\n");
#elif APP_DIRECTION_DIAGNOSTIC_ENABLED
    BSP_DebugUART_Write("Direction diagnostic starts after delay\r\n");
#elif APP_CLOSED_LOOP_TEST_ENABLED
    BSP_DebugUART_Write("Closed-loop speed test starts after delay\r\n");
#elif APP_SPEED_TEST_ENABLED
    BSP_DebugUART_Write("Open-loop speed test starts after delay\r\n");
#else
    BSP_DebugUART_Write("Motors are in safe idle mode\r\n");
#endif
}

void App_Run(void)
{
    int32_t left_count;
    int32_t right_count;
    uint32_t now_ms;
    bool speed_sample_ready;

    BSP_Encoder_Update();
    now_ms = HAL_GetTick();
    CYZ_Sensor_Process(now_ms);
    LineSensor_Process(now_ms);
#if APP_ZDT_STEPPER_ENABLED
    ZDT_Stepper_Process();
#if APP_ZDT_STEPPER_TEST_ENABLED
    App_ZDTStepperTestUpdate(now_ms);
#endif
#endif
    g_cyz_chassis_angle_mdeg = HeadingEstimator_ConvertGyroMdeg(
        &heading_estimator, g_cyz_angle_mdeg);
#if APP_CYZ_ZERO_TEST_ENABLED
    if ((g_cyz_zero_test_status == 0U) &&
        ((uint32_t)(now_ms - state_started_ms) >=
         APP_CYZ_ZERO_TEST_DELAY_MS) &&
        CYZ_Sensor_IsFresh(now_ms, 100U))
    {
        g_cyz_zero_test_status = CYZ_Sensor_ZeroAngle() ? 1U : 2U;
    }
#endif
    left_count = BSP_Encoder_GetLeftCount();
    right_count = BSP_Encoder_GetRightCount();
    speed_sample_ready = App_UpdateWheelSpeed(now_ms, left_count, right_count);
    if (speed_sample_ready)
    {
        HeadingEstimator_Update(
            &heading_estimator,
            chassis_odometry.heading_mdeg,
            g_cyz_angle_mdeg,
            CYZ_Sensor_IsFresh(now_ms, APP_CYZ_MAXIMUM_AGE_MS));
        g_fused_heading_mdeg = HeadingEstimator_GetHeadingMdeg(
            &heading_estimator);
    }

#if APP_LINE_POLARITY_TEST_ENABLED
    (void)speed_sample_ready;
    BSP_Motor_CoastAll();
    if (LineSensor_IsFresh(now_ms, APP_LINE_SENSOR_MAXIMUM_AGE_MS))
    {
        BSP_LED_Set(g_line_d4 != 0U);
    }
    else if ((uint32_t)(now_ms - led_changed_ms) >= 500U)
    {
        BSP_LED_Toggle();
        led_changed_ms = now_ms;
    }
#elif APP_H_TASK3_ENABLED
    if (test_state == APP_TEST_WAITING)
    {
        if ((uint32_t)(now_ms - led_changed_ms) >= 250U)
        {
            BSP_LED_Toggle();
            led_changed_ms = now_ms;
        }

        if (((uint32_t)(now_ms - state_started_ms) >=
             APP_H_TASK3_START_DELAY_MS) &&
            CYZ_Sensor_IsFresh(now_ms, APP_CYZ_MAXIMUM_AGE_MS))
        {
            BSP_LED_Set(true);
            WheelSpeedPI_Reset(&left_speed_controller);
            WheelSpeedPI_Reset(&right_speed_controller);
            ChassisVelocity_Reset(&chassis_velocity);
            h_task3_initial_heading_mdeg = g_fused_heading_mdeg;
            heading_controller.target_heading_mdeg =
                h_task3_initial_heading_mdeg;
            g_heading_hold_target_mdeg =
                heading_controller.target_heading_mdeg;
            g_h_task_start_distance_um = chassis_odometry.distance_um;
            DistanceProfile_Start(&distance_profile,
                                  chassis_odometry.distance_um,
                                  APP_H_TASK3_AC_APPROACH_MM);
            h_task3_phase = APP_H3_PHASE_DIAGONAL_AC;
            h_task3_phase_started_ms = now_ms;
            g_h_task_phase = h_task3_phase;
            test_state = APP_TEST_RUNNING;
            state_started_ms = now_ms;
        }
    }
    else if (test_state == APP_TEST_RUNNING)
    {
        g_h_task_elapsed_ms = now_ms - state_started_ms;

        if (!CYZ_Sensor_IsFresh(now_ms, APP_CYZ_RUNTIME_TIMEOUT_MS))
        {
            App_FinishChassisTest(6U, left_count, right_count);
        }
        else if (g_h_task_elapsed_ms >= APP_H_TASK3_TIMEOUT_MS)
        {
            App_FinishChassisTest(3U, left_count, right_count);
        }
        else if ((h_task3_phase == APP_H3_PHASE_STOPPING) &&
                 ((uint32_t)(now_ms - distance_finished_ms) >=
                  APP_H_TASK3_POST_STOP_MS))
        {
            g_line_follow_finished = 1U;
            App_FinishChassisTest(0U, left_count, right_count);
        }
        else if (speed_sample_ready)
        {
            int32_t linear_mm_s = 0;
            int32_t yaw_mdeg_s = 0;
            int32_t left_command_cps;
            int32_t right_command_cps;
            bool command_valid = true;

            if ((h_task3_phase == APP_H3_PHASE_DIAGONAL_AC) ||
                (h_task3_phase == APP_H3_PHASE_DIAGONAL_BD))
            {
                linear_mm_s = DistanceProfile_Update(
                    &distance_profile, chassis_odometry.distance_um);
                yaw_mdeg_s = HeadingController_Update(
                    &heading_controller, g_fused_heading_mdeg);
                g_distance_remaining_um = DistanceProfile_GetRemainingUm(
                    &distance_profile, chassis_odometry.distance_um);
                g_heading_hold_error_mdeg =
                    HeadingController_GetErrorMdeg(
                        &heading_controller, g_fused_heading_mdeg);
                g_heading_hold_correction_mdeg_s = yaw_mdeg_s;

                if (DistanceProfile_IsFinished(&distance_profile))
                {
                    h_task3_phase =
                        (h_task3_phase == APP_H3_PHASE_DIAGONAL_AC) ?
                        APP_H3_PHASE_ALIGN_C : APP_H3_PHASE_ALIGN_D;
                    h_task3_phase_started_ms = now_ms;
                }
            }
            else if (h_task3_phase == APP_H3_PHASE_ALIGN_C)
            {
                int32_t target_heading = h_task3_initial_heading_mdeg +
                    APP_H_TASK3_CORNER_ANGLE_MDEG;

                if (g_fused_heading_mdeg >=
                    target_heading - APP_H_TASK3_PIVOT_TOLERANCE_MDEG)
                {
                    h_task3_phase = APP_H3_PHASE_ACQUIRE_C;
                    h_task3_phase_started_ms = now_ms;
                    h_task3_line_checked_frame_count =
                        g_line_detection_event_count;
                    h_task3_line_stable_frame_count = 0U;
                    h_task3_arc_start_heading_mdeg =
                        g_fused_heading_mdeg;
                    h_task3_arc_start_distance_um =
                        chassis_odometry.distance_um;
                    linear_mm_s = APP_H_TASK3_ACQUIRE_SPEED_MM_S;
                    yaw_mdeg_s = APP_H_TASK3_ACQUIRE_YAW_MDEG_S;
                }
                else
                {
                    linear_mm_s = APP_H_TASK3_PIVOT_LINEAR_MM_S;
                    yaw_mdeg_s = APP_H_TASK3_PIVOT_YAW_MDEG_S;
                }
            }
            else if (h_task3_phase == APP_H3_PHASE_ALIGN_B)
            {
                int32_t target_heading = h_task3_initial_heading_mdeg +
                    180000 + (2 * APP_H_TASK3_CORNER_ANGLE_MDEG);

                if (g_fused_heading_mdeg >=
                    target_heading - APP_H_TASK3_PIVOT_TOLERANCE_MDEG)
                {
                    h_task3_phase = APP_H3_PHASE_DIAGONAL_BD;
                    h_task3_phase_started_ms = now_ms;
                    heading_controller.target_heading_mdeg = target_heading;
                    g_heading_hold_target_mdeg = target_heading;
                }
                else
                {
                    linear_mm_s = APP_H_TASK3_PIVOT_LINEAR_MM_S;
                    yaw_mdeg_s = APP_H_TASK3_PIVOT_YAW_MDEG_S;
                }
            }
            else if (h_task3_phase == APP_H3_PHASE_ALIGN_D)
            {
                int32_t target_heading = h_task3_initial_heading_mdeg +
                    180000 + APP_H_TASK3_CORNER_ANGLE_MDEG;

                if (g_fused_heading_mdeg <=
                    target_heading + APP_H_TASK3_PIVOT_TOLERANCE_MDEG)
                {
                    h_task3_phase = APP_H3_PHASE_ACQUIRE_D;
                    h_task3_phase_started_ms = now_ms;
                    h_task3_line_checked_frame_count =
                        g_line_detection_event_count;
                    h_task3_line_stable_frame_count = 0U;
                    h_task3_arc_start_heading_mdeg =
                        g_fused_heading_mdeg;
                    h_task3_arc_start_distance_um =
                        chassis_odometry.distance_um;
                    linear_mm_s = APP_H_TASK3_ACQUIRE_SPEED_MM_S;
                    yaw_mdeg_s = -APP_H_TASK3_ACQUIRE_YAW_MDEG_S;
                }
                else
                {
                    linear_mm_s = APP_H_TASK3_PIVOT_LINEAR_MM_S;
                    yaw_mdeg_s = -APP_H_TASK3_PIVOT_YAW_MDEG_S;
                }
            }
            else if ((h_task3_phase == APP_H3_PHASE_ACQUIRE_C) ||
                     (h_task3_phase == APP_H3_PHASE_ACQUIRE_D))
            {
                int32_t direction =
                    (h_task3_phase == APP_H3_PHASE_ACQUIRE_C) ? 1 : -1;

                linear_mm_s = APP_H_TASK3_ACQUIRE_SPEED_MM_S;
                yaw_mdeg_s = App_GetAcquireSweepYawMdegS(
                    (uint32_t)(now_ms - h_task3_phase_started_ms),
                    direction,
                    APP_H_TASK3_ACQUIRE_YAW_MDEG_S);
                g_h_task_phase_progress_mdeg = direction *
                    (g_fused_heading_mdeg -
                     h_task3_arc_start_heading_mdeg);
                g_h_task_phase_distance_um =
                    chassis_odometry.distance_um -
                    h_task3_arc_start_distance_um;

                if (LineSensor_IsFresh(
                        now_ms, APP_LINE_SENSOR_MAXIMUM_AGE_MS))
                {
                    if (g_line_detection_event_count !=
                        h_task3_line_checked_frame_count)
                    {
                        h_task3_line_checked_frame_count =
                            g_line_detection_event_count;
                        h_task3_line_stable_frame_count++;
                    }

                    if (h_task3_line_stable_frame_count >=
                        APP_H_TASK3_LINE_STABLE_FRAMES)
                    {
                        h_task3_phase =
                            (h_task3_phase == APP_H3_PHASE_ACQUIRE_C) ?
                            APP_H3_PHASE_ARC_CB : APP_H3_PHASE_ARC_DA;
                        h_task3_phase_started_ms = now_ms;
                        h_task3_line_stable_frame_count = 0U;
                        h_task3_line_lost_started_ms = 0U;
                        h_task3_arc_gyro_takeover = false;
                        linear_mm_s = APP_H_TASK3_ARC_SPEED_MM_S;
                        yaw_mdeg_s = direction *
                            APP_H_TASK3_ARC_BASE_YAW_MDEG_S +
                            LineFollower_Update(
                                &line_follower, g_line_position_x1000);
                    }
                }
                else
                {
                    h_task3_line_checked_frame_count =
                        g_line_detection_event_count;
                    h_task3_line_stable_frame_count = 0U;
                }

                if ((h_task3_phase == APP_H3_PHASE_ACQUIRE_C ||
                     h_task3_phase == APP_H3_PHASE_ACQUIRE_D) &&
                    ((uint32_t)(now_ms - h_task3_phase_started_ms) >=
                     APP_H_TASK3_ACQUIRE_TIMEOUT_MS))
                {
                    App_FinishChassisTest(5U, left_count, right_count);
                    command_valid = false;
                }
            }
            else if ((h_task3_phase == APP_H3_PHASE_ARC_CB) ||
                     (h_task3_phase == APP_H3_PHASE_ARC_DA))
            {
                int32_t direction =
                    (h_task3_phase == APP_H3_PHASE_ARC_CB) ? 1 : -1;
                bool arc_complete = false;
                bool line_available = true;

                g_h_task_phase_progress_mdeg = direction *
                    (g_fused_heading_mdeg -
                     h_task3_arc_start_heading_mdeg);
                g_h_task_phase_distance_um =
                    chassis_odometry.distance_um -
                    h_task3_arc_start_distance_um;

                if (g_h_task_phase_progress_mdeg >=
                    APP_H_TASK3_ARC_GYRO_TAKEOVER_MDEG)
                {
                    h_task3_arc_gyro_takeover = true;
                }

                if (g_h_task_phase_progress_mdeg >=
                    APP_H_TASK3_ARC_END_PROGRESS_MDEG)
                {
                    arc_complete = true;
                }
                else if (h_task3_arc_gyro_takeover)
                {
                    line_available = false;
                    h_task3_line_lost_started_ms = 0U;
                }
                else if (!LineSensor_IsFresh(
                             now_ms, APP_LINE_SENSOR_MAXIMUM_AGE_MS))
                {
                    App_FinishChassisTest(4U, left_count, right_count);
                    command_valid = false;
                }
                else if (g_line_lost != 0U)
                {
                    if (h_task3_line_lost_started_ms == 0U)
                    {
                        /* Keep the last measured line correction briefly.
                           This bridges sensor gaps and gives the arc controller
                           time to bring an edge detection back to the centre. */
                        h_task3_line_lost_started_ms = now_ms;
                    }
                    else if ((uint32_t)(now_ms -
                              h_task3_line_lost_started_ms) >=
                             APP_H_TASK3_LINE_LOST_GRACE_MS)
                    {
                        App_FinishChassisTest(5U,
                                              left_count,
                                              right_count);
                        command_valid = false;
                    }
                    else
                    {
                        /* Continue below using the retained line position. */
                    }
                }
                else
                {
                    h_task3_line_lost_started_ms = 0U;
                }

                if (command_valid && arc_complete)
                {
                    if (h_task3_phase == APP_H3_PHASE_ARC_CB)
                    {
                        h_task3_phase = APP_H3_PHASE_ALIGN_B;
                        h_task3_phase_started_ms = now_ms;
                        DistanceProfile_Start(
                            &distance_profile,
                            chassis_odometry.distance_um,
                            APP_H_TASK3_BD_APPROACH_MM);
                    }
                    else
                    {
                        h_task3_phase = APP_H3_PHASE_STOPPING;
                        h_task3_phase_started_ms = now_ms;
                        g_distance_finished = 1U;
                        distance_finished_ms = now_ms;
                    }
                }
                else if (command_valid)
                {
                    linear_mm_s = line_available ?
                        APP_H_TASK3_ARC_SPEED_MM_S :
                        APP_H_TASK3_ACQUIRE_SPEED_MM_S;
                    yaw_mdeg_s = direction *
                        (line_available ?
                         APP_H_TASK3_ARC_BASE_YAW_MDEG_S :
                         APP_H_TASK3_ACQUIRE_YAW_MDEG_S);
                    if (line_available)
                    {
                        int32_t line_distance =
                            App_AbsInt32(g_line_position_x1000);

                        if (line_distance >=
                            APP_H_TASK3_LINE_EDGE_POSITION_X1000)
                        {
                            linear_mm_s =
                                APP_H_TASK3_LINE_EDGE_SPEED_MM_S;
                        }
                        else if (line_distance >=
                                 APP_H_TASK3_LINE_MID_POSITION_X1000)
                        {
                            linear_mm_s =
                                APP_H_TASK3_LINE_MID_SPEED_MM_S;
                        }
                        yaw_mdeg_s += LineFollower_Update(
                            &line_follower, g_line_position_x1000);
                    }
                }
            }

            if (command_valid)
            {
                bool left_stalled;
                bool right_stalled;

                yaw_mdeg_s = App_ClampInt32(
                    yaw_mdeg_s,
                    -APP_H_TASK3_ARC_MAX_YAW_MDEG_S,
                    APP_H_TASK3_ARC_MAX_YAW_MDEG_S);
                g_h_task_phase = h_task3_phase;
                g_line_follow_yaw_mdeg_s = yaw_mdeg_s;
                g_chassis_linear_command_mm_s = linear_mm_s;
                g_chassis_yaw_command_mdeg_s = yaw_mdeg_s;
                ChassisKinematics_VelocityToWheelCps(
                    &chassis_kinematics,
                    linear_mm_s,
                    yaw_mdeg_s,
                    &left_command_cps,
                    &right_command_cps);
                g_chassis_translation_command_cps =
                    (left_command_cps + right_command_cps) / 2;
                g_chassis_turn_command_cps =
                    (right_command_cps - left_command_cps) / 2;
                ChassisVelocity_SetCommand(
                    &chassis_velocity,
                    g_chassis_translation_command_cps,
                    g_chassis_turn_command_cps);
                ChassisVelocity_SetSlewRate(
                    &chassis_velocity,
                    (((((h_task2_phase == APP_H2_PHASE_ARC_BC) ||
                        (h_task2_phase == APP_H2_PHASE_ARC_DA) ||
                        (h_task3_phase == APP_H3_PHASE_ARC_CB) ||
                        (h_task3_phase == APP_H3_PHASE_ARC_DA)) &&
                       (g_h_task_arc_gyro_takeover == 0U) &&
                       (g_line_lost == 0U) &&
                       (App_AbsInt32(g_line_position_x1000) >=
                        APP_H_LINE_FAST_ERROR_X1000)) ?
                     APP_H_LINE_FAST_SLEW_CPS_PER_SECOND :
                    (((h_task2_phase == APP_H2_PHASE_ARC_BC) ||
                      (h_task2_phase == APP_H2_PHASE_ARC_DA) ||
                      (h_task3_phase == APP_H3_PHASE_ARC_CB) ||
                      (h_task3_phase == APP_H3_PHASE_ARC_DA) ||
                      (App_AbsInt32(yaw_mdeg_s) >= 5000) ||
                      (((h_task3_phase == APP_H3_PHASE_DIAGONAL_AC) ||
                        (h_task3_phase == APP_H3_PHASE_DIAGONAL_BD)) &&
                       ((uint32_t)(now_ms - h_task3_phase_started_ms) <
                        APP_H_TURN_EXIT_FAST_RESPONSE_MS))) ?
                     APP_H_LINE_SLEW_CPS_PER_SECOND :
                     APP_CHASSIS_SLEW_CPS_PER_SECOND));
                ChassisVelocity_Update(&chassis_velocity,
                                       g_wheel_speed_elapsed_ms);
                g_chassis_left_target_cps =
                    ChassisVelocity_GetLeftTarget(&chassis_velocity);
                g_chassis_right_target_cps =
                    ChassisVelocity_GetRightTarget(&chassis_velocity);
                left_stalled =
                    (App_AbsInt32(g_chassis_left_target_cps) >=
                     APP_CLOSED_LOOP_MIN_VALID_CPS) &&
                    (App_SpeedAlongWheelTarget(
                         g_wheel_speed_left_cps,
                         g_chassis_left_target_cps) <
                     APP_STRAIGHT_STALL_MIN_MEASURED_CPS);
                right_stalled =
                    (App_AbsInt32(g_chassis_right_target_cps) >=
                     APP_CLOSED_LOOP_MIN_VALID_CPS) &&
                    (App_SpeedAlongWheelTarget(
                         g_wheel_speed_right_cps,
                         g_chassis_right_target_cps) <
                     APP_STRAIGHT_STALL_MIN_MEASURED_CPS);

                if ((h_task3_phase != APP_H3_PHASE_STOPPING) &&
                    (g_h_task_elapsed_ms >=
                     APP_CLOSED_LOOP_STALL_CHECK_MS) &&
                    (left_stalled || right_stalled))
                {
                    App_FinishChassisTest(1U,
                                          left_count,
                                          right_count);
                }
                else if ((App_AbsInt32(g_wheel_speed_left_cps) >
                          APP_CLOSED_LOOP_MAX_VALID_CPS) ||
                         (App_AbsInt32(g_wheel_speed_right_cps) >
                          APP_CLOSED_LOOP_MAX_VALID_CPS))
                {
                    App_FinishChassisTest(2U,
                                          left_count,
                                          right_count);
                }
                else
                {
                    g_pi_left_output_permille = WheelSpeedPI_Update(
                        &left_speed_controller,
                        g_chassis_left_target_cps,
                        g_wheel_speed_left_cps,
                        g_wheel_speed_elapsed_ms);
                    g_pi_right_output_permille = WheelSpeedPI_Update(
                        &right_speed_controller,
                        g_chassis_right_target_cps,
                        g_wheel_speed_right_cps,
                        g_wheel_speed_elapsed_ms);
                    BSP_Motor_SetSpeed(
                        BSP_MOTOR_LEFT,
                        (int16_t)g_pi_left_output_permille);
                    BSP_Motor_SetSpeed(
                        BSP_MOTOR_RIGHT,
                        (int16_t)g_pi_right_output_permille);
                }
            }
        }
    }
    else
    {
        BSP_Motor_CoastAll();
    }
#elif APP_H_TASK2_ENABLED
    if (test_state == APP_TEST_WAITING)
    {
        if ((uint32_t)(now_ms - led_changed_ms) >= 250U)
        {
            BSP_LED_Toggle();
            led_changed_ms = now_ms;
        }

        if (((uint32_t)(now_ms - state_started_ms) >=
             APP_H_TASK2_START_DELAY_MS) &&
            CYZ_Sensor_IsFresh(now_ms, APP_CYZ_MAXIMUM_AGE_MS))
        {
            BSP_LED_Set(true);
            WheelSpeedPI_Reset(&left_speed_controller);
            WheelSpeedPI_Reset(&right_speed_controller);
            ChassisVelocity_Reset(&chassis_velocity);
            h_task2_initial_heading_mdeg = g_fused_heading_mdeg;
            heading_controller.target_heading_mdeg =
                h_task2_initial_heading_mdeg;
            g_heading_hold_target_mdeg =
                heading_controller.target_heading_mdeg;
            g_h_task_start_distance_um = chassis_odometry.distance_um;
            DistanceProfile_Start(&distance_profile,
                                  chassis_odometry.distance_um,
                                  APP_H_TASK2_TARGET_MM);
            h_task2_phase = APP_H2_PHASE_STRAIGHT_AB;
            h_task2_phase_started_ms = now_ms;
            g_h_task_phase = h_task2_phase;
            test_state = APP_TEST_RUNNING;
            state_started_ms = now_ms;
        }
    }
    else if (test_state == APP_TEST_RUNNING)
    {
        g_h_task_elapsed_ms = now_ms - state_started_ms;

        if (!CYZ_Sensor_IsFresh(now_ms, APP_CYZ_RUNTIME_TIMEOUT_MS))
        {
            App_FinishChassisTest(6U, left_count, right_count);
        }
        else if (g_h_task_elapsed_ms >= APP_H_COMBINED_TIMEOUT_MS)
        {
            App_FinishChassisTest(3U, left_count, right_count);
        }
        else if ((h_task3_phase == APP_H3_PHASE_STOPPING) &&
                 ((uint32_t)(now_ms - distance_finished_ms) >=
                  APP_H_TASK3_POST_STOP_MS))
        {
            g_line_follow_finished = 1U;
            App_FinishChassisTest(0U, left_count, right_count);
        }
        else if (speed_sample_ready)
        {
            int32_t linear_mm_s = 0;
            int32_t yaw_mdeg_s = 0;
            int32_t left_command_cps;
            int32_t right_command_cps;
            bool command_valid = true;

            g_h_task_pre_entry_active = 0U;
            g_h_task_preview_yaw_mdeg_s = 0;
            g_h_task_diagonal_tail_slowdown = 0U;

            if (h_task3_phase != APP_H3_PHASE_IDLE)
            {
                command_valid = App_UpdateChainedCrossRoute(
                    now_ms, left_count, right_count,
                    &linear_mm_s, &yaw_mdeg_s);
            }
            else if ((h_task2_phase == APP_H2_PHASE_STRAIGHT_AB) ||
                (h_task2_phase == APP_H2_PHASE_STRAIGHT_CD))
            {
                linear_mm_s = DistanceProfile_Update(
                    &distance_profile, chassis_odometry.distance_um);
                yaw_mdeg_s = HeadingController_Update(
                    &heading_controller, g_fused_heading_mdeg);
                g_distance_remaining_um = DistanceProfile_GetRemainingUm(
                    &distance_profile, chassis_odometry.distance_um);
                g_heading_hold_error_mdeg =
                    HeadingController_GetErrorMdeg(
                        &heading_controller, g_fused_heading_mdeg);
                g_heading_hold_correction_mdeg_s = yaw_mdeg_s;

                if ((g_distance_remaining_um <=
                     APP_H_TASK2_PRE_ENTRY_DISTANCE_MM * 1000) &&
                    (linear_mm_s > APP_H_PRE_ENTRY_SPEED_MM_S))
                {
                    linear_mm_s = APP_H_PRE_ENTRY_SPEED_MM_S;
                    g_h_task_pre_entry_active = 1U;
                }

                /*
                 * 高速入弯预瞄：编码器接近预计弯口时逐步加入右转前馈，
                 * 先建立曲率，再由循迹首帧修正位置误差，避免高速直冲。
                 */
                if ((g_distance_remaining_um > 0) &&
                    (g_distance_remaining_um <=
                     APP_H_TASK2_PREVIEW_START_REMAINING_MM * 1000))
                {
                    int32_t preview_range_mm =
                        APP_H_TASK2_PREVIEW_START_REMAINING_MM -
                        APP_H_TASK2_PREVIEW_FULL_REMAINING_MM;
                    int32_t preview_travel_mm =
                        APP_H_TASK2_PREVIEW_START_REMAINING_MM -
                        (g_distance_remaining_um / 1000);

                    if (preview_travel_mm > preview_range_mm)
                    {
                        preview_travel_mm = preview_range_mm;
                    }
                    if ((preview_range_mm > 0) &&
                        (preview_travel_mm > 0))
                    {
                        g_h_task_preview_yaw_mdeg_s =
                            -(int32_t)(((int64_t)
                                APP_H_PREVIEW_ARC_YAW_MDEG_S *
                                preview_travel_mm) /
                                preview_range_mm);
                        yaw_mdeg_s += g_h_task_preview_yaw_mdeg_s;
                    }
                }

                if (DistanceProfile_IsFinished(&distance_profile) ||
                    ((g_distance_remaining_um <=
                      APP_H_ENDPOINT_LINE_ARM_REMAINING_MM * 1000) &&
                     App_HasRecentLineDetection(now_ms)))
                {
                    App_H2Phase acquire_phase =
                        (h_task2_phase == APP_H2_PHASE_STRAIGHT_AB) ?
                        APP_H2_PHASE_ACQUIRE_B : APP_H2_PHASE_ACQUIRE_D;
                    App_H2Phase arc_phase =
                        (h_task2_phase == APP_H2_PHASE_STRAIGHT_AB) ?
                        APP_H2_PHASE_ARC_BC : APP_H2_PHASE_ARC_DA;

                    h_task2_phase_started_ms = now_ms;
                    /* 预瞄可能已开始转向，圆弧零点必须使用直线理论航向。 */
                    h_task2_arc_start_heading_mdeg =
                        (h_task2_phase == APP_H2_PHASE_STRAIGHT_AB) ?
                        h_task2_initial_heading_mdeg :
                        h_task2_initial_heading_mdeg - 180000;
                    h_task2_arc_start_distance_um =
                        chassis_odometry.distance_um;
                    h_task2_arc_tail_start_distance_um = 0;
                    h_task2_arc_tail_target_um = 0;
                    h_task2_arc_takeover_progress_mdeg = 0;
                    g_h_task_arc_takeover_progress_mdeg = 0;
                    g_h_task_arc_tail_distance_um = 0;
                    g_h_task_arc_tail_target_um = 0;
                    g_h_task_arc_exit_reason = 0U;
                    g_h_task_arc_tail_expected_progress_mdeg = 0;
                    g_h_task_arc_tail_error_mdeg = 0;
                    g_h_task_arc_tail_yaw_mdeg_s = 0;
                    g_h_task_arc_tail_distance_hold = 0U;
                    h_task2_line_candidate_started_ms = 0U;
                    h_task2_line_lost_started_ms = 0U;

                    if (App_HasRecentLineDetection(now_ms))
                    {
                        h_task2_phase = arc_phase;
                        h_task2_center_checked_frame_count =
                            g_line_valid_frame_count;
                        h_task2_center_stable_frame_count = 0U;
                        h_task2_arc_entry_centered = false;
                        h_task2_arc_gyro_takeover = false;
                        g_h_task_line_centered = 0U;
                        g_h_task_arc_gyro_takeover = 0U;
                        g_h_task_arc_entry_centered = 0U;
                        linear_mm_s = APP_H_ARC_ENTRY_SPEED_MM_S;
                        yaw_mdeg_s = APP_H_TASK2_ARC_BASE_YAW_MDEG_S +
                            LineFollower_Update(
                                &line_follower,
                                g_line_last_detected_position_x1000);
                    }
                    else
                    {
                        h_task2_phase = acquire_phase;
                        h_task2_line_checked_frame_count =
                            g_line_detection_event_count;
                        h_task2_line_stable_frame_count = 0U;
                        h_task2_line_candidate_started_ms = 0U;
                        linear_mm_s = APP_H_TASK2_ACQUIRE_SPEED_MM_S;
                        yaw_mdeg_s = APP_H_TASK2_ACQUIRE_YAW_MDEG_S;
                    }
                }
            }
            else if ((h_task2_phase == APP_H2_PHASE_ACQUIRE_B) ||
                     (h_task2_phase == APP_H2_PHASE_ACQUIRE_D))
            {
                linear_mm_s = APP_H_TASK2_ACQUIRE_SPEED_MM_S;
                yaw_mdeg_s = App_GetAcquireSweepYawMdegS(
                    (uint32_t)(now_ms - h_task2_phase_started_ms),
                    -1,
                    APP_H_TASK2_ACQUIRE_YAW_MDEG_S);
                g_h_task_phase_progress_mdeg =
                    h_task2_arc_start_heading_mdeg -
                    g_fused_heading_mdeg;
                g_h_task_phase_distance_um =
                    chassis_odometry.distance_um -
                    h_task2_arc_start_distance_um;

                if (LineSensor_IsFresh(
                        now_ms, APP_LINE_SENSOR_MAXIMUM_AGE_MS))
                {
                    if (g_line_detection_event_count !=
                        h_task2_line_checked_frame_count)
                    {
                        h_task2_line_checked_frame_count =
                            g_line_detection_event_count;
                        if (h_task2_line_candidate_started_ms == 0U)
                        {
                            h_task2_line_candidate_started_ms = now_ms;
                            h_task2_line_stable_frame_count = 0U;
                        }
                        h_task2_line_stable_frame_count++;
                    }

                    if ((h_task2_line_candidate_started_ms != 0U) &&
                        ((uint32_t)(now_ms -
                          h_task2_line_candidate_started_ms) >
                         APP_H_TASK2_LINE_STABLE_WINDOW_MS))
                    {
                        h_task2_line_candidate_started_ms = 0U;
                        h_task2_line_stable_frame_count = 0U;
                    }

                    /* React to the first credible edge hit while the frame
                       window decides whether the line is persistent. */
                    if (h_task2_line_candidate_started_ms != 0U)
                    {
                        yaw_mdeg_s = APP_H_TASK2_ARC_BASE_YAW_MDEG_S +
                            LineFollower_Update(
                                &line_follower,
                                g_line_last_detected_position_x1000);
                    }

                    if (h_task2_line_stable_frame_count >=
                        APP_H_TASK2_LINE_STABLE_FRAMES)
                    {
                        h_task2_phase =
                            (h_task2_phase == APP_H2_PHASE_ACQUIRE_B) ?
                            APP_H2_PHASE_ARC_BC : APP_H2_PHASE_ARC_DA;
                        h_task2_phase_started_ms = now_ms;
                        h_task2_line_stable_frame_count = 0U;
                        h_task2_line_candidate_started_ms = 0U;
                        h_task2_line_lost_started_ms = 0U;
                        h_task2_center_checked_frame_count =
                            g_line_valid_frame_count;
                        h_task2_center_stable_frame_count = 0U;
                        h_task2_arc_entry_centered = false;
                        h_task2_arc_gyro_takeover = false;
                        g_h_task_line_centered = 0U;
                        g_h_task_arc_gyro_takeover = 0U;
                        g_h_task_arc_entry_centered = 0U;
                        linear_mm_s = APP_H_ARC_ENTRY_SPEED_MM_S;
                        yaw_mdeg_s = APP_H_TASK2_ARC_BASE_YAW_MDEG_S +
                            LineFollower_Update(
                                &line_follower,
                                g_line_last_detected_position_x1000);
                    }
                }
                else
                {
                    h_task2_line_checked_frame_count =
                        g_line_detection_event_count;
                    h_task2_line_stable_frame_count = 0U;
                    h_task2_line_candidate_started_ms = 0U;
                }

                if ((h_task2_phase == APP_H2_PHASE_ACQUIRE_B ||
                     h_task2_phase == APP_H2_PHASE_ACQUIRE_D) &&
                    ((uint32_t)(now_ms - h_task2_phase_started_ms) >=
                     APP_H_TASK2_ACQUIRE_TIMEOUT_MS))
                {
                    App_FinishChassisTest(5U, left_count, right_count);
                    command_valid = false;
                }
            }
            else if ((h_task2_phase == APP_H2_PHASE_ARC_BC) ||
                     (h_task2_phase == APP_H2_PHASE_ARC_DA))
            {
                bool arc_complete = false;
                bool line_available = true;
                bool center_detected_now = false;

                g_h_task_phase_progress_mdeg =
                    h_task2_arc_start_heading_mdeg -
                    g_fused_heading_mdeg;
                g_h_task_phase_distance_um =
                    chassis_odometry.distance_um -
                    h_task2_arc_start_distance_um;

                if (!h_task2_arc_entry_centered &&
                    LineSensor_IsFresh(
                        now_ms, APP_LINE_SENSOR_MAXIMUM_AGE_MS) &&
                    (g_line_lost == 0U) &&
                    (g_line_detected_mask != 0U) &&
                    ((g_line_detected_mask &
                      ~APP_H_TASK2_CENTER_DETECTED_MASK) == 0U))
                {
                    h_task2_arc_entry_centered = true;
                }
                g_h_task_arc_entry_centered =
                    h_task2_arc_entry_centered ? 1U : 0U;

                if ((g_h_task_phase_progress_mdeg >=
                     APP_H_TASK2_CENTERING_START_MDEG) &&
                    LineSensor_IsFresh(
                        now_ms, APP_LINE_SENSOR_MAXIMUM_AGE_MS) &&
                    (g_line_lost == 0U))
                {
                    center_detected_now =
                        (g_line_detected_mask != 0U) &&
                        ((g_line_detected_mask &
                          ~APP_H_TASK2_CENTER_DETECTED_MASK) == 0U);

                    if (g_line_valid_frame_count !=
                        h_task2_center_checked_frame_count)
                    {
                        h_task2_center_checked_frame_count =
                            g_line_valid_frame_count;
                        if (center_detected_now)
                        {
                            h_task2_center_stable_frame_count++;
                        }
                        else
                        {
                            h_task2_center_stable_frame_count = 0U;
                            if (!h_task2_arc_gyro_takeover)
                            {
                                g_h_task_line_centered = 0U;
                            }
                        }
                    }
                }
                else if (g_h_task_phase_progress_mdeg >=
                         APP_H_TASK2_CENTERING_START_MDEG)
                {
                    h_task2_center_stable_frame_count = 0U;
                }

                if (h_task2_center_stable_frame_count >=
                    APP_H_TASK2_CENTERING_STABLE_FRAMES)
                {
                    /* Latch qualification for the remainder of this arc.
                       The front sensor may leave the finite black endpoint
                       before the gyro takeover angle is reached. */
                    g_h_task_line_centered = 1U;
                }

                if (!h_task2_arc_gyro_takeover &&
                    (g_h_task_phase_progress_mdeg >=
                     APP_H_TASK2_ARC_GYRO_TAKEOVER_MDEG) &&
                    (g_h_task_line_centered != 0U))
                {
                    h_task2_arc_gyro_takeover = true;
                    h_task2_arc_tail_start_distance_um =
                        chassis_odometry.distance_um;
                    h_task2_arc_takeover_progress_mdeg =
                        g_h_task_phase_progress_mdeg;
                    h_task2_arc_tail_target_um = App_GetArcTailTargetUm(
                        h_task2_arc_takeover_progress_mdeg,
                        APP_H_TASK2_ARC_END_PROGRESS_MDEG);
                    g_h_task_arc_takeover_progress_mdeg =
                        h_task2_arc_takeover_progress_mdeg;
                    g_h_task_arc_tail_target_um =
                        h_task2_arc_tail_target_um;
                }
                g_h_task_arc_gyro_takeover =
                    h_task2_arc_gyro_takeover ? 1U : 0U;

                if (h_task2_arc_gyro_takeover)
                {
                    g_h_task_arc_tail_distance_um =
                        chassis_odometry.distance_um -
                        h_task2_arc_tail_start_distance_um;
                    g_h_task_arc_exit_reason = App_GetArcExitReason(
                        g_h_task_phase_progress_mdeg,
                        APP_H_TASK2_ARC_END_PROGRESS_MDEG,
                        g_h_task_arc_tail_distance_um,
                        h_task2_arc_tail_target_um);
                    arc_complete = (g_h_task_arc_exit_reason != 0U);
                }
                if (!arc_complete && !h_task2_arc_gyro_takeover &&
                         (g_h_task_phase_progress_mdeg >=
                          APP_H_TASK2_CENTERING_MAX_MDEG))
                {
                    App_FinishChassisTest(5U, left_count, right_count);
                    command_valid = false;
                }
                else if (h_task2_arc_gyro_takeover)
                {
                    line_available = false;
                    h_task2_line_lost_started_ms = 0U;
                }
                else if (!LineSensor_IsFresh(
                             now_ms, APP_LINE_SENSOR_MAXIMUM_AGE_MS))
                {
                    App_FinishChassisTest(4U, left_count, right_count);
                    command_valid = false;
                }
                else if (g_line_lost != 0U)
                {
                    if (h_task2_line_lost_started_ms == 0U)
                    {
                        h_task2_line_lost_started_ms = now_ms;
                    }
                    else if ((uint32_t)(now_ms -
                              h_task2_line_lost_started_ms) >=
                             (((uint32_t)(now_ms -
                                h_task2_phase_started_ms) <
                               APP_H_ENTRY_CAPTURE_PHASE_MS) ?
                              APP_H_ENTRY_CAPTURE_LOST_GRACE_MS :
                              APP_H_TASK2_LINE_LOST_GRACE_MS))
                    {
                        App_FinishChassisTest(5U,
                                              left_count,
                                              right_count);
                        command_valid = false;
                    }
                    else
                    {
                        /* Recover with the last valid line position below. */
                    }
                }
                else
                {
                    h_task2_line_lost_started_ms = 0U;
                }

                if (command_valid && arc_complete)
                {
                    if (h_task2_phase == APP_H2_PHASE_ARC_BC)
                    {
                        g_h_task_bc_exit_fused_heading_mdeg =
                            g_fused_heading_mdeg;
                        g_h_task_bc_exit_odometry_heading_mdeg =
                            chassis_odometry.heading_mdeg;
                        g_h_task_bc_exit_progress_mdeg =
                            g_h_task_phase_progress_mdeg;
                        g_h_task_bc_exit_distance_um =
                            g_h_task_phase_distance_um;
                        h_task2_phase = APP_H2_PHASE_STRAIGHT_CD;
                        h_task2_phase_started_ms = now_ms;
                        heading_controller.target_heading_mdeg =
                            h_task2_initial_heading_mdeg - 180000;
                        g_heading_hold_target_mdeg =
                            heading_controller.target_heading_mdeg;
                        DistanceProfile_Start(
                            &distance_profile,
                            chassis_odometry.distance_um,
                            APP_H_TASK2_TARGET_MM);
                        linear_mm_s = DistanceProfile_Update(
                            &distance_profile,
                            chassis_odometry.distance_um);
                        yaw_mdeg_s = HeadingController_Update(
                            &heading_controller,
                            g_fused_heading_mdeg);
                    }
                    else
                    {
                        h_task2_phase = APP_H2_PHASE_STOPPING;
                        h_task2_phase_started_ms = now_ms;
                        g_h_task_da_exit_fused_heading_mdeg =
                            g_fused_heading_mdeg;
                        g_h_task_da_exit_odometry_heading_mdeg =
                            chassis_odometry.heading_mdeg;
                        g_h_task_da_exit_progress_mdeg =
                            g_h_task_phase_progress_mdeg;
                        g_h_task_da_exit_distance_um =
                            g_h_task_phase_distance_um;
#if APP_H_DA_EXIT_MEASUREMENT_ENABLED
                        h_task3_phase = APP_H3_PHASE_STOPPING;
                        h_task3_phase_started_ms = now_ms;
                        g_distance_finished = 1U;
                        distance_finished_ms = now_ms;
                        g_h_task_number = 2U;
                        linear_mm_s = 0;
                        yaw_mdeg_s = 0;
#else
                        h_task3_initial_heading_mdeg =
                            g_fused_heading_mdeg -
                            APP_H_TASK3_CORNER_ANGLE_MDEG;
                        h_task3_phase = APP_H3_PHASE_ALIGN_A;
                        h_task3_phase_started_ms = now_ms;
                        h_task3_completed_laps = 0U;
                        g_h_task_cross_completed_laps = 0U;
                        g_h_task_number =
                            (APP_H_CHAINED_CROSS_LOOPS >= 4U) ? 4U : 3U;
                        DistanceProfile_Init(
                            &distance_profile,
                            APP_H_TASK3_MAX_SPEED_MM_S,
                            APP_H_TASK3_MIN_SPEED_MM_S,
                            APP_H_TASK3_SLOWDOWN_MM,
                            APP_H_TASK3_TOLERANCE_MM);
                        linear_mm_s = APP_H_TASK3_PIVOT_LINEAR_MM_S;
                        yaw_mdeg_s = -APP_H_TASK3_PIVOT_YAW_MDEG_S;
#endif
                    }
                }
                else if (command_valid)
                {
                    linear_mm_s = line_available ?
                        APP_H_TASK2_ARC_SPEED_MM_S :
                        APP_H_TASK2_ACQUIRE_SPEED_MM_S;
                    yaw_mdeg_s = line_available ?
                        APP_H_TASK2_ARC_BASE_YAW_MDEG_S :
                        APP_H_TASK2_ACQUIRE_YAW_MDEG_S;
                    if (line_available)
                    {
                        int32_t line_distance =
                            App_AbsInt32(g_line_position_x1000);
                        int32_t line_correction_mdeg_s;

                        if (line_distance >=
                            APP_H_TASK2_LINE_EDGE_POSITION_X1000)
                        {
                            linear_mm_s =
                                APP_H_TASK2_LINE_EDGE_SPEED_MM_S;
                        }
                        else if (line_distance >=
                                 APP_H_TASK2_LINE_MID_POSITION_X1000)
                        {
                            linear_mm_s =
                                APP_H_TASK2_LINE_MID_SPEED_MM_S;
                        }
                        line_correction_mdeg_s = LineFollower_Update(
                            &line_follower, g_line_position_x1000);
                        if ((g_h_task_phase_progress_mdeg >=
                             APP_H_TASK2_CENTERING_START_MDEG) &&
                            (g_h_task_line_centered == 0U))
                        {
                            linear_mm_s =
                                APP_H_TASK2_CENTERING_SPEED_MM_S;
                            line_correction_mdeg_s =
                                (line_correction_mdeg_s *
                                 APP_H_TASK2_CENTERING_GAIN_NUMERATOR) /
                                APP_H_TASK2_CENTERING_GAIN_DENOMINATOR;
                        }
                        yaw_mdeg_s += line_correction_mdeg_s;
                        if (g_h_task_phase_progress_mdeg >=
                            APP_H_TASK2_CENTERING_START_MDEG)
                        {
                            linear_mm_s = APP_H_ARC_EXIT_SPEED_MM_S;
                        }
                        else if (h_task2_arc_entry_centered)
                        {
                            linear_mm_s = APP_H_TASK2_ARC_SPEED_MM_S;
                        }
                        else
                        {
                            linear_mm_s = APP_H_ARC_ENTRY_SPEED_MM_S;
                        }
                    }
                    else
                    {
                        int32_t tail_yaw_magnitude =
                            App_GetArcTailYawMagnitude(
                                h_task2_arc_takeover_progress_mdeg,
                                APP_H_TASK2_ARC_END_PROGRESS_MDEG,
                                g_h_task_arc_tail_distance_um,
                                h_task2_arc_tail_target_um,
                                APP_H_TASK2_ACQUIRE_YAW_MDEG_S);

                        if (g_h_task_arc_tail_distance_um >=
                            h_task2_arc_tail_target_um)
                        {
                            linear_mm_s = 0;
                            g_h_task_arc_tail_distance_hold = 1U;
                        }
                        else
                        {
                            linear_mm_s =
                                APP_H_TASK2_ACQUIRE_SPEED_MM_S;
                        }

                        yaw_mdeg_s =
                            (g_h_task_phase_progress_mdeg >=
                             APP_H_TASK2_ARC_END_PROGRESS_MDEG) ?
                            0 : -tail_yaw_magnitude;
                    }
                }
            }

            if (command_valid)
            {
                yaw_mdeg_s = App_ClampInt32(
                    yaw_mdeg_s,
                    -APP_H_TASK2_ARC_MAX_YAW_MDEG_S,
                    APP_H_TASK2_ARC_MAX_YAW_MDEG_S);
                g_h_task_phase =
                    (h_task3_phase != APP_H3_PHASE_IDLE) ?
                    (100U + (uint32_t)h_task3_phase) :
                    (uint32_t)h_task2_phase;
                g_line_follow_yaw_mdeg_s = yaw_mdeg_s;
                g_chassis_linear_command_mm_s = linear_mm_s;
                g_chassis_yaw_command_mdeg_s = yaw_mdeg_s;
                ChassisKinematics_VelocityToWheelCps(
                    &chassis_kinematics,
                    linear_mm_s,
                    yaw_mdeg_s,
                    &left_command_cps,
                    &right_command_cps);
                g_chassis_translation_command_cps =
                    (left_command_cps + right_command_cps) / 2;
                g_chassis_turn_command_cps =
                    (right_command_cps - left_command_cps) / 2;
                ChassisVelocity_SetCommand(
                    &chassis_velocity,
                    g_chassis_translation_command_cps,
                    g_chassis_turn_command_cps);
                ChassisVelocity_SetSlewRate(
                    &chassis_velocity,
                    (((((h_task2_phase == APP_H2_PHASE_ARC_BC) ||
                        (h_task2_phase == APP_H2_PHASE_ARC_DA) ||
                        (h_task3_phase == APP_H3_PHASE_ARC_CB) ||
                        (h_task3_phase == APP_H3_PHASE_ARC_DA)) &&
                       (g_h_task_arc_gyro_takeover == 0U) &&
                       (g_line_lost == 0U) &&
                       (App_AbsInt32(g_line_position_x1000) >=
                        APP_H_LINE_FAST_ERROR_X1000)) ?
                     APP_H_LINE_FAST_SLEW_CPS_PER_SECOND :
                    (((h_task2_phase == APP_H2_PHASE_ARC_BC) ||
                      (h_task2_phase == APP_H2_PHASE_ARC_DA) ||
                      (h_task3_phase == APP_H3_PHASE_ARC_CB) ||
                      (h_task3_phase == APP_H3_PHASE_ARC_DA) ||
                      (h_task3_phase == APP_H3_PHASE_STOPPING) ||
                      (g_h_task_pre_entry_active != 0U) ||
                      (App_AbsInt32(yaw_mdeg_s) >= 5000) ||
                      ((h_task2_phase == APP_H2_PHASE_STRAIGHT_CD) &&
                       ((uint32_t)(now_ms - h_task2_phase_started_ms) <
                        APP_H_TURN_EXIT_FAST_RESPONSE_MS)) ||
                      (((h_task3_phase == APP_H3_PHASE_DIAGONAL_AC) ||
                        (h_task3_phase == APP_H3_PHASE_DIAGONAL_BD)) &&
                       ((uint32_t)(now_ms - h_task3_phase_started_ms) <
                         APP_H_TURN_EXIT_FAST_RESPONSE_MS)) ||
                      (h_task3_phase == APP_H3_PHASE_ALIGN_A)) ?
                     APP_H_LINE_SLEW_CPS_PER_SECOND :
                     APP_CHASSIS_SLEW_CPS_PER_SECOND))));
                ChassisVelocity_Update(&chassis_velocity,
                                       g_wheel_speed_elapsed_ms);
                g_chassis_left_target_cps =
                    ChassisVelocity_GetLeftTarget(&chassis_velocity);
                g_chassis_right_target_cps =
                    ChassisVelocity_GetRightTarget(&chassis_velocity);

                {
                    bool motion_active =
                        ((h_task3_phase == APP_H3_PHASE_IDLE) &&
                         (h_task2_phase != APP_H2_PHASE_STOPPING)) ||
                        ((h_task3_phase != APP_H3_PHASE_IDLE) &&
                         (h_task3_phase != APP_H3_PHASE_STOPPING));
                    uint32_t phase_elapsed_ms =
                        (h_task3_phase != APP_H3_PHASE_IDLE) ?
                        (uint32_t)(now_ms - h_task3_phase_started_ms) :
                        (uint32_t)(now_ms - h_task2_phase_started_ms);
                    bool stall_sample_detected =
                        ((App_AbsInt32(g_chassis_left_target_cps) >=
                          APP_CLOSED_LOOP_MIN_VALID_CPS) &&
                         (App_SpeedAlongWheelTarget(
                              g_wheel_speed_left_cps,
                              g_chassis_left_target_cps) <
                          APP_STRAIGHT_STALL_MIN_MEASURED_CPS)) ||
                        ((App_AbsInt32(g_chassis_right_target_cps) >=
                          APP_CLOSED_LOOP_MIN_VALID_CPS) &&
                         (App_SpeedAlongWheelTarget(
                              g_wheel_speed_right_cps,
                              g_chassis_right_target_cps) <
                          APP_STRAIGHT_STALL_MIN_MEASURED_CPS));

                    if (motion_active &&
                        (g_h_task_elapsed_ms >=
                         APP_CLOSED_LOOP_STALL_CHECK_MS) &&
                        (phase_elapsed_ms >=
                         APP_H_PHASE_STALL_GRACE_MS) &&
                        stall_sample_detected)
                    {
                        g_h_task_stall_consecutive_samples++;
                    }
                    else
                    {
                        g_h_task_stall_consecutive_samples = 0U;
                    }
                }

                if (g_h_task_stall_consecutive_samples >=
                    APP_H_STALL_CONFIRM_SAMPLES)
                {
                    App_FinishChassisTest(1U,
                                          left_count,
                                          right_count);
                }
                else if ((App_AbsInt32(g_wheel_speed_left_cps) >
                          APP_CLOSED_LOOP_MAX_VALID_CPS) ||
                         (App_AbsInt32(g_wheel_speed_right_cps) >
                          APP_CLOSED_LOOP_MAX_VALID_CPS))
                {
                    App_FinishChassisTest(2U,
                                          left_count,
                                          right_count);
                }
                else
                {
                    g_pi_left_output_permille = WheelSpeedPI_Update(
                        &left_speed_controller,
                        g_chassis_left_target_cps,
                        g_wheel_speed_left_cps,
                        g_wheel_speed_elapsed_ms);
                    g_pi_right_output_permille = WheelSpeedPI_Update(
                        &right_speed_controller,
                        g_chassis_right_target_cps,
                        g_wheel_speed_right_cps,
                        g_wheel_speed_elapsed_ms);
                    BSP_Motor_SetSpeed(
                        BSP_MOTOR_LEFT,
                        (int16_t)g_pi_left_output_permille);
                    BSP_Motor_SetSpeed(
                        BSP_MOTOR_RIGHT,
                        (int16_t)g_pi_right_output_permille);
                }
            }
        }
    }
    else
    {
        BSP_Motor_CoastAll();
    }
#elif APP_H_TRACK_TASK_ENABLED
    if (test_state == APP_TEST_WAITING)
    {
        if ((uint32_t)(now_ms - led_changed_ms) >= 250U)
        {
            BSP_LED_Toggle();
            led_changed_ms = now_ms;
        }

        if (((uint32_t)(now_ms - state_started_ms) >=
             APP_H_ACTIVE_START_DELAY_MS) &&
            LineSensor_IsFresh(now_ms, APP_LINE_SENSOR_MAXIMUM_AGE_MS) &&
            (g_line_lost == 0U))
        {
            BSP_LED_Set(true);
            WheelSpeedPI_Reset(&left_speed_controller);
            WheelSpeedPI_Reset(&right_speed_controller);
            ChassisVelocity_Reset(&chassis_velocity);
            g_h_task_start_distance_um = chassis_odometry.distance_um;
            DistanceProfile_Start(&distance_profile,
                                  chassis_odometry.distance_um,
                                  APP_H_ACTIVE_TARGET_MM);
            test_state = APP_TEST_RUNNING;
            state_started_ms = now_ms;
        }
    }
    else if (test_state == APP_TEST_RUNNING)
    {
        g_h_task_elapsed_ms = now_ms - state_started_ms;

        if (!g_distance_finished &&
            !LineSensor_IsFresh(now_ms, APP_LINE_SENSOR_MAXIMUM_AGE_MS))
        {
            App_FinishChassisTest(4U, left_count, right_count);
        }
        else if (!g_distance_finished && (g_line_lost != 0U))
        {
            App_FinishChassisTest(5U, left_count, right_count);
        }
        else if (!g_distance_finished &&
                 (g_h_task_elapsed_ms >= APP_H_ACTIVE_TIMEOUT_MS))
        {
            App_FinishChassisTest(3U, left_count, right_count);
        }
        else if (g_distance_finished &&
                 ((uint32_t)(now_ms - distance_finished_ms) >=
                  APP_H_ACTIVE_POST_STOP_MS))
        {
            g_line_follow_finished = 1U;
            App_FinishChassisTest(0U, left_count, right_count);
        }
        else if (speed_sample_ready)
        {
            int32_t linear_mm_s;
            int32_t yaw_mdeg_s;
            int32_t left_command_cps;
            int32_t right_command_cps;

            linear_mm_s = DistanceProfile_Update(
                &distance_profile,
                chassis_odometry.distance_um);
            g_distance_remaining_um = DistanceProfile_GetRemainingUm(
                &distance_profile,
                chassis_odometry.distance_um);

            if (DistanceProfile_IsFinished(&distance_profile) &&
                !g_distance_finished)
            {
                g_distance_finished = 1U;
                distance_finished_ms = now_ms;
            }

            if (g_distance_finished)
            {
                yaw_mdeg_s = 0;
            }
            else
            {
                yaw_mdeg_s = LineFollower_Update(
                    &line_follower, g_line_position_x1000);
            }

            g_line_follow_yaw_mdeg_s = yaw_mdeg_s;
            g_chassis_linear_command_mm_s = linear_mm_s;
            g_chassis_yaw_command_mdeg_s = yaw_mdeg_s;
            ChassisKinematics_VelocityToWheelCps(
                &chassis_kinematics,
                linear_mm_s,
                yaw_mdeg_s,
                &left_command_cps,
                &right_command_cps);
            g_chassis_translation_command_cps =
                (left_command_cps + right_command_cps) / 2;
            g_chassis_turn_command_cps =
                (right_command_cps - left_command_cps) / 2;
            ChassisVelocity_SetCommand(
                &chassis_velocity,
                g_chassis_translation_command_cps,
                g_chassis_turn_command_cps);
            ChassisVelocity_Update(&chassis_velocity,
                                   g_wheel_speed_elapsed_ms);
            g_chassis_left_target_cps =
                ChassisVelocity_GetLeftTarget(&chassis_velocity);
            g_chassis_right_target_cps =
                ChassisVelocity_GetRightTarget(&chassis_velocity);

            if (!g_distance_finished &&
                (g_h_task_elapsed_ms >= APP_CLOSED_LOOP_STALL_CHECK_MS) &&
                (App_AbsInt32(g_chassis_left_target_cps) >=
                 APP_CLOSED_LOOP_MIN_VALID_CPS) &&
                ((App_SpeedAlongWheelTarget(g_wheel_speed_left_cps,
                                            g_chassis_left_target_cps) <
                  APP_STRAIGHT_STALL_MIN_MEASURED_CPS) ||
                 (App_SpeedAlongWheelTarget(g_wheel_speed_right_cps,
                                            g_chassis_right_target_cps) <
                  APP_STRAIGHT_STALL_MIN_MEASURED_CPS)))
            {
                App_FinishChassisTest(1U, left_count, right_count);
            }
            else if ((App_AbsInt32(g_wheel_speed_left_cps) >
                      APP_CLOSED_LOOP_MAX_VALID_CPS) ||
                     (App_AbsInt32(g_wheel_speed_right_cps) >
                      APP_CLOSED_LOOP_MAX_VALID_CPS))
            {
                App_FinishChassisTest(2U, left_count, right_count);
            }
            else
            {
                g_pi_left_output_permille = WheelSpeedPI_Update(
                    &left_speed_controller,
                    g_chassis_left_target_cps,
                    g_wheel_speed_left_cps,
                    g_wheel_speed_elapsed_ms);
                g_pi_right_output_permille = WheelSpeedPI_Update(
                    &right_speed_controller,
                    g_chassis_right_target_cps,
                    g_wheel_speed_right_cps,
                    g_wheel_speed_elapsed_ms);
                BSP_Motor_SetSpeed(BSP_MOTOR_LEFT,
                                   (int16_t)g_pi_left_output_permille);
                BSP_Motor_SetSpeed(BSP_MOTOR_RIGHT,
                                   (int16_t)g_pi_right_output_permille);
            }
        }
    }
    else
    {
        BSP_Motor_CoastAll();
    }
#elif APP_LINE_FOLLOW_TEST_ENABLED
    if (test_state == APP_TEST_WAITING)
    {
        if ((uint32_t)(now_ms - led_changed_ms) >= 250U)
        {
            BSP_LED_Toggle();
            led_changed_ms = now_ms;
        }

        if (((uint32_t)(now_ms - state_started_ms) >=
             APP_LINE_FOLLOW_TEST_DELAY_MS) &&
            LineSensor_IsFresh(now_ms, APP_LINE_SENSOR_MAXIMUM_AGE_MS) &&
            CYZ_Sensor_IsFresh(now_ms, APP_CYZ_MAXIMUM_AGE_MS) &&
            (g_line_lost == 0U))
        {
            BSP_LED_Set(true);
            WheelSpeedPI_Reset(&left_speed_controller);
            WheelSpeedPI_Reset(&right_speed_controller);
            ChassisVelocity_Reset(&chassis_velocity);
            ChassisVelocity_SetSlewRate(
                &chassis_velocity,
                APP_LINE_FOLLOW_SLEW_CPS_PER_SECOND);
            lap_line_checked_frame_count = g_line_valid_frame_count;
            lap_line_clear_stable_frame_count = 0U;
            lap_stop_line_stable_frame_count = 0U;
            lap_stopped_stable_sample_count = 0U;
            lap_line_lost_started_ms = 0U;
            lap_start_distance_um = chassis_odometry.distance_um;
            lap_start_heading_mdeg = chassis_odometry.heading_mdeg;
            lap_gyro_previous_mdeg = g_cyz_chassis_angle_mdeg;
            lap_gyro_accumulated_mdeg = 0;
            line_yaw_command_mdeg_s = 0;
            App_LineProfileReset();
            lap_gyro_checked_frame_count = g_cyz_telemetry_frame_count;
            lap_line_lost_active = false;
            lap_stop_requested = false;
            g_lap_stop_line_armed = 0U;
            g_lap_stop_line_detected = 0U;
            g_lap_line_active_count_max = 0U;
            g_lap_stop_candidate_max_frames = 0U;
            g_lap_elapsed_ms = 0U;
            g_lap_distance_um = 0;
            g_lap_heading_progress_mdeg = 0;
            g_lap_encoder_heading_progress_mdeg = 0;
            g_lap_gyro_heading_signed_mdeg = 0;
            g_lap_gyro_valid = 1U;
            g_lap_stop_line_distance_um = 0;
            g_line_motion_phase = APP_LINE_MOTION_ACCELERATING;
            g_motion_log_count = 0U;
            g_motion_log_write_index = 0U;
            g_motion_log_overflow_count = 0U;
            g_cruise_straight_sample_count = 0U;
            g_cruise_turn_sample_count = 0U;
            motion_log_last_ms = now_ms - APP_MOTION_LOG_INTERVAL_MS;
            g_vehicle_profile_speed_mm_s = 0;
            g_vehicle_acceleration_command_mm_s2 = 0;
            g_vehicle_acceleration_raw_mm_s2 = 0;
            g_vehicle_acceleration_mm_s2 = 0;
            g_vehicle_acceleration_max_mm_s2 = 0;
            g_vehicle_acceleration_min_mm_s2 = 0;
            vehicle_previous_speed_mm_s = g_vehicle_linear_speed_mm_s;
            vehicle_acceleration_sample_valid = true;
            test_state = APP_TEST_RUNNING;
            state_started_ms = now_ms;
        }
    }
    else if (test_state == APP_TEST_RUNNING)
    {
        uint32_t run_elapsed_ms = now_ms - state_started_ms;

        g_lap_elapsed_ms = run_elapsed_ms;
        g_lap_distance_um =
            chassis_odometry.distance_um - lap_start_distance_um;
        g_lap_encoder_heading_progress_mdeg = App_AbsInt32(
            chassis_odometry.heading_mdeg - lap_start_heading_mdeg);
        if (lap_gyro_checked_frame_count != g_cyz_telemetry_frame_count)
        {
            lap_gyro_checked_frame_count = g_cyz_telemetry_frame_count;
            lap_gyro_accumulated_mdeg += App_LineWrappedAngleDelta(
                g_cyz_chassis_angle_mdeg, lap_gyro_previous_mdeg);
            lap_gyro_previous_mdeg = g_cyz_chassis_angle_mdeg;
            g_lap_gyro_heading_signed_mdeg = lap_gyro_accumulated_mdeg;
            g_lap_heading_progress_mdeg = App_AbsInt32(
                lap_gyro_accumulated_mdeg);
        }

        if (!CYZ_Sensor_IsFresh(now_ms, APP_CYZ_RUNTIME_TIMEOUT_MS))
        {
            g_lap_gyro_valid = 0U;
            App_FinishChassisTest(6U, left_count, right_count);
        }
        else if (!LineSensor_IsFresh(now_ms,
                                     APP_LINE_SENSOR_MAXIMUM_AGE_MS))
        {
            App_FinishChassisTest(4U, left_count, right_count);
        }
        else if ((!lap_stop_requested) &&
                 (run_elapsed_ms >= APP_LINE_FOLLOW_TEST_TIMEOUT_MS))
        {
            App_FinishChassisTest(3U, left_count, right_count);
        }
        else
        {
            if (lap_line_checked_frame_count != g_line_valid_frame_count)
            {
                lap_line_checked_frame_count = g_line_valid_frame_count;
                if (g_lap_stop_line_armed == 0U)
                {
                    if ((g_line_active_count > 0U) &&
                        (g_line_active_count <=
                         APP_LINE_STOP_CLEAR_MAX_ACTIVE_COUNT))
                    {
                        lap_line_clear_stable_frame_count++;
                        if (lap_line_clear_stable_frame_count >=
                            APP_LINE_STOP_CLEAR_STABLE_FRAMES)
                        {
                            g_lap_stop_line_armed = 1U;
                            lap_stop_line_stable_frame_count = 0U;
                        }
                    }
                    else
                    {
                        lap_line_clear_stable_frame_count = 0U;
                    }
                }
                else if (!lap_stop_requested)
                {
                    if (g_line_active_count >
                        g_lap_line_active_count_max)
                    {
                        g_lap_line_active_count_max =
                            g_line_active_count;
                    }
                    if ((run_elapsed_ms >= APP_LINE_STOP_MIN_RUN_MS) &&
                        (g_lap_heading_progress_mdeg >=
                         APP_LINE_STOP_MIN_HEADING_MDEG) &&
                        (g_line_active_count >=
                         APP_LINE_STOP_ACTIVE_COUNT))
                    {
                        lap_stop_line_stable_frame_count++;
                        if (lap_stop_line_stable_frame_count >
                            g_lap_stop_candidate_max_frames)
                        {
                            g_lap_stop_candidate_max_frames =
                                lap_stop_line_stable_frame_count;
                        }
                        if (lap_stop_line_stable_frame_count >=
                            APP_LINE_STOP_DETECT_STABLE_FRAMES)
                        {
                            lap_stop_requested = true;
                            g_lap_stop_line_detected = 1U;
                            g_lap_stop_line_distance_um =
                                g_lap_distance_um;
                        }
                    }
                    else
                    {
                        lap_stop_line_stable_frame_count = 0U;
                    }
                }
            }

            if (!lap_stop_requested && (g_line_lost != 0U))
            {
                if (!lap_line_lost_active)
                {
                    lap_line_lost_active = true;
                    lap_line_lost_started_ms = now_ms;
                }
            }
            else if (g_line_lost == 0U)
            {
                lap_line_lost_active = false;
            }

            if (lap_line_lost_active &&
                ((uint32_t)(now_ms - lap_line_lost_started_ms) >=
                 APP_LINE_FOLLOW_LOST_GRACE_MS))
            {
                App_FinishChassisTest(5U, left_count, right_count);
            }
            else if (speed_sample_ready)
            {
                int32_t linear_mm_s;
                int32_t yaw_mdeg_s;
                int32_t left_command_cps;
                int32_t right_command_cps;
                int32_t line_position_x1000;
                int32_t profile_target_mm_s;
                int32_t profile_accel_limit_mm_s2;
                int32_t raw_yaw_mdeg_s;

                if (lap_stop_requested)
                {
                    profile_target_mm_s = 0;
                    profile_accel_limit_mm_s2 =
                        APP_LINE_FOLLOW_STOP_DECELERATION_MM_S2;
                    raw_yaw_mdeg_s = 0;
                    g_line_motion_phase = APP_LINE_MOTION_STOPPING;
                }
                else
                {
                    line_position_x1000 = (g_line_lost != 0U) ?
                        g_line_last_detected_position_x1000 :
                        g_line_position_x1000;
                    raw_yaw_mdeg_s = App_LineFollowerUpdate(
                        line_position_x1000);
                    if (g_lap_heading_progress_mdeg >=
                        APP_LINE_FOLLOW_SLOWDOWN_START_MDEG)
                    {
                        profile_target_mm_s =
                            APP_LINE_FOLLOW_APPROACH_MM_S;
                        profile_accel_limit_mm_s2 =
                            APP_LINE_FOLLOW_DECELERATION_MM_S2;
                        g_line_motion_phase =
                            APP_LINE_MOTION_APPROACHING;
                    }
                    else
                    {
                        profile_target_mm_s =
                            APP_LINE_FOLLOW_LINEAR_MM_S;
                        profile_accel_limit_mm_s2 =
                            APP_LINE_FOLLOW_ACCELERATION_MM_S2;
                        g_line_motion_phase =
                            APP_LINE_MOTION_ACCELERATING;
                    }
                }
                yaw_mdeg_s = App_LineYawStep(
                    line_yaw_command_mdeg_s,
                    raw_yaw_mdeg_s,
                    g_wheel_speed_elapsed_ms);
                line_yaw_command_mdeg_s = yaw_mdeg_s;
                App_LineProfileSetTarget(
                    profile_target_mm_s,
                    profile_accel_limit_mm_s2);
                linear_mm_s = App_LineProfileUpdate(
                    g_wheel_speed_elapsed_ms);
                if ((!lap_stop_requested) &&
                    (profile_target_mm_s ==
                     APP_LINE_FOLLOW_LINEAR_MM_S))
                {
                    g_line_motion_phase = App_LineProfileIsFinished() ?
                        APP_LINE_MOTION_CRUISING :
                        APP_LINE_MOTION_ACCELERATING;
                }
                g_vehicle_profile_speed_mm_s = line_profile_speed_mm_s;
                g_vehicle_acceleration_command_mm_s2 =
                    line_profile_acceleration_mm_s2;
                g_line_follow_yaw_mdeg_s = yaw_mdeg_s;
                g_chassis_linear_command_mm_s = linear_mm_s;
                g_chassis_yaw_command_mdeg_s = yaw_mdeg_s;
                ChassisKinematics_VelocityToWheelCps(
                    &chassis_kinematics,
                    linear_mm_s,
                    yaw_mdeg_s,
                    &left_command_cps,
                    &right_command_cps);
                g_chassis_translation_command_cps =
                    (left_command_cps + right_command_cps) / 2;
                g_chassis_turn_command_cps =
                    (right_command_cps - left_command_cps) / 2;
                ChassisVelocity_SetCommand(
                    &chassis_velocity,
                    g_chassis_translation_command_cps,
                    g_chassis_turn_command_cps);
                ChassisVelocity_Update(&chassis_velocity,
                                       g_wheel_speed_elapsed_ms);
                g_chassis_left_target_cps =
                    ChassisVelocity_GetLeftTarget(&chassis_velocity);
                g_chassis_right_target_cps =
                    ChassisVelocity_GetRightTarget(&chassis_velocity);

                if (lap_stop_requested &&
                    (g_chassis_left_target_cps == 0) &&
                    (g_chassis_right_target_cps == 0) &&
                    (App_AbsInt32(g_wheel_speed_left_cps) <=
                     APP_LINE_STOPPED_MAX_CPS) &&
                    (App_AbsInt32(g_wheel_speed_right_cps) <=
                     APP_LINE_STOPPED_MAX_CPS))
                {
                    lap_stopped_stable_sample_count++;
                }
                else
                {
                    lap_stopped_stable_sample_count = 0U;
                }

                if (lap_stopped_stable_sample_count >=
                    APP_LINE_STOPPED_STABLE_SAMPLES)
                {
                    g_line_follow_finished = 1U;
                    g_line_motion_phase = APP_LINE_MOTION_FINISHED;
                    g_vehicle_acceleration_command_mm_s2 = 0;
                    App_FinishChassisTest(0U, left_count, right_count);
                }
                else if ((!lap_stop_requested) &&
                         (run_elapsed_ms >=
                          APP_CLOSED_LOOP_STALL_CHECK_MS) &&
                         (App_AbsInt32(g_chassis_left_target_cps) >=
                          APP_CLOSED_LOOP_MIN_VALID_CPS) &&
                         ((App_SpeedAlongWheelTarget(
                               g_wheel_speed_left_cps,
                               g_chassis_left_target_cps) <
                           APP_STRAIGHT_STALL_MIN_MEASURED_CPS) ||
                          (App_SpeedAlongWheelTarget(
                               g_wheel_speed_right_cps,
                               g_chassis_right_target_cps) <
                           APP_STRAIGHT_STALL_MIN_MEASURED_CPS)))
                {
                    App_FinishChassisTest(1U, left_count, right_count);
                }
                else if ((App_AbsInt32(g_wheel_speed_left_cps) >
                          APP_CLOSED_LOOP_MAX_VALID_CPS) ||
                         (App_AbsInt32(g_wheel_speed_right_cps) >
                          APP_CLOSED_LOOP_MAX_VALID_CPS))
                {
                    App_FinishChassisTest(2U, left_count, right_count);
                }
                else
                {
                    g_pi_left_output_permille = WheelSpeedPI_Update(
                        &left_speed_controller,
                        g_chassis_left_target_cps,
                        g_wheel_speed_left_cps,
                        g_wheel_speed_elapsed_ms);
                    g_pi_right_output_permille = WheelSpeedPI_Update(
                        &right_speed_controller,
                        g_chassis_right_target_cps,
                        g_wheel_speed_right_cps,
                        g_wheel_speed_elapsed_ms);
                    BSP_Motor_SetSpeed(
                        BSP_MOTOR_LEFT,
                        (int16_t)g_pi_left_output_permille);
                    BSP_Motor_SetSpeed(
                        BSP_MOTOR_RIGHT,
                        (int16_t)g_pi_right_output_permille);
                }
            }
        }
    }
    else
    {
        BSP_Motor_CoastAll();
    }
#elif APP_ARC_TEST_ENABLED
    if (test_state == APP_TEST_WAITING)
    {
        if ((uint32_t)(now_ms - led_changed_ms) >= 250U)
        {
            BSP_LED_Toggle();
            led_changed_ms = now_ms;
        }

        if (((uint32_t)(now_ms - state_started_ms) >=
             APP_ARC_TEST_DELAY_MS) &&
            CYZ_Sensor_IsFresh(now_ms, APP_CYZ_MAXIMUM_AGE_MS))
        {
            BSP_LED_Set(true);
            WheelSpeedPI_Reset(&left_speed_controller);
            WheelSpeedPI_Reset(&right_speed_controller);
            ChassisVelocity_Reset(&chassis_velocity);
            ArcProfile_Start(&arc_profile,
                             g_fused_heading_mdeg,
                             APP_ARC_TEST_TARGET_MDEG);
            arc_start_distance_um = chassis_odometry.distance_um;
            test_state = APP_TEST_RUNNING;
            state_started_ms = now_ms;
        }
    }
    else if (test_state == APP_TEST_RUNNING)
    {
        if (!CYZ_Sensor_IsFresh(now_ms, APP_CYZ_MAXIMUM_AGE_MS))
        {
            App_FinishChassisTest(4U, left_count, right_count);
        }
        else if (!g_arc_finished &&
                 ((uint32_t)(now_ms - state_started_ms) >=
                  APP_ARC_TEST_TIMEOUT_MS))
        {
            App_FinishChassisTest(3U, left_count, right_count);
        }
        else if (g_arc_finished &&
                 ((uint32_t)(now_ms - angle_finished_ms) >=
                  APP_ARC_TEST_POST_STOP_MS))
        {
            App_FinishChassisTest(0U, left_count, right_count);
        }
        else if (speed_sample_ready)
        {
            int32_t linear_mm_s;
            int32_t yaw_mdeg_s;
            int32_t left_command_cps;
            int32_t right_command_cps;

            ArcProfile_Update(&arc_profile,
                              g_fused_heading_mdeg,
                              &linear_mm_s,
                              &yaw_mdeg_s);
            g_arc_remaining_mdeg = ArcProfile_GetRemainingMdeg(
                &arc_profile, g_fused_heading_mdeg);
            g_arc_actual_length_um = chassis_odometry.distance_um -
                                     arc_start_distance_um;

            if (ArcProfile_IsFinished(&arc_profile) && !g_arc_finished)
            {
                g_arc_finished = 1U;
                angle_finished_ms = now_ms;
                ChassisVelocity_Reset(&chassis_velocity);
                WheelSpeedPI_Reset(&left_speed_controller);
                WheelSpeedPI_Reset(&right_speed_controller);
            }

            g_chassis_linear_command_mm_s = linear_mm_s;
            g_chassis_yaw_command_mdeg_s = yaw_mdeg_s;
            ChassisKinematics_VelocityToWheelCps(
                &chassis_kinematics,
                linear_mm_s,
                yaw_mdeg_s,
                &left_command_cps,
                &right_command_cps);
            g_chassis_translation_command_cps =
                (left_command_cps + right_command_cps) / 2;
            g_chassis_turn_command_cps =
                (right_command_cps - left_command_cps) / 2;
            ChassisVelocity_SetCommand(
                &chassis_velocity,
                g_chassis_translation_command_cps,
                g_chassis_turn_command_cps);
            ChassisVelocity_Update(&chassis_velocity,
                                   g_wheel_speed_elapsed_ms);
            g_chassis_left_target_cps =
                ChassisVelocity_GetLeftTarget(&chassis_velocity);
            g_chassis_right_target_cps =
                ChassisVelocity_GetRightTarget(&chassis_velocity);

            if (!g_arc_finished &&
                ((uint32_t)(now_ms - state_started_ms) >=
                 APP_CLOSED_LOOP_STALL_CHECK_MS) &&
                (App_AbsInt32(g_chassis_left_target_cps) >=
                 APP_CLOSED_LOOP_MIN_VALID_CPS) &&
                ((App_SpeedAlongWheelTarget(g_wheel_speed_left_cps,
                                            g_chassis_left_target_cps) <
                  APP_STRAIGHT_STALL_MIN_MEASURED_CPS) ||
                 (App_SpeedAlongWheelTarget(g_wheel_speed_right_cps,
                                            g_chassis_right_target_cps) <
                  APP_STRAIGHT_STALL_MIN_MEASURED_CPS)))
            {
                App_FinishChassisTest(1U, left_count, right_count);
            }
            else if ((App_AbsInt32(g_wheel_speed_left_cps) >
                      APP_CLOSED_LOOP_MAX_VALID_CPS) ||
                     (App_AbsInt32(g_wheel_speed_right_cps) >
                      APP_CLOSED_LOOP_MAX_VALID_CPS))
            {
                App_FinishChassisTest(2U, left_count, right_count);
            }
            else
            {
                g_pi_left_output_permille = WheelSpeedPI_Update(
                    &left_speed_controller,
                    g_chassis_left_target_cps,
                    g_wheel_speed_left_cps,
                    g_wheel_speed_elapsed_ms);
                g_pi_right_output_permille = WheelSpeedPI_Update(
                    &right_speed_controller,
                    g_chassis_right_target_cps,
                    g_wheel_speed_right_cps,
                    g_wheel_speed_elapsed_ms);
                BSP_Motor_SetSpeed(BSP_MOTOR_LEFT,
                                   (int16_t)g_pi_left_output_permille);
                BSP_Motor_SetSpeed(BSP_MOTOR_RIGHT,
                                   (int16_t)g_pi_right_output_permille);
            }
        }
    }
    else
    {
        BSP_Motor_CoastAll();
    }
#elif APP_ANGLE_TEST_ENABLED
    if (test_state == APP_TEST_WAITING)
    {
        if ((uint32_t)(now_ms - led_changed_ms) >= 250U)
        {
            BSP_LED_Toggle();
            led_changed_ms = now_ms;
        }

        if (((uint32_t)(now_ms - state_started_ms) >=
             APP_ANGLE_TEST_DELAY_MS) &&
            CYZ_Sensor_IsFresh(now_ms, APP_CYZ_MAXIMUM_AGE_MS))
        {
            BSP_LED_Set(true);
            WheelSpeedPI_Reset(&left_speed_controller);
            WheelSpeedPI_Reset(&right_speed_controller);
            ChassisVelocity_Reset(&chassis_velocity);
            AngleProfile_Start(&angle_profile,
                               g_fused_heading_mdeg,
                               APP_ANGLE_TEST_TARGET_MDEG);
            test_state = APP_TEST_RUNNING;
            state_started_ms = now_ms;
        }
    }
    else if (test_state == APP_TEST_RUNNING)
    {
        if (!CYZ_Sensor_IsFresh(now_ms, APP_CYZ_MAXIMUM_AGE_MS))
        {
            App_FinishChassisTest(4U, left_count, right_count);
        }
        else if (!g_angle_finished &&
            ((uint32_t)(now_ms - state_started_ms) >=
             APP_ANGLE_TEST_TIMEOUT_MS))
        {
            App_FinishChassisTest(3U, left_count, right_count);
        }
        else if (g_angle_finished &&
            ((uint32_t)(now_ms - angle_finished_ms) >=
             APP_ANGLE_TEST_POST_STOP_MS))
        {
            App_FinishChassisTest(0U, left_count, right_count);
        }
        else if (speed_sample_ready)
        {
            int32_t yaw_mdeg_s;
            int32_t left_command_cps;
            int32_t right_command_cps;

            yaw_mdeg_s = AngleProfile_Update(
                &angle_profile,
                g_fused_heading_mdeg);
            g_angle_remaining_mdeg = AngleProfile_GetRemainingMdeg(
                &angle_profile,
                g_fused_heading_mdeg);

            if (AngleProfile_IsFinished(&angle_profile) &&
                !g_angle_finished)
            {
                g_angle_finished = 1U;
                angle_finished_ms = now_ms;
                ChassisVelocity_Reset(&chassis_velocity);
                WheelSpeedPI_Reset(&left_speed_controller);
                WheelSpeedPI_Reset(&right_speed_controller);
            }

            g_chassis_linear_command_mm_s = 0;
            g_chassis_yaw_command_mdeg_s = yaw_mdeg_s;
            ChassisKinematics_VelocityToWheelCps(
                &chassis_kinematics,
                0,
                yaw_mdeg_s,
                &left_command_cps,
                &right_command_cps);
            g_chassis_translation_command_cps =
                (left_command_cps + right_command_cps) / 2;
            g_chassis_turn_command_cps =
                (right_command_cps - left_command_cps) / 2;
            ChassisVelocity_SetCommand(
                &chassis_velocity,
                g_chassis_translation_command_cps,
                g_chassis_turn_command_cps);
            ChassisVelocity_Update(&chassis_velocity,
                                   g_wheel_speed_elapsed_ms);
            g_chassis_left_target_cps =
                ChassisVelocity_GetLeftTarget(&chassis_velocity);
            g_chassis_right_target_cps =
                ChassisVelocity_GetRightTarget(&chassis_velocity);

            if (!g_angle_finished &&
                ((uint32_t)(now_ms - state_started_ms) >=
                 APP_CLOSED_LOOP_STALL_CHECK_MS) &&
                (App_AbsInt32(g_chassis_left_target_cps) >=
                 APP_CLOSED_LOOP_MIN_VALID_CPS) &&
                ((App_SpeedAlongWheelTarget(g_wheel_speed_left_cps,
                                            g_chassis_left_target_cps) <
                  APP_ANGLE_STALL_MIN_MEASURED_CPS) ||
                 (App_SpeedAlongWheelTarget(g_wheel_speed_right_cps,
                                            g_chassis_right_target_cps) <
                  APP_ANGLE_STALL_MIN_MEASURED_CPS)))
            {
                App_FinishChassisTest(1U, left_count, right_count);
            }
            else if ((App_AbsInt32(g_wheel_speed_left_cps) >
                      APP_CLOSED_LOOP_MAX_VALID_CPS) ||
                     (App_AbsInt32(g_wheel_speed_right_cps) >
                      APP_CLOSED_LOOP_MAX_VALID_CPS))
            {
                App_FinishChassisTest(2U, left_count, right_count);
            }
            else
            {
                g_pi_left_output_permille = WheelSpeedPI_Update(
                    &left_speed_controller,
                    g_chassis_left_target_cps,
                    g_wheel_speed_left_cps,
                    g_wheel_speed_elapsed_ms);
                g_pi_right_output_permille = WheelSpeedPI_Update(
                    &right_speed_controller,
                    g_chassis_right_target_cps,
                    g_wheel_speed_right_cps,
                    g_wheel_speed_elapsed_ms);
                BSP_Motor_SetSpeed(BSP_MOTOR_LEFT,
                                   (int16_t)g_pi_left_output_permille);
                BSP_Motor_SetSpeed(BSP_MOTOR_RIGHT,
                                   (int16_t)g_pi_right_output_permille);
            }
        }
    }
    else
    {
        BSP_Motor_CoastAll();
    }
#elif APP_DISTANCE_TEST_ENABLED || APP_HEADING_STRAIGHT_TEST_ENABLED
    if (test_state == APP_TEST_WAITING)
    {
        if ((uint32_t)(now_ms - led_changed_ms) >= 250U)
        {
            BSP_LED_Toggle();
            led_changed_ms = now_ms;
        }

        if (((uint32_t)(now_ms - state_started_ms) >=
             APP_ACTIVE_DISTANCE_DELAY_MS)
#if APP_HEADING_STRAIGHT_TEST_ENABLED
            && CYZ_Sensor_IsFresh(now_ms, APP_CYZ_MAXIMUM_AGE_MS)
#endif
            )
        {
            BSP_LED_Set(true);
            WheelSpeedPI_Reset(&left_speed_controller);
            WheelSpeedPI_Reset(&right_speed_controller);
            ChassisVelocity_Reset(&chassis_velocity);
            DistanceProfile_Start(&distance_profile,
                                  chassis_odometry.distance_um,
                                  APP_ACTIVE_DISTANCE_TARGET_MM);
#if APP_HEADING_STRAIGHT_TEST_ENABLED
            HeadingController_Start(&heading_controller,
                                    g_fused_heading_mdeg);
            g_heading_hold_target_mdeg =
                heading_controller.target_heading_mdeg;
#endif
            test_state = APP_TEST_RUNNING;
            state_started_ms = now_ms;
        }
    }
    else if (test_state == APP_TEST_RUNNING)
    {
        if (
#if APP_HEADING_STRAIGHT_TEST_ENABLED
            !CYZ_Sensor_IsFresh(now_ms, APP_CYZ_MAXIMUM_AGE_MS))
        {
            App_FinishChassisTest(4U, left_count, right_count);
        }
        else if (!g_distance_finished &&
                 ((uint32_t)(now_ms - state_started_ms) >=
                  APP_HEADING_STRAIGHT_TIMEOUT_MS))
        {
            App_FinishChassisTest(3U, left_count, right_count);
        }
        else if (
#endif
            g_distance_finished &&
            ((uint32_t)(now_ms - distance_finished_ms) >=
             APP_ACTIVE_DISTANCE_POST_STOP_MS))
        {
            App_FinishChassisTest(0U, left_count, right_count);
        }
        else if (speed_sample_ready)
        {
            int32_t linear_mm_s;
            int32_t yaw_mdeg_s = 0;
            int32_t left_command_cps;
            int32_t right_command_cps;

            linear_mm_s = DistanceProfile_Update(
                &distance_profile,
                chassis_odometry.distance_um);
            g_distance_remaining_um = DistanceProfile_GetRemainingUm(
                &distance_profile,
                chassis_odometry.distance_um);

            if (DistanceProfile_IsFinished(&distance_profile) &&
                !g_distance_finished)
            {
                g_distance_finished = 1U;
                distance_finished_ms = now_ms;
            }

#if APP_HEADING_STRAIGHT_TEST_ENABLED
            yaw_mdeg_s = HeadingController_Update(
                &heading_controller, g_fused_heading_mdeg);
            g_heading_hold_error_mdeg = HeadingController_GetErrorMdeg(
                &heading_controller, g_fused_heading_mdeg);
            g_heading_hold_correction_mdeg_s = yaw_mdeg_s;
#endif

            g_chassis_linear_command_mm_s = linear_mm_s;
            g_chassis_yaw_command_mdeg_s = yaw_mdeg_s;
            ChassisKinematics_VelocityToWheelCps(
                &chassis_kinematics,
                linear_mm_s,
                yaw_mdeg_s,
                &left_command_cps,
                &right_command_cps);
            g_chassis_translation_command_cps =
                (left_command_cps + right_command_cps) / 2;
            g_chassis_turn_command_cps =
                (right_command_cps - left_command_cps) / 2;
            ChassisVelocity_SetCommand(
                &chassis_velocity,
                g_chassis_translation_command_cps,
                g_chassis_turn_command_cps);
            ChassisVelocity_Update(&chassis_velocity,
                                   g_wheel_speed_elapsed_ms);
            g_chassis_left_target_cps =
                ChassisVelocity_GetLeftTarget(&chassis_velocity);
            g_chassis_right_target_cps =
                ChassisVelocity_GetRightTarget(&chassis_velocity);

            if (!g_distance_finished &&
                ((uint32_t)(now_ms - state_started_ms) >=
                 APP_CLOSED_LOOP_STALL_CHECK_MS) &&
                (App_AbsInt32(g_chassis_left_target_cps) >=
                 APP_CLOSED_LOOP_MIN_VALID_CPS) &&
                ((g_wheel_speed_left_cps <
                  APP_STRAIGHT_STALL_MIN_MEASURED_CPS) ||
                 (g_wheel_speed_right_cps <
                  APP_STRAIGHT_STALL_MIN_MEASURED_CPS)))
            {
                App_FinishChassisTest(1U, left_count, right_count);
            }
            else if ((App_AbsInt32(g_wheel_speed_left_cps) >
                 APP_CLOSED_LOOP_MAX_VALID_CPS) ||
                (App_AbsInt32(g_wheel_speed_right_cps) >
                 APP_CLOSED_LOOP_MAX_VALID_CPS))
            {
                App_FinishChassisTest(2U, left_count, right_count);
            }
            else
            {
                g_pi_left_output_permille = WheelSpeedPI_Update(
                    &left_speed_controller,
                    g_chassis_left_target_cps,
                    g_wheel_speed_left_cps,
                    g_wheel_speed_elapsed_ms);
                g_pi_right_output_permille = WheelSpeedPI_Update(
                    &right_speed_controller,
                    g_chassis_right_target_cps,
                    g_wheel_speed_right_cps,
                    g_wheel_speed_elapsed_ms);
                BSP_Motor_SetSpeed(BSP_MOTOR_LEFT,
                                   (int16_t)g_pi_left_output_permille);
                BSP_Motor_SetSpeed(BSP_MOTOR_RIGHT,
                                   (int16_t)g_pi_right_output_permille);
            }
        }
    }
    else
    {
        BSP_Motor_CoastAll();
    }
#elif APP_CHASSIS_MIX_TEST_ENABLED || APP_GROUND_STRAIGHT_TEST_ENABLED
    if (test_state == APP_TEST_WAITING)
    {
        if ((uint32_t)(now_ms - led_changed_ms) >= 250U)
        {
            BSP_LED_Toggle();
            led_changed_ms = now_ms;
        }

        if ((uint32_t)(now_ms - state_started_ms) >=
            APP_ACTIVE_MOTION_DELAY_MS)
        {
            int32_t left_command_cps;
            int32_t right_command_cps;

            BSP_LED_Set(true);
            WheelSpeedPI_Reset(&left_speed_controller);
            WheelSpeedPI_Reset(&right_speed_controller);
            ChassisVelocity_Reset(&chassis_velocity);
#if APP_GROUND_STRAIGHT_TEST_ENABLED
            g_chassis_linear_command_mm_s = APP_GROUND_TEST_LINEAR_MM_S;
            g_chassis_yaw_command_mdeg_s = APP_GROUND_TEST_YAW_MDEG_S;
            ChassisKinematics_VelocityToWheelCps(
                &chassis_kinematics,
                g_chassis_linear_command_mm_s,
                g_chassis_yaw_command_mdeg_s,
                &left_command_cps,
                &right_command_cps);
            g_chassis_translation_command_cps =
                (left_command_cps + right_command_cps) / 2;
            g_chassis_turn_command_cps =
                (right_command_cps - left_command_cps) / 2;
#else
            left_command_cps = APP_CHASSIS_TEST_TRANSLATION_CPS -
                               APP_CHASSIS_TEST_TURN_CPS;
            right_command_cps = APP_CHASSIS_TEST_TRANSLATION_CPS +
                                APP_CHASSIS_TEST_TURN_CPS;
#endif
            ChassisVelocity_SetCommand(&chassis_velocity,
                                       (left_command_cps + right_command_cps) / 2,
                                       (right_command_cps - left_command_cps) / 2);
            test_state = APP_TEST_RUNNING;
            state_started_ms = now_ms;
        }
    }
    else if (test_state == APP_TEST_RUNNING)
    {
        uint32_t run_elapsed_ms = now_ms - state_started_ms;

        if (run_elapsed_ms >= (APP_ACTIVE_MOTION_RUN_MS +
                               APP_ACTIVE_MOTION_STOP_MS))
        {
            App_FinishChassisTest(0U, left_count, right_count);
        }
        else if (speed_sample_ready)
        {
            if (run_elapsed_ms >= APP_ACTIVE_MOTION_RUN_MS)
            {
                ChassisVelocity_Stop(&chassis_velocity);
            }
            ChassisVelocity_Update(&chassis_velocity,
                                   g_wheel_speed_elapsed_ms);
            g_chassis_left_target_cps =
                ChassisVelocity_GetLeftTarget(&chassis_velocity);
            g_chassis_right_target_cps =
                ChassisVelocity_GetRightTarget(&chassis_velocity);

            if ((run_elapsed_ms >= APP_CLOSED_LOOP_STALL_CHECK_MS) &&
                (run_elapsed_ms < APP_ACTIVE_MOTION_RUN_MS) &&
                ((App_SpeedAlongWheelTarget(g_wheel_speed_left_cps,
                                            g_chassis_left_target_cps) <
                  APP_CLOSED_LOOP_MIN_VALID_CPS) ||
                 (App_SpeedAlongWheelTarget(g_wheel_speed_right_cps,
                                            g_chassis_right_target_cps) <
                  APP_CLOSED_LOOP_MIN_VALID_CPS)))
            {
                App_FinishChassisTest(1U, left_count, right_count);
            }
            else if ((App_AbsInt32(g_wheel_speed_left_cps) >
                      APP_CLOSED_LOOP_MAX_VALID_CPS) ||
                     (App_AbsInt32(g_wheel_speed_right_cps) >
                      APP_CLOSED_LOOP_MAX_VALID_CPS))
            {
                App_FinishChassisTest(2U, left_count, right_count);
            }
            else
            {
                g_pi_left_output_permille = WheelSpeedPI_Update(
                    &left_speed_controller,
                    g_chassis_left_target_cps,
                    g_wheel_speed_left_cps,
                    g_wheel_speed_elapsed_ms);
                g_pi_right_output_permille = WheelSpeedPI_Update(
                    &right_speed_controller,
                    g_chassis_right_target_cps,
                    g_wheel_speed_right_cps,
                    g_wheel_speed_elapsed_ms);
                BSP_Motor_SetSpeed(BSP_MOTOR_LEFT,
                                   (int16_t)g_pi_left_output_permille);
                BSP_Motor_SetSpeed(BSP_MOTOR_RIGHT,
                                   (int16_t)g_pi_right_output_permille);

                if ((run_elapsed_ms >= APP_ACTIVE_MOTION_SETTLE_MS) &&
                    (run_elapsed_ms < APP_ACTIVE_MOTION_RUN_MS))
                {
                    pi_left_cps_sum += g_wheel_speed_left_cps;
                    pi_right_cps_sum += g_wheel_speed_right_cps;
                    pi_left_output_sum += g_pi_left_output_permille;
                    pi_right_output_sum += g_pi_right_output_permille;
                    g_pi_settled_sample_count++;
                }
            }
        }
    }
    else
    {
        BSP_Motor_CoastAll();
    }
#elif APP_DIRECTION_DIAGNOSTIC_ENABLED
    {
        uint32_t diagnostic_elapsed_ms = now_ms - state_started_ms;
        uint32_t left_end_ms = APP_DIRECTION_DIAGNOSTIC_DELAY_MS +
                               APP_DIRECTION_DIAGNOSTIC_RUN_MS;
        uint32_t pause_end_ms = left_end_ms +
                                APP_DIRECTION_DIAGNOSTIC_PAUSE_MS;
        uint32_t right_end_ms = pause_end_ms +
                                APP_DIRECTION_DIAGNOSTIC_RUN_MS;

        (void)speed_sample_ready;
        if (diagnostic_elapsed_ms < APP_DIRECTION_DIAGNOSTIC_DELAY_MS)
        {
            BSP_Motor_CoastAll();
            if ((uint32_t)(now_ms - led_changed_ms) >= 250U)
            {
                BSP_LED_Toggle();
                led_changed_ms = now_ms;
            }
        }
        else if (diagnostic_elapsed_ms < left_end_ms)
        {
            BSP_LED_Set(true);
            BSP_Motor_SetSpeed(BSP_MOTOR_LEFT,
                               APP_DIRECTION_DIAGNOSTIC_DUTY);
            BSP_Motor_Coast(BSP_MOTOR_RIGHT);
        }
        else if (diagnostic_elapsed_ms < pause_end_ms)
        {
            BSP_LED_Set(false);
            BSP_Motor_CoastAll();
        }
        else if (diagnostic_elapsed_ms < right_end_ms)
        {
            if ((uint32_t)(now_ms - led_changed_ms) >= 100U)
            {
                BSP_LED_Toggle();
                led_changed_ms = now_ms;
            }
            BSP_Motor_Coast(BSP_MOTOR_LEFT);
            BSP_Motor_SetSpeed(BSP_MOTOR_RIGHT,
                               APP_DIRECTION_DIAGNOSTIC_DUTY);
        }
        else
        {
            BSP_LED_Set(false);
            BSP_Motor_CoastAll();
        }
    }
#elif APP_CLOSED_LOOP_TEST_ENABLED
    if (test_state == APP_TEST_WAITING)
    {
        if ((uint32_t)(now_ms - led_changed_ms) >= 250U)
        {
            BSP_LED_Toggle();
            led_changed_ms = now_ms;
        }

        if ((uint32_t)(now_ms - state_started_ms) >=
            APP_CLOSED_LOOP_TEST_DELAY_MS)
        {
            BSP_LED_Set(true);
            WheelSpeedPI_Reset(&left_speed_controller);
            WheelSpeedPI_Reset(&right_speed_controller);
            test_state = APP_TEST_RUNNING;
            state_started_ms = now_ms;
        }
    }
    else if (test_state == APP_TEST_RUNNING)
    {
        uint32_t run_elapsed_ms = now_ms - state_started_ms;

        if (run_elapsed_ms >= APP_CLOSED_LOOP_TEST_RUN_MS)
        {
            App_FinishClosedLoopTest(0U, left_count, right_count);
        }
        else if (speed_sample_ready)
        {
            if ((run_elapsed_ms >= APP_CLOSED_LOOP_STALL_CHECK_MS) &&
                ((App_SpeedAlongTargetDirection(g_wheel_speed_left_cps) <
                  APP_CLOSED_LOOP_MIN_VALID_CPS) ||
                 (App_SpeedAlongTargetDirection(g_wheel_speed_right_cps) <
                  APP_CLOSED_LOOP_MIN_VALID_CPS)))
            {
                App_FinishClosedLoopTest(1U, left_count, right_count);
            }
            else if ((App_AbsInt32(g_wheel_speed_left_cps) >
                      APP_CLOSED_LOOP_MAX_VALID_CPS) ||
                     (App_AbsInt32(g_wheel_speed_right_cps) >
                      APP_CLOSED_LOOP_MAX_VALID_CPS))
            {
                App_FinishClosedLoopTest(2U, left_count, right_count);
            }
            else
            {
                g_pi_left_output_permille = WheelSpeedPI_Update(
                    &left_speed_controller,
                    APP_CLOSED_LOOP_TARGET_CPS,
                    g_wheel_speed_left_cps,
                    g_wheel_speed_elapsed_ms);
                g_pi_right_output_permille = WheelSpeedPI_Update(
                    &right_speed_controller,
                    APP_CLOSED_LOOP_TARGET_CPS,
                    g_wheel_speed_right_cps,
                    g_wheel_speed_elapsed_ms);
                BSP_Motor_SetSpeed(BSP_MOTOR_LEFT,
                                   (int16_t)g_pi_left_output_permille);
                BSP_Motor_SetSpeed(BSP_MOTOR_RIGHT,
                                   (int16_t)g_pi_right_output_permille);

                if (run_elapsed_ms >= APP_CLOSED_LOOP_SETTLE_MS)
                {
                    pi_left_cps_sum += g_wheel_speed_left_cps;
                    pi_right_cps_sum += g_wheel_speed_right_cps;
                    pi_left_output_sum += g_pi_left_output_permille;
                    pi_right_output_sum += g_pi_right_output_permille;
                    g_pi_settled_sample_count++;
                }
            }
        }
    }
    else
    {
        BSP_Motor_CoastAll();
    }
#elif APP_SPEED_TEST_ENABLED
    if (test_state == APP_TEST_WAITING)
    {
        if ((uint32_t)(now_ms - led_changed_ms) >= 250U)
        {
            BSP_LED_Toggle();
            led_changed_ms = now_ms;
        }

        if ((uint32_t)(now_ms - state_started_ms) >= APP_SPEED_TEST_DELAY_MS)
        {
            BSP_LED_Set(true);
            BSP_Motor_SetSpeed(BSP_MOTOR_LEFT,
                               APP_SPEED_TEST_DUTY_PERMILLE);
            BSP_Motor_SetSpeed(BSP_MOTOR_RIGHT,
                               APP_SPEED_TEST_DUTY_PERMILLE);
            test_state = APP_TEST_RUNNING;
            state_started_ms = now_ms;
        }
    }
    else if (test_state == APP_TEST_RUNNING)
    {
        if ((uint32_t)(now_ms - state_started_ms) >= APP_SPEED_TEST_RUN_MS)
        {
            BSP_Motor_CoastAll();
            BSP_LED_Set(false);
            g_speed_test_left_end_count = left_count;
            g_speed_test_right_end_count = right_count;
            test_state = APP_TEST_FINISHED;
            BSP_DebugUART_Write("Open-loop speed test finished\r\n");
        }
    }
    else
    {
        BSP_Motor_CoastAll();
    }
#else
    (void)speed_sample_ready;
    BSP_Motor_CoastAll();

    if ((left_count != displayed_left_count) ||
        (right_count != displayed_right_count))
    {
        displayed_left_count = left_count;
        displayed_right_count = right_count;
        encoder_activity_ms = now_ms;
        encoder_led_active = true;
        BSP_LED_Set(true);
    }
    else if (encoder_led_active &&
             ((uint32_t)(now_ms - encoder_activity_ms) >= 100U))
    {
        encoder_led_active = false;
        BSP_LED_Set(false);
    }
#endif
#if APP_LINE_FOLLOW_TEST_ENABLED
    App_UpdateMotionTelemetry(now_ms, speed_sample_ready);
#endif
}
