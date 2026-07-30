#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* Common fixed-rate wheel-speed sampling configuration. */
#define APP_SPEED_SAMPLE_PERIOD_MS        20U

/* ZDT V5 stepper driver on the former debug USART3 port (PB10/PB11). */
#define APP_ZDT_STEPPER_ENABLED                  1U
#define APP_ZDT_STEPPER_ADDRESS                  1U
#define APP_ZDT_STEPPER_PULSES_PER_REVOLUTION 3200U
#define APP_ZDT_STEPPER_TEST_ENABLED              0U
#define APP_ZDT_STEPPER_TEST_DELAY_MS           3000U
#define APP_ZDT_STEPPER_TEST_ENABLE_SETTLE_MS    200U
#define APP_ZDT_STEPPER_TEST_SPEED_RPM            60U
#define APP_ZDT_STEPPER_TEST_ACCELERATION          20U
#define APP_ZDT_STEPPER_TEST_RELATIVE_PULSES     3200
#define APP_ZDT_STEPPER_TEST_POSITION_DELAY_MS   2500U
#define APP_ZDT_STEPPER_TEST_REPLY_TIMEOUT_MS    1000U

/* CY-Z mini uses I2C1 remap: PB8=SCL, PB9=SDA, 7-bit address 0x42. */
#define APP_CYZ_USE_I2C                   1U

/* Safe CY-Z command-link test: send one angle-zero command after startup. */
#define APP_CYZ_ZERO_TEST_ENABLED         0U
#define APP_CYZ_ZERO_TEST_DELAY_MS        2000U

/* Chassis convention is positive yaw to the left; this CY-Z reports it negative. */
#define APP_CYZ_YAW_SIGN                  (-1)
#define APP_HEADING_GYRO_WEIGHT_PERMILLE  950
#define APP_CYZ_MAXIMUM_AGE_MS            100U
/* Keep startup strict, but tolerate a brief UART gap while odometry carries heading. */
#define APP_CYZ_RUNTIME_TIMEOUT_MS         500U

/* Passive diagnostic: motors stay off and PC13 displays raw D4 state. */
#define APP_LINE_POLARITY_TEST_ENABLED         0U

/* 2024 H problem, task 1: follow the 1000 mm A-to-B straight and stop. */
#define APP_H_TASK1_ENABLED                    0U
#define APP_H_TASK1_START_DELAY_MS             10000U
#define APP_H_TASK1_TARGET_MM                  1000
#define APP_H_TASK1_MAX_SPEED_MM_S             220
#define APP_H_TASK1_MIN_SPEED_MM_S             80
#define APP_H_TASK1_SLOWDOWN_MM                180
#define APP_H_TASK1_TOLERANCE_MM               5
#define APP_H_TASK1_POST_STOP_MS               1000U
#define APP_H_TASK1_TIMEOUT_MS                 12000U
#define APP_H_TASK1_LINE_KP                    18
#define APP_H_TASK1_MAX_YAW_MDEG_S             45000

/* Task 2: two unmarked 1000 mm straights joined by two marked semicircles. */
#define APP_H_TASK2_ENABLED                    0U
#define APP_H_TASK2_START_DELAY_MS             10000U
#define APP_H_TASK2_TARGET_MM                  1000
#define APP_H_TASK2_MAX_SPEED_MM_S             360
#define APP_H_TASK2_MIN_SPEED_MM_S             80
#define APP_H_TASK2_SLOWDOWN_MM                160
#define APP_H_TASK2_TOLERANCE_MM               5
#define APP_H_TASK2_POST_STOP_MS               1000U
#define APP_H_TASK2_TIMEOUT_MS                 28000U
#define APP_H_TASK2_LINE_KP                    44
#define APP_H_TASK2_MAX_YAW_MDEG_S            110000
#define APP_H_TASK2_LINE_STABLE_FRAMES          1U
#define APP_H_TASK2_LINE_STABLE_WINDOW_MS       250U
#define APP_H_TASK2_LINE_LOST_GRACE_MS          200U
#define APP_H_TASK2_LINE_MID_POSITION_X1000     1000
#define APP_H_TASK2_LINE_EDGE_POSITION_X1000    2000
#define APP_H_TASK2_LINE_MID_SPEED_MM_S         360
#define APP_H_TASK2_LINE_EDGE_SPEED_MM_S        360
#define APP_H_TASK2_CENTERING_START_MDEG         145000
#define APP_H_TASK2_CENTERING_MAX_MDEG           190000
#define APP_H_TASK2_CENTERING_STABLE_FRAMES      1U
#define APP_H_TASK2_CENTERING_SPEED_MM_S          360
#define APP_H_TASK2_CENTERING_GAIN_NUMERATOR      3
#define APP_H_TASK2_CENTERING_GAIN_DENOMINATOR    2
#define APP_H_TASK2_CENTER_DETECTED_MASK          0x18UL
#define APP_H_TASK2_ACQUIRE_SPEED_MM_S          120
#define APP_H_TASK2_ACQUIRE_YAW_MDEG_S          (-17200)
#define APP_H_TASK2_ACQUIRE_TIMEOUT_MS          3000U
#define APP_H_ACQUIRE_SWEEP_SHALLOW_START_MS     500U
#define APP_H_ACQUIRE_SWEEP_SHARP_START_MS      1300U
#define APP_H_ACQUIRE_SWEEP_NOMINAL_START_MS    2300U
#define APP_H_ACQUIRE_SWEEP_SHALLOW_YAW_MDEG_S  11000
#define APP_H_ACQUIRE_SWEEP_SHARP_YAW_MDEG_S    28000
#define APP_H_ENTRY_PRECAPTURE_MAX_AGE_MS        1000U
#define APP_H_ENDPOINT_LINE_ARM_REMAINING_MM       300
#define APP_H_DIAGONAL_LINE_ARM_REMAINING_MM       260
#define APP_H_DIAGONAL_CAPTURE_STABLE_FRAMES         2U
/* 循迹模块位于驱动轮接地点前方，检测端点后轮轴还需继续前进该距离。 */
#define APP_H_LINE_SENSOR_FORWARD_OFFSET_MM         240
#define APP_H_TASK2_PRE_ENTRY_DISTANCE_MM           360
#define APP_H_TASK3_PRE_ENTRY_DISTANCE_MM           260
#define APP_H_PRE_ENTRY_SPEED_MM_S                  360
#define APP_H_TASK3_DIAGONAL_TAIL_DISTANCE_MM       175
#define APP_H_TASK3_DIAGONAL_TAIL_SPEED_MM_S        360
/* 高速预瞄：在预计弯口前平滑加入圆弧前馈，黑线只修正模型误差。 */
#define APP_H_TASK2_PREVIEW_START_REMAINING_MM      260
#define APP_H_TASK2_PREVIEW_FULL_REMAINING_MM       210
#define APP_H_PREVIEW_ARC_YAW_MDEG_S              43000
#define APP_H_ENTRY_CAPTURE_PHASE_MS              1200U
#define APP_H_ENTRY_CAPTURE_LOST_GRACE_MS          1000U
#define APP_H_ARC_ENTRY_SPEED_MM_S                  360
#define APP_H_ARC_EXIT_SPEED_MM_S                   360
#define APP_H_ARC_NOMINAL_RADIUS_MM                   400
#define APP_H_ARC_TAIL_HEADING_KP                        3
#define APP_H_ARC_TAIL_MAX_YAW_MDEG_S                43000
#define APP_H_TASK2_ARC_SPEED_MM_S              360
#define APP_H_TASK2_ARC_BASE_YAW_MDEG_S         (-43000)
#define APP_H_TASK2_ARC_MAX_YAW_MDEG_S         150000
#define APP_H_TASK2_ARC_GYRO_TAKEOVER_MDEG      155000
#define APP_H_TASK2_ARC_END_PROGRESS_MDEG       180000

/* Continue from task 2's final A point into four crossed-route laps (task 4). */
#define APP_H_CHAINED_CROSS_ENABLED             1U
#define APP_H_CHAINED_CROSS_LOOPS               4U
#define APP_H_COMBINED_TIMEOUT_MS              180000U
/* 临时标定：第二题 D->A 满足出弯条件后停车，不进入 A->C。 */
#define APP_H_DA_EXIT_MEASUREMENT_ENABLED       0U

/* Task 3: AC diagonal, C-to-B arc, BD diagonal, D-to-A arc. */
#define APP_H_TASK3_ENABLED                    0U
#define APP_H_TASK3_START_DELAY_MS             10000U
#define APP_H_TASK3_DIAGONAL_MM                1281
#define APP_H_TASK3_AC_APPROACH_MM             1281
#define APP_H_TASK3_BD_APPROACH_MM             1281
#define APP_H_TASK3_MAX_SPEED_MM_S             360
#define APP_H_TASK3_MIN_SPEED_MM_S             80
#define APP_H_TASK3_SLOWDOWN_MM                180
#define APP_H_TASK3_TOLERANCE_MM               6
#define APP_H_TASK3_POST_STOP_MS               1000U
#define APP_H_TASK3_TIMEOUT_MS                 38000U
#define APP_H_TASK3_CORNER_ANGLE_MDEG          38660
/* A->C 单独沿原右转方向多补 2 度，不影响 B->D 和圆弧切线。 */
#define APP_H_TASK3_AC_ENTRY_OFFSET_MDEG        2000
#define APP_H_TASK3_PIVOT_LINEAR_MM_S           0
#define APP_H_TASK3_PIVOT_YAW_MDEG_S           60000
#define APP_H_TASK3_PIVOT_TOLERANCE_MDEG       800
#define APP_H_TASK3_LINE_KP                    44
#define APP_H_TASK3_MAX_LINE_YAW_MDEG_S       110000
#define APP_H_TASK3_LINE_STABLE_FRAMES         1U
#define APP_H_TASK3_LINE_LOST_GRACE_MS         200U
#define APP_H_TASK3_LINE_MID_POSITION_X1000    1000
#define APP_H_TASK3_LINE_EDGE_POSITION_X1000   2000
#define APP_H_TASK3_LINE_MID_SPEED_MM_S        360
#define APP_H_TASK3_LINE_EDGE_SPEED_MM_S       360
#define APP_H_TASK3_ACQUIRE_SPEED_MM_S         120
#define APP_H_TASK3_ACQUIRE_YAW_MDEG_S         17200
#define APP_H_TASK3_ACQUIRE_TIMEOUT_MS         3000U
#define APP_H_TASK3_ARC_SPEED_MM_S             360
#define APP_H_TASK3_ARC_BASE_YAW_MDEG_S        43000
#define APP_H_TASK3_ARC_MAX_YAW_MDEG_S        150000
#define APP_H_TASK3_ARC_GYRO_TAKEOVER_MDEG     155000
#define APP_H_TASK3_ARC_END_PROGRESS_MDEG      180000

/* Stage 4 test is enabled only immediately before its physical test. */
#define APP_SPEED_TEST_ENABLED            0U
#define APP_SPEED_TEST_DELAY_MS           10000U
#define APP_SPEED_TEST_RUN_MS             3000U
#define APP_SPEED_TEST_DUTY_PERMILLE      500

/* Direction diagnostic: left forward, pause, then right forward. */
#define APP_DIRECTION_DIAGNOSTIC_ENABLED  0U
#define APP_DIRECTION_DIAGNOSTIC_DELAY_MS 3000U
#define APP_DIRECTION_DIAGNOSTIC_RUN_MS   2000U
#define APP_DIRECTION_DIAGNOSTIC_PAUSE_MS 1000U
#define APP_DIRECTION_DIAGNOSTIC_DUTY      500

/* Stage 6 chassis mixer test. Keep disabled until explicitly downloaded. */
#define APP_CHASSIS_MIX_TEST_ENABLED       0U
#define APP_CHASSIS_MIX_TEST_DELAY_MS      10000U
#define APP_CHASSIS_MIX_TEST_RUN_MS        5000U
#define APP_CHASSIS_MIX_TEST_STOP_MS       1000U
#define APP_CHASSIS_TEST_TRANSLATION_CPS   0
#define APP_CHASSIS_TEST_TURN_CPS          3000
#define APP_CHASSIS_MAX_WHEEL_CPS          5000
#define APP_CHASSIS_SLEW_CPS_PER_SECOND    6000
#define APP_H_LINE_SLEW_CPS_PER_SECOND     24000
/*
 * 圆弧循迹时，D3/D6 及更外侧探头已经说明车身偏差较大。
 * 此时临时提高轮速目标斜坡，避免“已经检测到黑线但电机迟迟不纠偏”。
 * D4/D5 附近仍使用普通循迹斜坡，防止中线附近来回抖动。
 */
#define APP_H_LINE_FAST_ERROR_X1000        1000
#define APP_H_LINE_FAST_SLEW_CPS_PER_SECOND 90000
#define APP_H_TURN_EXIT_FAST_RESPONSE_MS   300U

/* First ground test: a short, low-speed straight movement. */
#define APP_GROUND_STRAIGHT_TEST_ENABLED   0U
#define APP_GROUND_TEST_DELAY_MS           10000U
#define APP_GROUND_TEST_RUN_MS             1500U
#define APP_GROUND_TEST_STOP_MS            1000U
#define APP_GROUND_TEST_SETTLE_MS          500U
#define APP_GROUND_TEST_LINEAR_MM_S        150
#define APP_GROUND_TEST_YAW_MDEG_S         0

/* First relative-distance test. */
#define APP_DISTANCE_TEST_ENABLED           0U
#define APP_DISTANCE_TEST_DELAY_MS          10000U
#define APP_DISTANCE_TEST_TARGET_MM         300
#define APP_DISTANCE_TEST_MAX_SPEED_MM_S    200
#define APP_DISTANCE_TEST_MIN_SPEED_MM_S    80
#define APP_DISTANCE_TEST_SLOWDOWN_MM       100
#define APP_DISTANCE_TEST_TOLERANCE_MM      5
#define APP_DISTANCE_TEST_POST_STOP_MS      1000U

/* Gyro/encoder fused straight-line distance test. */
#define APP_HEADING_STRAIGHT_TEST_ENABLED       0U
#define APP_HEADING_STRAIGHT_TEST_DELAY_MS      10000U
#define APP_HEADING_STRAIGHT_TARGET_MM          500
#define APP_HEADING_STRAIGHT_MAX_SPEED_MM_S     200
#define APP_HEADING_STRAIGHT_MIN_SPEED_MM_S     80
#define APP_HEADING_STRAIGHT_SLOWDOWN_MM        120
#define APP_HEADING_STRAIGHT_TOLERANCE_MM       5
#define APP_HEADING_STRAIGHT_POST_STOP_MS       1000U
#define APP_HEADING_STRAIGHT_TIMEOUT_MS         7000U
#define APP_HEADING_HOLD_KP_X1000               2500
#define APP_HEADING_HOLD_MAX_YAW_MDEG_S         30000

/* H-problem 400 mm radius quarter-circle validation. */
#define APP_ARC_TEST_ENABLED                     0U
#define APP_ARC_TEST_DELAY_MS                    10000U
#define APP_ARC_TEST_RADIUS_MM                   400
#define APP_ARC_TEST_TARGET_MDEG                 90000
#define APP_ARC_TEST_MAX_SPEED_MM_S              150
#define APP_ARC_TEST_MIN_SPEED_MM_S              70
#define APP_ARC_TEST_SLOWDOWN_MDEG               40000
#define APP_ARC_TEST_TOLERANCE_MDEG              1000
#define APP_ARC_TEST_POST_STOP_MS                1000U
#define APP_ARC_TEST_TIMEOUT_MS                  8000U

/*
 * Geometry-independent one-lap mode. D2..D7 follow the continuous track;
 * a wide transverse black marker is both the start reference and stop line.
 */
#define APP_LINE_FOLLOW_TEST_ENABLED             1U
#define APP_LINE_FOLLOW_TEST_DELAY_MS            10000U
#define APP_LINE_FOLLOW_TEST_TIMEOUT_MS          120000U
#define APP_LINE_FOLLOW_LINEAR_MM_S              350
#define APP_LINE_FOLLOW_ACCELERATION_MM_S2       160
#define APP_LINE_FOLLOW_DECELERATION_MM_S2       160
#define APP_LINE_FOLLOW_STOP_DECELERATION_MM_S2  250
#define APP_LINE_FOLLOW_SLOWDOWN_START_MDEG      260000
#define APP_LINE_FOLLOW_APPROACH_MM_S            100
#define APP_LINE_FOLLOW_YAW_SLEW_MDEG_S2         400000
#define APP_LINE_TURN_YAW_THRESHOLD_MDEG_S       10000
#define APP_MOTION_LOG_INTERVAL_MS                100U
#define APP_MOTION_LOG_CAPACITY                   192U
#define APP_LINE_FOLLOW_KP                       44
#define APP_LINE_FOLLOW_CENTER_KP                18
#define APP_LINE_FOLLOW_GAIN_FULL_SCALE_X1000  2500
#define APP_LINE_SENSOR_FORWARD_OFFSET_MM        240
#define APP_LINE_SENSOR_ACTIVE_SPAN_MM            65
#define APP_LINE_FOLLOW_MAX_YAW_MDEG_S          100000
#define APP_LINE_FOLLOW_SLEW_CPS_PER_SECOND     24000
#define APP_LINE_FOLLOW_LOST_GRACE_MS            150U
#define APP_LINE_STOP_ACTIVE_COUNT               3U
#define APP_LINE_STOP_CLEAR_MAX_ACTIVE_COUNT     3U
#define APP_LINE_STOP_CLEAR_STABLE_FRAMES        5U
#define APP_LINE_STOP_DETECT_STABLE_FRAMES       1U
#define APP_LINE_STOP_MIN_RUN_MS                 1000U
#define APP_LINE_STOP_MIN_HEADING_MDEG            320000
#define APP_LINE_STOPPED_MAX_CPS                 120
#define APP_LINE_STOPPED_STABLE_SAMPLES          5U
#define APP_ACCELERATION_FILTER_DIVISOR           4
#define APP_LINE_SENSOR_MAXIMUM_AGE_MS           100U

/* First encoder-only relative-angle test. */
#define APP_ANGLE_TEST_ENABLED               0U
#define APP_ANGLE_TEST_DELAY_MS              10000U
#define APP_ANGLE_TEST_TARGET_MDEG           90000
#define APP_ANGLE_TEST_MAX_YAW_MDEG_S        100000
#define APP_ANGLE_TEST_MIN_YAW_MDEG_S        35000
#define APP_ANGLE_TEST_SLOWDOWN_MDEG         50000
#define APP_ANGLE_TEST_TOLERANCE_MDEG        1000
#define APP_ANGLE_TEST_POST_STOP_MS          1000U
#define APP_ANGLE_TEST_TIMEOUT_MS            6000U

/* Stage 5 first closed-loop test. Keep disabled until the user is ready. */
#define APP_CLOSED_LOOP_TEST_ENABLED       0U
#define APP_CLOSED_LOOP_TEST_DELAY_MS      10000U
#define APP_CLOSED_LOOP_TEST_RUN_MS        5000U
#define APP_CLOSED_LOOP_SETTLE_MS          1500U
#define APP_CLOSED_LOOP_TARGET_CPS         (-4000)
#define APP_CLOSED_LOOP_STALL_CHECK_MS     750U
#define APP_CLOSED_LOOP_MIN_VALID_CPS      500
#define APP_H_PHASE_STALL_GRACE_MS          500U
#define APP_H_STALL_CONFIRM_SAMPLES           5U
#define APP_ANGLE_STALL_MIN_MEASURED_CPS   100
#define APP_STRAIGHT_STALL_MIN_MEASURED_CPS 100
#define APP_CLOSED_LOOP_MAX_VALID_CPS      8000

/* Tuned for the replacement high-reduction wheel motors. */
#define APP_SPEED_PI_KP_X1000              100
#define APP_SPEED_PI_KI_X1000_PER_SECOND   300
#define APP_SPEED_PI_FEEDFORWARD_GAIN_X1000 230
#define APP_SPEED_PI_STATIC_FEEDFORWARD_PERMILLE 50
#define APP_SPEED_PI_STATIC_FEEDFORWARD_CUTOFF_CPS 1800
#define APP_SPEED_PI_HIGH_SPEED_FEEDFORWARD_GAIN_X1000 65
#define APP_SPEED_PI_HIGH_SPEED_FEEDFORWARD_START_CPS 2500
#define APP_SPEED_PI_OUTPUT_MIN_PERMILLE   0
#define APP_SPEED_PI_OUTPUT_MAX_PERMILLE   850
#define APP_SPEED_PI_INTEGRAL_LIMIT        150

#endif /* APP_CONFIG_H */
