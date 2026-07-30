cmake_minimum_required(VERSION 3.22)

project(24Y_Hti_TI C ASM)

option(H_TASK_APPLICATION "Run the fully ported two-wheel H-task application" OFF)

set(MSPM0_SDK_DIR "C:/software/ti/mspm0_sdk_2_11_00_07" CACHE PATH
    "TI MSPM0 SDK root")
set(SYSCONFIG_CLI
    "C:/software/ti/ccs2100/ccs/utils/sysconfig_1.28.0/sysconfig_cli.bat"
    CACHE FILEPATH "TI SysConfig command-line tool")

if(NOT EXISTS "${MSPM0_SDK_DIR}/.metadata/product.json")
    message(FATAL_ERROR "Invalid MSPM0_SDK_DIR: ${MSPM0_SDK_DIR}")
endif()
if(NOT EXISTS "${SYSCONFIG_CLI}")
    message(FATAL_ERROR "Invalid SYSCONFIG_CLI: ${SYSCONFIG_CLI}")
endif()

set(SYSCFG_INPUT "${CMAKE_CURRENT_SOURCE_DIR}/config/board.syscfg")
set(GENERATED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/generated")
set(SYSCFG_OUTPUTS
    "${GENERATED_DIR}/ti_msp_dl_config.c"
    "${GENERATED_DIR}/ti_msp_dl_config.h"
    "${GENERATED_DIR}/device.opt"
    "${GENERATED_DIR}/device.lds.genlibs"
    "${GENERATED_DIR}/device_linker.lds")

add_custom_command(
    OUTPUT ${SYSCFG_OUTPUTS}
    COMMAND "${SYSCONFIG_CLI}"
            --compiler gcc
            --product "${MSPM0_SDK_DIR}/.metadata/product.json"
            --output "${GENERATED_DIR}"
            "${SYSCFG_INPUT}"
    DEPENDS "${SYSCFG_INPUT}"
    COMMENT "Generating MSPM0 configuration with SysConfig"
    VERBATIM)

add_custom_target(sysconfig DEPENDS ${SYSCFG_OUTPUTS})

add_executable(24Y_Hti_TI
    src/main.c
    App/Src/app_main.c
    BSP/Src/bsp_debug_uart.c
    BSP/Src/bsp_led.c
    BSP/Src/bsp_encoder.c
    BSP/Src/bsp_gyro_uart.c
    BSP/Src/bsp_line_uart.c
    BSP/Src/bsp_motor.c
    Control/Src/angle_profile.c
    Control/Src/arc_profile.c
    Control/Src/chassis_kinematics.c
    Control/Src/chassis_velocity.c
    Control/Src/distance_profile.c
    Control/Src/heading_controller.c
    Control/Src/heading_estimator.c
    Control/Src/line_follower.c
    Control/Src/wheel_speed.c
    Control/Src/wheel_speed_pi.c
    Platform/Src/bsp_fault.c
    Platform/Src/bsp_time.c
    Platform/Src/startup_mspm0g350x_gcc.c
    Sensors/Src/cyz_sensor.c
    Sensors/Src/line_sensor.c
    "${GENERATED_DIR}/ti_msp_dl_config.c")
add_dependencies(24Y_Hti_TI sysconfig)

target_include_directories(24Y_Hti_TI PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/App/Inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/BSP/Inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/Control/Inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/Platform/Inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/Sensors/Inc"
    "${GENERATED_DIR}"
    "${MSPM0_SDK_DIR}/source"
    "${MSPM0_SDK_DIR}/source/third_party/CMSIS/Core/Include")

if(H_TASK_APPLICATION)
    target_compile_definitions(24Y_Hti_TI PRIVATE H_TASK_APPLICATION=1)
else()
    target_compile_definitions(24Y_Hti_TI PRIVATE H_TASK_APPLICATION=0)
endif()

target_compile_options(24Y_Hti_TI PRIVATE
    -mcpu=cortex-m0plus
    -march=armv6-m
    -mthumb
    -mfloat-abi=soft
    -ffunction-sections
    -fdata-sections
    -fno-common
    -Wall
    -Wextra
    -Werror=implicit-function-declaration
    $<$<CONFIG:Debug>:-Og>
    $<$<CONFIG:Debug>:-g3>
    $<$<CONFIG:Release>:-O2>
    "@${GENERATED_DIR}/device.opt")

target_link_options(24Y_Hti_TI PRIVATE
    -mcpu=cortex-m0plus
    -march=armv6-m
    -mthumb
    -mfloat-abi=soft
    -nostartfiles
    -static
    --specs=nano.specs
    --specs=nosys.specs
    "-Wl,-T,${GENERATED_DIR}/device.lds.genlibs"
    "-Wl,-T,${GENERATED_DIR}/device_linker.lds"
    "-Wl,-Map,${CMAKE_CURRENT_BINARY_DIR}/24Y_Hti_TI.map"
    -Wl,--gc-sections)

target_link_directories(24Y_Hti_TI PRIVATE
    "${MSPM0_SDK_DIR}/source")

target_link_libraries(24Y_Hti_TI PRIVATE
    "${MSPM0_SDK_DIR}/source/ti/driverlib/lib/gcc/m0p/mspm0g1x0x_g3x0x/driverlib.a"
    gcc c m)

set_target_properties(24Y_Hti_TI PROPERTIES SUFFIX ".out")

add_custom_command(TARGET 24Y_Hti_TI POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" -O ihex
            "$<TARGET_FILE:24Y_Hti_TI>"
            "${CMAKE_CURRENT_BINARY_DIR}/24Y_Hti_TI.hex"
    COMMAND "${CMAKE_OBJCOPY}" -O binary
            "$<TARGET_FILE:24Y_Hti_TI>"
            "${CMAKE_CURRENT_BINARY_DIR}/24Y_Hti_TI.bin"
    COMMAND "${CMAKE_SIZE}" "$<TARGET_FILE:24Y_Hti_TI>"
    COMMENT "Creating HEX/BIN images and printing image size")
