cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME HtiCar)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Actuators/Src/zdt_stepper.c
    App/Src/app_main.c
    BSP/Src/bsp_debug_uart.c
    BSP/Src/bsp_encoder.c
    BSP/Src/bsp_gyro_uart.c
    BSP/Src/bsp_i2c.c
    BSP/Src/bsp_i2c_callbacks.c
    BSP/Src/bsp_line_uart.c
    BSP/Src/bsp_led.c
    BSP/Src/bsp_motor.c
    BSP/Src/bsp_stepper_uart.c
    BSP/Src/bsp_uart_callbacks.c
    Control/Src/angle_profile.c
    Control/Src/arc_profile.c
    Control/Src/chassis_kinematics.c
    Control/Src/chassis_velocity.c
    Control/Src/distance_profile.c
    Control/Src/heading_estimator.c
    Control/Src/heading_controller.c
    Control/Src/line_follower.c
    Control/Src/wheel_speed.c
    Control/Src/wheel_speed_pi.c
    Sensors/Src/cyz_sensor.c
    Sensors/Src/line_sensor.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Actuators/Inc
    App/Inc
    BSP/Inc
    Control/Inc
    Sensors/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE_NAME:${CMAKE_PROJECT_NAME}>
            ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE_NAME:${CMAKE_PROJECT_NAME}>
            ${CMAKE_PROJECT_NAME}.bin
)
