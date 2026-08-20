cmake_minimum_required(VERSION 3.16)
project(irc_humanoid_motion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

# 설치 위치 예: /usr/local/include/dynamixel_sdk/dynamixel_sdk.h
find_path(DYNAMIXEL_SDK_INCLUDE_DIR dynamixel_sdk/dynamixel_sdk.h)
find_library(DYNAMIXEL_SDK_LIBRARY NAMES dxl_x64_cpp dynamixel_sdk)

add_library(motion_core
    motion_pattern.cpp
    motion_callback.cpp
)
target_include_directories(motion_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(motion_core PRIVATE -Wall -Wextra -Wpedantic)

add_library(motion_player_core
    robot_motion_player.cpp
)
target_include_directories(motion_player_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(motion_player_core PUBLIC motion_core)
target_compile_options(motion_player_core PRIVATE -Wall -Wextra -Wpedantic)

if(PROJECT_IS_TOP_LEVEL)
    add_executable(test_motion_pattern test_motion_pattern.cpp)
    target_link_libraries(test_motion_pattern PRIVATE motion_core)
    target_compile_definitions(test_motion_pattern PRIVATE
        TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )

    add_executable(test_motion_callback test_motion_callback.cpp)
    target_link_libraries(test_motion_callback PRIVATE motion_core)

    add_executable(test_robot_motion_player test_robot_motion_player.cpp)
    target_link_libraries(test_robot_motion_player PRIVATE motion_player_core)
    target_compile_options(test_robot_motion_player PRIVATE -Wall -Wextra -Wpedantic)

    enable_testing()
    add_test(NAME motion_pattern_test COMMAND test_motion_pattern)
    add_test(NAME motion_callback_test COMMAND test_motion_callback)
    add_test(NAME robot_motion_player_test COMMAND test_robot_motion_player)
    add_test(
        NAME constructor_side_effect_source_test
        COMMAND ${CMAKE_COMMAND}
            -DSDK_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/test_constructor_side_effects.cmake
    )
endif()

if(DYNAMIXEL_SDK_INCLUDE_DIR AND DYNAMIXEL_SDK_LIBRARY)
    add_library(robot_control
        robot_motion_player_dynamixel.cpp
        dynamixel_motion_hardware.cpp
        step_dynamixel.cpp
        dynamixel_controller.cpp
    )
    target_include_directories(robot_control PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${DYNAMIXEL_SDK_INCLUDE_DIR}
    )
    target_link_libraries(robot_control PUBLIC
        motion_player_core
        Eigen3::Eigen
        Threads::Threads
        ${DYNAMIXEL_SDK_LIBRARY}
    )
    target_compile_options(robot_control PRIVATE -Wall -Wextra -Wpedantic)

    if(PROJECT_IS_TOP_LEVEL)
        add_executable(
            test_dynamixel_hardware_config
            test_dynamixel_hardware_config.cpp
        )
        target_link_libraries(test_dynamixel_hardware_config PRIVATE robot_control)
        target_compile_options(
            test_dynamixel_hardware_config PRIVATE -Wall -Wextra -Wpedantic
        )
        add_test(
            NAME dynamixel_hardware_config_test
            COMMAND test_dynamixel_hardware_config
        )

        add_executable(
            test_dynamixel_motion_hardware_preflight
            test_dynamixel_motion_hardware_preflight.cpp
        )
        target_link_libraries(
            test_dynamixel_motion_hardware_preflight PRIVATE robot_control
        )
        target_compile_options(
            test_dynamixel_motion_hardware_preflight
            PRIVATE -Wall -Wextra -Wpedantic
        )
        add_test(
            NAME dynamixel_motion_hardware_preflight_test
            COMMAND test_dynamixel_motion_hardware_preflight
        )

        add_executable(irc_robot main.cpp)
        target_link_libraries(irc_robot PRIVATE robot_control)
        target_compile_options(irc_robot PRIVATE -Wall -Wextra -Wpedantic)
    endif()
else()
    message(WARNING "DYNAMIXEL SDK not found: irc_robot target will not be built")
endif()
