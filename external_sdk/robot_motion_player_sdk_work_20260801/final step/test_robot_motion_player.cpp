#include "mock_motion_hardware.hpp"
#include "robot_motion_player.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <thread>

using namespace irc_step;

namespace {
std::filesystem::path writeTestLibrary() {
    const auto path =
        std::filesystem::temp_directory_path() / "robot_player_test.json";
    std::ofstream(path) << R"({
      "version": 1,
      "motions": [
        {
          "name": "test_walk",
          "repeatable": true,
          "start_pose": "ready",
          "end_pose": "ready",
          "max_seq_ms": 250,
          "repeat_count": 2,
          "playback_speed": 1.0,
          "completion": {
            "position_tolerance_deg": 1.0,
            "settle_duration_ms": 0,
            "settle_timeout_ms": 120
          },
          "frames": [
            {"name":"ready","start_ms":0,"time_ms":100,
             "angles":{"1":80.0,"2":25.0},"torques":{"1":true}},
            {"name":"partial","start_ms":100,"time_ms":100,
             "angles":{"1":160.0},"torques":{"1":true}}
          ]
        },
        {
          "name": "fast_profile",
          "repeatable": true,
          "start_pose": "ready",
          "end_pose": "ready",
          "max_seq_ms": 120,
          "repeat_count": 1,
          "playback_speed": 2.0,
          "completion": {
            "position_tolerance_deg": 1.0,
            "settle_duration_ms": 0,
            "settle_timeout_ms": 120
          },
          "frames": [
            {"name":"fast","start_ms":0,"time_ms":100,
             "angles":{"1":100.0},"torques":{"1":true}}
          ]
        },
        {
          "name": "slow_profile",
          "repeatable": true,
          "start_pose": "ready",
          "end_pose": "ready",
          "max_seq_ms": 120,
          "repeat_count": 1,
          "playback_speed": 0.5,
          "completion": {
            "position_tolerance_deg": 1.0,
            "settle_duration_ms": 0,
            "settle_timeout_ms": 120
          },
          "frames": [
            {"name":"slow","start_ms":0,"time_ms":100,
             "angles":{"1":100.0},"torques":{"1":true}}
          ]
        }
      ]
    })";
    return path;
}

void spinUntilDone(RobotMotionPlayer& player) {
    const auto timeout =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (player.running() && std::chrono::steady_clock::now() < timeout) {
        player.update();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
}

int main() {
    const auto path = writeTestLibrary();

    MockMotionHardware hardware;
    RobotMotionPlayer player(path.string(), hardware);
    assert(!player.hardwareReady());
    assert(player.initialize());
    assert(player.hardwareReady());
    assert(player.contains("test_walk"));
    const auto info = player.motionInfo("test_walk");
    assert(info.timeline_duration_ms == 200);
    assert(info.expected_duration_ms == 400);
    assert(info.start_pose == "ready");
    assert(info.end_pose == "ready");
    assert(info.repeatable);

    hardware.setPresentPositions({{1, 0.0}, {2, 15.0}});
    assert(player.start("missing") == StartResult::MotionNotFound);
    assert(player.start("test_walk") == StartResult::Accepted);
    assert(player.start("test_walk") == StartResult::RejectedBusy);
    spinUntilDone(player);
    assert(player.status() == MotionStatus::Succeeded);
    assert(player.succeeded());
    assert(player.result() == MotionError::None);
    // A 100ms frame is sampled repeatedly rather than sent once.
    assert(hardware.commandCount() > 20);
    const auto& history = hardware.commandHistory();
    assert(history.front().at(1) < 1.0);
    const bool has_half_cosine_midpoint = std::any_of(
        history.begin(), history.end(),
        [](const JointAngles& sample) {
            const auto angle = sample.find(1);
            return angle != sample.end()
                && angle->second > 36.0 && angle->second < 44.0;
        });
    assert(has_half_cosine_midpoint);
    assert(std::any_of(
        history.begin(), history.end(),
        [](const JointAngles& sample) {
            const auto angle = sample.find(1);
            return angle != sample.end()
                && std::abs(angle->second - 80.0) < 1e-9;
        }));
    bool completed_first_repeat = false;
    bool restarted_second_repeat = false;
    for (const auto& sample : history) {
        const auto angle = sample.find(1);
        if (angle == sample.end()) continue;
        if (angle->second > 150.0) completed_first_repeat = true;
        if (completed_first_repeat && angle->second < 5.0)
            restarted_second_repeat = true;
    }
    assert(restarted_second_repeat);
    // The partial second frame never resets joint 2 to zero.
    assert(hardware.commandedPositions().at(2) == 25.0);

    MockMotionHardware fast_hardware;
    fast_hardware.setAutoReachGoal(false);
    fast_hardware.setPresentPositions({{1, 0.0}});
    RobotMotionPlayer fast_player(path.string(), fast_hardware);
    assert(fast_player.initialize());
    assert(fast_player.start("fast_profile") == StartResult::Accepted);
    fast_player.update();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    fast_player.update();
    assert(fast_hardware.commandedPositions().at(1) > 99.0);
    assert(fast_player.cancel() == CancelResult::Cancelled);

    MockMotionHardware slow_hardware;
    slow_hardware.setAutoReachGoal(false);
    slow_hardware.setPresentPositions({{1, 0.0}});
    RobotMotionPlayer slow_player(path.string(), slow_hardware);
    assert(slow_player.initialize());
    assert(slow_player.start("slow_profile") == StartResult::Accepted);
    slow_player.update();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    slow_player.update();
    const double slow_angle = slow_hardware.commandedPositions().at(1);
    assert(slow_angle > 10.0 && slow_angle < 20.0);
    assert(slow_player.cancel() == CancelResult::Cancelled);

    // 10 -> 350 follows the -20 degree shortest path, so midpoint is near 0.
    assert(std::abs(
        RobotMotionPlayer::interpolateShortest(10.0, 350.0, 0.5)) < 1e-9);
    assert(hardware.profileRestoreCount() == 0);

    MockMotionHardware startup_hardware;
    RobotMotionPlayer startup_player(path.string(), startup_hardware);
    assert(startup_player.initialize());
    std::vector<double> startup_target(kMotionMotorCount, 0.0);
    for (int motor_id = 0; motor_id < kMotionMotorCount; ++motor_id)
        startup_target[motor_id] = static_cast<double>(motor_id);
    assert(!startup_player.startPoseTransition({}, 1));
    assert(!startup_player.startPoseTransition(startup_target, 0));
    startup_target[7] = std::numeric_limits<double>::infinity();
    assert(!startup_player.startPoseTransition(startup_target, 1));
    startup_target[7] = 7.0;
    assert(startup_player.startPoseTransition(startup_target, 1));
    assert(startup_hardware.commandCount() == 1);
    assert(startup_hardware.lastDurationMs() == 1);
    assert(startup_hardware.lastAccelerationMs() == 0);
    assert(startup_player.start("test_walk") == StartResult::RejectedBusy);
    assert(startup_player.updateStartupPose() == MotionStatus::Running);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    assert(startup_player.updateStartupPose() == MotionStatus::Settling);
    assert(startup_hardware.profileRestoreCount() == 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(85));
    assert(startup_player.updateStartupPose() == MotionStatus::Succeeded);
    assert(startup_hardware.profileRestoreCount() == 1);
    assert(startup_player.updateStartupPose() == MotionStatus::Succeeded);
    assert(startup_hardware.profileRestoreCount() == 1);
    assert(startup_player.start("test_walk") == StartResult::Accepted);
    for (int update = 0; update < 5; ++update)
        startup_player.update();
    assert(startup_hardware.profileRestoreCount() == 1);
    assert(startup_player.cancel() == CancelResult::Cancelled);

    MockMotionHardware startup_restore_failure_hardware;
    startup_restore_failure_hardware.setProfileRestoreSuccess(false);
    RobotMotionPlayer startup_restore_failure_player(
        path.string(), startup_restore_failure_hardware);
    assert(startup_restore_failure_player.initialize());
    assert(startup_restore_failure_player.startPoseTransition(
        startup_target, 1));
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    assert(startup_restore_failure_player.updateStartupPose()
           == MotionStatus::Settling);
    std::this_thread::sleep_for(std::chrono::milliseconds(85));
    assert(startup_restore_failure_player.updateStartupPose()
           == MotionStatus::Failed);
    assert(startup_restore_failure_hardware.profileRestoreCount() == 1);
    assert(startup_restore_failure_player.result()
           == MotionError::CommunicationError);
    assert(!startup_restore_failure_player.lastError().empty());

    MockMotionHardware startup_read_failure_hardware;
    RobotMotionPlayer startup_read_failure_player(
        path.string(), startup_read_failure_hardware);
    assert(startup_read_failure_player.initialize());
    assert(startup_read_failure_player.startPoseTransition(startup_target, 1));
    startup_read_failure_hardware.setReadSuccess(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    assert(startup_read_failure_player.updateStartupPose()
           == MotionStatus::Failed);
    assert(startup_read_failure_player.result()
           == MotionError::PresentPositionReadFailed);

    const int holds_before_cancel_test = hardware.holdCount();
    assert(player.start("test_walk") == StartResult::Accepted);
    assert(player.cancel() == CancelResult::Cancelled);
    assert(player.status() == MotionStatus::Cancelled);
    assert(hardware.holdCount() == holds_before_cancel_test + 1);

    hardware.setCommandSuccess(false);
    assert(player.start("test_walk") == StartResult::Accepted);
    player.update();
    assert(player.status() == MotionStatus::Failed);
    assert(player.result() == MotionError::FrameSendFailed);
    assert(!player.lastError().empty());
    hardware.setCommandSuccess(true);

    hardware.setAutoReachGoal(false);
    hardware.setPresentPositions({{1, -30.0}});
    const int commands_before_timeout_test = hardware.commandCount();
    const auto history_before_timeout_test = hardware.commandHistory().size();
    assert(player.start("test_walk") == StartResult::Accepted);
    spinUntilDone(player);
    assert(player.status() == MotionStatus::Failed);
    assert(player.result() == MotionError::PositionTimeout);
    // Intermediate arrival error does not block samples, frames, or repeats.
    // PositionTimeout is only the final settling result.
    assert(hardware.commandCount() > commands_before_timeout_test + 20);
    assert(std::any_of(
        hardware.commandHistory().begin() + history_before_timeout_test,
        hardware.commandHistory().end(),
        [](const JointAngles& sample) {
            const auto angle = sample.find(1);
            return angle != sample.end() && angle->second > 100.0;
        }));
    assert(player.lastError().find("final Present Position")
           != std::string_view::npos);
    assert(player.lastError().find("frame goal was not reached")
           == std::string_view::npos);

    std::filesystem::remove(path);
    std::cout << "RobotMotionPlayer mock tests passed\n";
}
