#include "robot_motion_player.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <utility>

namespace irc_step {
namespace {
constexpr std::uint32_t kCancelHoldDurationMs = 50;
constexpr auto kPositionCheckInterval = std::chrono::milliseconds(20);
constexpr MotionCompletion kStartupCompletion{};
constexpr std::int64_t kMinimumTimelineFrameMs = 10;
constexpr double kLiftTargetArrivalRatio = 0.80;

bool isLiftFrame(std::string_view name) {
    constexpr std::string_view keywords[]{"발들", "들기", "오들", "왼들"};
    return std::any_of(
        std::begin(keywords), std::end(keywords),
        [name](std::string_view keyword) {
            return name.find(keyword) != std::string_view::npos;
        });
}

void mergeAngles(JointAngles& destination, const JointAngles& source) {
    for (const auto& [motor_id, degree] : source)
        destination[motor_id] = degree;
}
}

RobotMotionPlayer::RobotMotionPlayer(
    const std::string& json_path, IMotionHardware& hardware)
    : library_(MotionLibrary::loadGuiJson(json_path)),
      hardware_(&hardware) {}

RobotMotionPlayer::~RobotMotionPlayer() { shutdown(); }

bool RobotMotionPlayer::initialize() noexcept {
    if (hardware_ == nullptr) {
        fail(MotionError::InternalError, "motion hardware is not configured");
        return false;
    }
    if (!hardware_->initialize()) {
        fail(MotionError::HardwareNotReady, std::string(hardware_->lastError()));
        return false;
    }
    initialized_ = true;
    status_ = MotionStatus::Idle;
    error_ = MotionError::None;
    last_error_.clear();
    return true;
}

bool RobotMotionPlayer::hardwareReady() const noexcept {
    return initialized_ && hardware_ != nullptr && hardware_->ready();
}

std::vector<std::string> RobotMotionPlayer::motionNames() const {
    return library_.names();
}

bool RobotMotionPlayer::contains(std::string_view name) const {
    return library_.contains(std::string(name));
}

MotionInfo RobotMotionPlayer::motionInfo(std::string_view name) const {
    const std::string owned_name(name);
    const auto& pattern = library_.motion(owned_name);
    const auto nominal_once_ms = static_cast<std::int64_t>(std::ceil(
        pattern.durationMs() / pattern.playbackSpeed()));
    return MotionInfo{
        owned_name,
        pattern.durationMs(),
        nominal_once_ms * pattern.repeatCount(),
        pattern.startPose(),
        pattern.endPose(),
        pattern.repeatable(),
        pattern.repeatCount(),
        pattern.playbackSpeed(),
    };
}

StartResult RobotMotionPlayer::start(std::string_view motion_name) noexcept {
    if (running() || startup_status_ == MotionStatus::Running
        || startup_status_ == MotionStatus::Settling)
        return StartResult::RejectedBusy;
    if (!hardwareReady()) {
        fail(MotionError::HardwareNotReady, "motion hardware is not ready");
        return StartResult::HardwareNotReady;
    }
    const std::string name(motion_name);
    if (!library_.contains(name)) {
        error_ = MotionError::None;
        last_error_ = "motion not found: " + name;
        return StartResult::MotionNotFound;
    }
    try {
        pattern_ = &library_.motion(name);
        if (pattern_->frames().empty()) {
            last_error_ = "motion has no frames: " + name;
            return StartResult::InvalidMotion;
        }
        current_motion_ = name;
        next_frame_ = 0;
        repeat_ = 1;
        final_goal_deg_.clear();
        final_goal_ids_.clear();
        trajectory_start_deg_.clear();
        trajectory_initialized_ = false;
        gated_frame_ = nullptr;
        within_tolerance_ = false;
        started_at_ = Clock::now();
        status_ = MotionStatus::Running;
        error_ = MotionError::None;
        last_error_.clear();
        return StartResult::Accepted;
    } catch (const std::exception& error) {
        fail(MotionError::InternalError, error.what());
        return StartResult::InvalidMotion;
    } catch (...) {
        fail(MotionError::InternalError, "unknown motion start error");
        return StartResult::InvalidMotion;
    }
}

bool RobotMotionPlayer::startPoseTransition(
    const std::vector<double>& target_angles_deg,
    std::int64_t duration_ms) noexcept {
    if (!hardwareReady()) {
        error_ = MotionError::HardwareNotReady;
        last_error_ = "motion hardware is not ready for startup pose";
        startup_status_ = MotionStatus::Failed;
        return false;
    }
    if (running() || startup_status_ == MotionStatus::Running
        || startup_status_ == MotionStatus::Settling) {
        last_error_ = "motion player is busy";
        return false;
    }
    if (target_angles_deg.size() != kMotionMotorCount || duration_ms <= 0
        || duration_ms > static_cast<std::int64_t>(kMaxTimeProfileMs)) {
        last_error_ = "invalid startup pose motor count or duration";
        startup_status_ = MotionStatus::Failed;
        return false;
    }

    JointAngles target;
    std::vector<int> motor_ids;
    target.reserve(kMotionMotorCount);
    motor_ids.reserve(kMotionMotorCount);
    for (int motor_id = 0; motor_id < kMotionMotorCount; ++motor_id) {
        const double angle_deg = target_angles_deg[motor_id];
        if (!std::isfinite(angle_deg)) {
            last_error_ = "startup pose contains a non-finite angle";
            startup_status_ = MotionStatus::Failed;
            return false;
        }
        target[motor_id] = angle_deg;
        motor_ids.push_back(motor_id);
    }
    if (!hardware_->commandPosition(
            target, motor_ids, static_cast<std::uint32_t>(duration_ms), 0)) {
        error_ = MotionError::FrameSendFailed;
        last_error_ = hardware_->lastError().empty()
            ? "failed to write startup pose Goal Position"
            : std::string(hardware_->lastError());
        startup_status_ = MotionStatus::Failed;
        return false;
    }

    startup_target_deg_ = std::move(target);
    startup_motor_ids_ = std::move(motor_ids);
    startup_duration_ms_ = duration_ms;
    startup_started_at_ = Clock::now();
    startup_settling_started_at_ = Clock::time_point{};
    startup_within_tolerance_since_ = Clock::time_point{};
    startup_last_position_check_at_ = Clock::time_point{};
    startup_within_tolerance_ = false;
    startup_status_ = MotionStatus::Running;
    error_ = MotionError::None;
    last_error_.clear();
    return true;
}

MotionStatus RobotMotionPlayer::updateStartupPose() noexcept {
    if (startup_status_ != MotionStatus::Running
        && startup_status_ != MotionStatus::Settling)
        return startup_status_;
    try {
        const auto now = Clock::now();
        const auto moving_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - startup_started_at_).count();
        if (moving_ms < startup_duration_ms_) return MotionStatus::Running;

        if (startup_status_ == MotionStatus::Running) {
            startup_status_ = MotionStatus::Settling;
            startup_settling_started_at_ = now;
        }
        const auto settling_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - startup_settling_started_at_).count();
        if (settling_ms > kStartupCompletion.settle_timeout_ms) {
            error_ = MotionError::PositionTimeout;
            last_error_ =
                "startup pose Present Position did not reach the goal before timeout";
            startup_status_ = MotionStatus::Failed;
            return startup_status_;
        }
        if (startup_last_position_check_at_ != Clock::time_point{}
            && now - startup_last_position_check_at_ < kPositionCheckInterval)
            return startup_status_;
        startup_last_position_check_at_ = now;

        JointAngles present_deg;
        if (!hardware_->readPresentPositions(present_deg)) {
            error_ = MotionError::PresentPositionReadFailed;
            last_error_ = hardware_->lastError().empty()
                ? "failed to read startup pose Present Position"
                : std::string(hardware_->lastError());
            startup_status_ = MotionStatus::Failed;
            return startup_status_;
        }
        bool all_reached = true;
        for (const int motor_id : startup_motor_ids_) {
            const auto goal = startup_target_deg_.find(motor_id);
            const auto present = present_deg.find(motor_id);
            if (goal == startup_target_deg_.end() || present == present_deg.end()
                || std::abs(goal->second - present->second)
                    > kStartupCompletion.position_tolerance_deg) {
                all_reached = false;
                break;
            }
        }
        if (!all_reached) {
            startup_within_tolerance_ = false;
            return startup_status_;
        }
        if (!startup_within_tolerance_) {
            startup_within_tolerance_ = true;
            startup_within_tolerance_since_ = now;
            return startup_status_;
        }
        const auto stable_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - startup_within_tolerance_since_).count();
        if (stable_ms < kStartupCompletion.settle_duration_ms)
            return startup_status_;

        if (!hardware_->restoreDirectPlaybackProfile()) {
            error_ = MotionError::CommunicationError;
            last_error_ = hardware_->lastError().empty()
                ? "failed to restore direct playback profile"
                : std::string(hardware_->lastError());
            startup_status_ = MotionStatus::Failed;
            return startup_status_;
        }

        startup_status_ = MotionStatus::Succeeded;
        error_ = MotionError::None;
        last_error_.clear();
        return startup_status_;
    } catch (const std::exception& error) {
        error_ = MotionError::InternalError;
        last_error_ = error.what();
    } catch (...) {
        error_ = MotionError::InternalError;
        last_error_ = "unknown startup pose update error";
    }
    startup_status_ = MotionStatus::Failed;
    return startup_status_;
}

MotionStatus RobotMotionPlayer::update() noexcept {
    try {
        const auto now = Clock::now();
        if (status_ == MotionStatus::Running) return updateRunning(now);
        if (status_ == MotionStatus::Settling) return updateSettling(now);
        return status_;
    } catch (const std::exception& error) {
        fail(MotionError::InternalError, error.what());
    } catch (...) {
        fail(MotionError::InternalError, "unknown RobotMotionPlayer update error");
    }
    return status_;
}

MotionStatus RobotMotionPlayer::updateRunning(Clock::time_point now) {
    if (pattern_ == nullptr) {
        fail(MotionError::InternalError, "running motion has no pattern");
        return status_;
    }
    const double real_ms =
        std::chrono::duration<double, std::milli>(now - started_at_).count();
    const auto timeline_ms = static_cast<std::int64_t>(
        real_ms * pattern_->playbackSpeed());

    if (!trajectory_initialized_) {
        if (!hardware_->readPresentPositions(trajectory_start_deg_)) {
            fail(
                MotionError::PresentPositionReadFailed,
                hardware_->lastError().empty()
                    ? "failed to read trajectory start Present Position"
                    : std::string(hardware_->lastError()));
            return status_;
        }
        trajectory_initialized_ = true;
    }

    if (!gateCurrentFrame(timeline_ms)) return status_;

    if (!sendTrajectorySample(
            std::min(timeline_ms, pattern_->durationMs()))) {
        fail(
            MotionError::FrameSendFailed,
            hardware_->lastError().empty()
                ? "DYNAMIXEL trajectory sample transmission failed"
                : std::string(hardware_->lastError()));
        return status_;
    }

    if (timeline_ms < pattern_->durationMs()) return status_;
    if (repeat_ < pattern_->repeatCount()) {
        ++repeat_;
        next_frame_ = 0;
        started_at_ = now;
        gated_frame_ = nullptr;
        return status_;
    }

    status_ = MotionStatus::Settling;
    settling_started_at_ = now;
    last_position_check_at_ = Clock::time_point{};
    within_tolerance_ = false;
    return updateSettling(now);
}

bool RobotMotionPlayer::gateCurrentFrame(std::int64_t timeline_ms) {
    if (gated_frame_ == nullptr) return true;
    const auto frame_end_ms = gated_frame_->start_ms + gated_frame_->time_ms;
    if (timeline_ms < frame_end_ms) return true;

    // A sampled direct trajectory may not land on the exact frame boundary.
    // Match the GUI by confirming the final corrected Goal once, then release
    // without Present Position waiting or timeline adjustment.
    if (!sendDirectGoal(correctedFrameAngles(*gated_frame_))) {
        fail(
            MotionError::FrameSendFailed,
            hardware_->lastError().empty()
                ? "DYNAMIXEL frame boundary transmission failed"
                : std::string(hardware_->lastError()));
        return false;
    }
    gated_frame_ = nullptr;
    return false;
}

MotionStatus RobotMotionPlayer::updateSettling(Clock::time_point now) {
    if (pattern_ == nullptr) {
        fail(MotionError::InternalError, "settling motion has no pattern");
        return status_;
    }
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - settling_started_at_).count();
    if (elapsed_ms > pattern_->completion().settle_timeout_ms) {
        fail(
            MotionError::PositionTimeout,
            "final Present Position did not reach the goal before timeout");
        return status_;
    }
    if (last_position_check_at_ != Clock::time_point{}
        && now - last_position_check_at_ < kPositionCheckInterval) {
        return status_;
    }
    last_position_check_at_ = now;

    JointAngles present_deg;
    if (!hardware_->readPresentPositions(present_deg)) {
        fail(
            MotionError::PresentPositionReadFailed,
            hardware_->lastError().empty()
                ? "failed to read final Present Position"
                : std::string(hardware_->lastError()));
        return status_;
    }

    bool all_reached = !final_goal_ids_.empty();
    for (const int motor_id : final_goal_ids_) {
        const auto goal = final_goal_deg_.find(motor_id);
        const auto present = present_deg.find(motor_id);
        if (goal == final_goal_deg_.end() || present == present_deg.end()
            || std::abs(goal->second - present->second)
                > pattern_->completion().position_tolerance_deg) {
            all_reached = false;
            break;
        }
    }

    if (!all_reached) {
        within_tolerance_ = false;
        return status_;
    }
    if (!within_tolerance_) {
        within_tolerance_ = true;
        within_tolerance_since_ = now;
        return status_;
    }
    const auto stable_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - within_tolerance_since_).count();
    if (stable_ms < pattern_->completion().settle_duration_ms)
        return status_;

    status_ = MotionStatus::Succeeded;
    error_ = MotionError::None;
    last_error_.clear();
    std::cout << "[Info] Motion succeeded: " << current_motion_ << std::endl;
    return status_;
}

bool RobotMotionPlayer::running() const noexcept {
    return status_ == MotionStatus::Running
        || status_ == MotionStatus::Settling;
}

MotionStatus RobotMotionPlayer::status() const noexcept { return status_; }

bool RobotMotionPlayer::succeeded() const noexcept {
    return status_ == MotionStatus::Succeeded;
}

MotionError RobotMotionPlayer::result() const noexcept { return error_; }

std::string_view RobotMotionPlayer::lastError() const noexcept {
    return last_error_;
}

std::string_view RobotMotionPlayer::currentMotion() const noexcept {
    return current_motion_;
}

CancelResult RobotMotionPlayer::cancel() noexcept {
    if (!running()) return CancelResult::NotRunning;
    if (!hardwareReady()) {
        fail(MotionError::HardwareNotReady, "hardware is not ready for cancel");
        return CancelResult::HardwareNotReady;
    }
    if (!hardware_->holdCurrentPosition(kCancelHoldDurationMs)) {
        fail(
            MotionError::CancelFailed,
            hardware_->lastError().empty()
                ? "failed to hold current motor positions"
                : std::string(hardware_->lastError()));
        return CancelResult::HoldFailed;
    }
    status_ = MotionStatus::Cancelled;
    error_ = MotionError::None;
    last_error_.clear();
    pattern_ = nullptr;
    next_frame_ = 0;
    return CancelResult::Cancelled;
}

bool RobotMotionPlayer::emergencyStop() noexcept {
    if (hardware_ == nullptr) return false;
    const bool success = hardware_->setTorqueEnabled(false);
    initialized_ = false;
    status_ = success ? MotionStatus::Cancelled : MotionStatus::Failed;
    if (!success) {
        error_ = MotionError::CommunicationError;
        last_error_ = std::string(hardware_->lastError());
    }
    pattern_ = nullptr;
    next_frame_ = 0;
    return success;
}

void RobotMotionPlayer::stop() noexcept {
    if (running()) static_cast<void>(cancel());
}

bool RobotMotionPlayer::playBlocking(std::string_view motion_name) {
    if (start(motion_name) != StartResult::Accepted) return false;
    while (running()) {
        update();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return succeeded();
}

void RobotMotionPlayer::setJointCorrection(
    int motor_id, double correction_deg) {
    if (motor_id < 0 || motor_id >= kMotionMotorCount)
        throw std::out_of_range("invalid correction motor ID");
    corrections_deg_[motor_id] = correction_deg;
}

void RobotMotionPlayer::setCorrections(const JointAngles& corrections_deg) {
    for (const auto& [motor_id, correction] : corrections_deg)
        setJointCorrection(motor_id, correction);
}

void RobotMotionPlayer::clearJointCorrections() noexcept {
    corrections_deg_.clear();
}

void RobotMotionPlayer::setFrameCorrection(
    const std::string& motion_name, const std::string& frame_name,
    int motor_id, double correction_deg) {
    if (motion_name.empty() || frame_name.empty())
        throw std::invalid_argument("motion and frame names are required");
    if (!library_.contains(motion_name))
        throw std::out_of_range("unknown correction motion: " + motion_name);
    if (motor_id < 0 || motor_id >= kMotionMotorCount)
        throw std::out_of_range("invalid frame correction motor ID");
    const auto& frames = library_.motion(motion_name).frames();
    const bool matching_target = std::any_of(
        frames.begin(), frames.end(),
        [&](const MotionFrame& frame) {
            return frame.name == frame_name && frame.angles.contains(motor_id);
        });
    if (!matching_target)
        throw std::out_of_range(
            "motion/frame does not command motor " + std::to_string(motor_id));
    frame_corrections_deg_[{motion_name, frame_name, motor_id}] = correction_deg;
}

void RobotMotionPlayer::clearFrameCorrection(
    const std::string& motion_name, const std::string& frame_name,
    int motor_id) noexcept {
    frame_corrections_deg_.erase({motion_name, frame_name, motor_id});
}

void RobotMotionPlayer::clearFrameCorrections() noexcept {
    frame_corrections_deg_.clear();
}

void RobotMotionPlayer::clearCorrections() noexcept {
    clearJointCorrections();
    clearFrameCorrections();
}

void RobotMotionPlayer::shutdown() noexcept {
    if (hardware_ != nullptr && initialized_) {
        if (running()) static_cast<void>(cancel());
        static_cast<void>(hardware_->setTorqueEnabled(false));
    }
    initialized_ = false;
    if (status_ == MotionStatus::Running
        || status_ == MotionStatus::Settling) {
        status_ = MotionStatus::Cancelled;
    }
}

void RobotMotionPlayer::fail(
    MotionError error, std::string message) noexcept {
    error_ = error;
    last_error_ = std::move(message);
    status_ = MotionStatus::Failed;
    pattern_ = nullptr;
    next_frame_ = 0;
}

JointAngles RobotMotionPlayer::correctedFrameAngles(
    const MotionFrame& frame) const {
    JointAngles corrected = frame.angles;
    for (auto& [motor_id, degree] : corrected) {
        if (const auto correction = corrections_deg_.find(motor_id);
            correction != corrections_deg_.end()) {
            degree += correction->second;
        }
        if (const auto correction = frame_corrections_deg_.find(
                {current_motion_, frame.name, motor_id});
            correction != frame_corrections_deg_.end()) {
            degree += correction->second;
        }
    }
    return corrected;
}

double RobotMotionPlayer::interpolateShortest(
    double from_deg, double to_deg, double progress) {
    double delta = std::fmod(to_deg - from_deg + 180.0, 360.0);
    if (delta < 0.0) delta += 360.0;
    return from_deg + (delta - 180.0) * progress;
}

JointAngles RobotMotionPlayer::accumulatedPoseBefore(
    const MotionFrame& active_frame) const {
    JointAngles pose = trajectory_start_deg_;
    const auto& frames = pattern_->frames();
    if (&active_frame == &frames.front() && active_frame.start_ms > 0) {
        for (const auto& frame : frames) {
            const auto corrected = correctedFrameAngles(frame);
            mergeAngles(pose, corrected);
        }
        return pose;
    }
    for (const auto& frame : frames) {
        if (frame.start_ms + frame.time_ms > active_frame.start_ms) break;
        const auto corrected = correctedFrameAngles(frame);
        mergeAngles(pose, corrected);
    }
    return pose;
}

JointAngles RobotMotionPlayer::sampledPose(std::int64_t timeline_ms) const {
    const auto& frames = pattern_->frames();
    const MotionFrame* active = nullptr;
    for (const auto& frame : frames) {
        const auto frame_end_ms = frame.start_ms + frame.time_ms;
        if ((frame.start_ms <= timeline_ms && timeline_ms < frame_end_ms)
            || (timeline_ms == pattern_->durationMs()
                && frame_end_ms == pattern_->durationMs())) {
            active = &frame;
            break;
        }
    }

    if (active != nullptr) {
        const auto previous = accumulatedPoseBefore(*active);
        const auto corrected = correctedFrameAngles(*active);
        const auto duration_ms = std::max(
            kMinimumTimelineFrameMs, active->time_ms);
        const double arrival_ratio =
            isLiftFrame(active->name) ? kLiftTargetArrivalRatio : 1.0;
        const double elapsed_ms = std::clamp(
            static_cast<double>(timeline_ms - active->start_ms),
            0.0, static_cast<double>(duration_ms));
        const double linear_progress = std::min(
            1.0, elapsed_ms / (duration_ms * arrival_ratio));
        const double progress =
            0.5 - 0.5 * std::cos(std::numbers::pi * linear_progress);
        JointAngles target;
        for (const auto& [motor_id, final_deg] : corrected) {
            const auto start = previous.find(motor_id);
            const double start_deg =
                start == previous.end() ? final_deg : start->second;
            target[motor_id] = interpolateShortest(
                start_deg, final_deg, progress);
        }
        return target;
    }

    JointAngles hold = trajectory_start_deg_;
    const auto first_start_ms = frames.front().start_ms;
    for (const auto& frame : frames) {
        if (timeline_ms < first_start_ms
            || frame.start_ms + frame.time_ms <= timeline_ms) {
            const auto corrected = correctedFrameAngles(frame);
            mergeAngles(hold, corrected);
        }
    }
    return hold;
}

bool RobotMotionPlayer::sendTrajectorySample(std::int64_t timeline_ms) {
    const auto& frames = pattern_->frames();
    gated_frame_ = nullptr;
    for (const auto& frame : frames) {
        const auto frame_end_ms = frame.start_ms + frame.time_ms;
        if ((frame.start_ms <= timeline_ms && timeline_ms < frame_end_ms)
            || (timeline_ms == pattern_->durationMs()
                && frame_end_ms == pattern_->durationMs())) {
            gated_frame_ = &frame;
            break;
        }
    }
    return sendDirectGoal(sampledPose(timeline_ms));
}

bool RobotMotionPlayer::sendDirectGoal(const JointAngles& goal_deg) {
    std::vector<int> target_ids;
    target_ids.reserve(goal_deg.size());
    for (const auto& [motor_id, degree] : goal_deg) {
        if (motor_id < 0 || motor_id >= kMotionMotorCount)
            throw std::runtime_error(
                "invalid motor ID in JSON: " + std::to_string(motor_id));
        target_ids.push_back(motor_id);
        final_goal_deg_[motor_id] = degree;
    }
    if (target_ids.empty()) return true;
    final_goal_ids_.clear();
    final_goal_ids_.reserve(final_goal_deg_.size());
    for (const auto& [motor_id, unused] : final_goal_deg_)
        final_goal_ids_.push_back(motor_id);
    return hardware_->commandDirectPosition(goal_deg, target_ids);
}

}  // namespace irc_step
