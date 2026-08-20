#include "dynamixel_motion_hardware.hpp"

#include <cmath>
#include <exception>
#include <utility>

namespace irc_step {

DynamixelMotionHardware::DynamixelMotionHardware()
    : DynamixelMotionHardware(LegacyDynamixelMotionHardwareConfig()) {}

DynamixelMotionHardware::DynamixelMotionHardware(
    DynamixelMotionHardwareConfig config)
    : dxl_(std::move(config)),
      controller_(&dxl_),
      active_dxl_(&dxl_),
      active_controller_(&controller_),
      desired_rad_(Eigen::VectorXd::Zero(NUMBER_OF_DYNAMIXELS)) {}

DynamixelMotionHardware::DynamixelMotionHardware(
    Dxl& dxl, Dxl_Controller& controller)
    : dxl_(),
      controller_(&dxl_),
      active_dxl_(&dxl),
      active_controller_(&controller),
      desired_rad_(Eigen::VectorXd::Zero(NUMBER_OF_DYNAMIXELS)) {}

const DynamixelMotionHardwareConfig&
DynamixelMotionHardware::config() const noexcept {
    return active_dxl_->Config();
}

void DynamixelMotionHardware::setError(std::string message) noexcept {
    last_error_ = std::move(message);
}

bool DynamixelMotionHardware::preflight() noexcept {
    try {
        initialized_ = false;
        preflight_ready_ = false;
        if (!active_dxl_->Preflight()) {
            setError(active_dxl_->LastError().empty()
                         ? "failed DYNAMIXEL safety preflight"
                         : std::string(active_dxl_->LastError()));
            return false;
        }
        desired_rad_ = active_controller_->GetJointTheta();
        preflight_ready_ = true;
        last_error_.clear();
        return true;
    } catch (const std::exception& error) {
        setError(error.what());
        initialized_ = false;
        preflight_ready_ = false;
        return false;
    } catch (...) {
        setError("unknown DYNAMIXEL preflight error");
        initialized_ = false;
        preflight_ready_ = false;
        return false;
    }
}

bool DynamixelMotionHardware::preflightReady() const noexcept {
    return preflight_ready_ && active_dxl_->IsReady();
}

bool DynamixelMotionHardware::initialize() noexcept {
    try {
        if (initialized_ && active_dxl_->IsReady()) {
            last_error_.clear();
            return true;
        }
        initialized_ = false;
        if (!preflightReady() && !preflight()) return false;
        if (!active_dxl_->Initialize()) {
            setError(active_dxl_->LastError().empty()
                         ? "failed to configure DYNAMIXEL motion"
                         : std::string(active_dxl_->LastError()));
            return false;
        }
        if (!active_controller_->ConfigureTimeBasedProfile()) {
            setError("failed to configure time-based profiles");
            return false;
        }
        desired_rad_ = active_controller_->GetJointTheta();
        if (!active_controller_->SetTorqueEnabled(true)) {
            setError("failed to enable motor torque");
            return false;
        }
        initialized_ = true;
        last_error_.clear();
        return true;
    } catch (const std::exception& error) {
        setError(error.what());
        initialized_ = false;
        return false;
    } catch (...) {
        setError("unknown DYNAMIXEL initialization error");
        initialized_ = false;
        return false;
    }
}

bool DynamixelMotionHardware::ready() const noexcept {
    return initialized_ && active_dxl_->IsReady();
}

bool DynamixelMotionHardware::commandPosition(
    const JointAngles& goal_deg, const std::vector<int>& motor_ids,
    std::uint32_t duration_ms, std::uint32_t acceleration_ms) noexcept {
    if (!ready()) {
        setError("DYNAMIXEL hardware is not initialized");
        return false;
    }
    for (const int motor_id : motor_ids) {
        const auto goal = goal_deg.find(motor_id);
        if (motor_id < 0
            || static_cast<std::size_t>(motor_id) >= active_dxl_->MotorCount()
            || goal == goal_deg.end()) {
            setError("invalid or missing motor goal");
            return false;
        }
        desired_rad_[motor_id] = goal->second * DEG2RAD;
    }
    if (!active_controller_->SetTimeBasedPosition(
            desired_rad_, motor_ids, duration_ms, acceleration_ms)) {
        setError("DYNAMIXEL time-profile SyncWrite failed");
        return false;
    }
    last_error_.clear();
    return true;
}

bool DynamixelMotionHardware::commandDirectPosition(
    const JointAngles& goal_deg,
    const std::vector<int>& motor_ids) noexcept {
    if (!ready()) {
        setError("DYNAMIXEL hardware is not initialized");
        return false;
    }
    for (const int motor_id : motor_ids) {
        const auto goal = goal_deg.find(motor_id);
        if (motor_id < 0
            || static_cast<std::size_t>(motor_id) >= active_dxl_->MotorCount()
            || goal == goal_deg.end()) {
            setError("invalid or missing motor goal");
            return false;
        }
        desired_rad_[motor_id] = goal->second * DEG2RAD;
    }
    if (!active_controller_->SetPosition(desired_rad_)) {
        setError("DYNAMIXEL direct Goal Position SyncWrite failed");
        return false;
    }
    last_error_.clear();
    return true;
}

bool DynamixelMotionHardware::restoreDirectPlaybackProfile() noexcept
{
    if (!ready()) {
        setError("DYNAMIXEL hardware is not initialized");
        return false;
    }
    if (!active_controller_->RestoreDirectPlaybackProfile()) {
        setError("failed to restore direct playback profile");
        return false;
    }
    last_error_.clear();
    return true;
}

bool DynamixelMotionHardware::readPresentPositions(
    JointAngles& positions_deg) noexcept {
    if (!ready()) {
        setError("DYNAMIXEL hardware is not initialized");
        return false;
    }
    try {
        const auto positions_rad = active_controller_->GetJointTheta();
        positions_deg.clear();
        for (std::size_t motor_id = 0; motor_id < active_dxl_->MotorCount(); ++motor_id)
            positions_deg[motor_id] = positions_rad[motor_id] / DEG2RAD;
        last_error_.clear();
        return true;
    } catch (const std::exception& error) {
        setError(error.what());
        return false;
    } catch (...) {
        setError("unknown Present Position read error");
        return false;
    }
}

bool DynamixelMotionHardware::holdCurrentPosition(
    std::uint32_t stop_duration_ms) noexcept {
    JointAngles current_deg;
    if (!readPresentPositions(current_deg)) return false;
    std::vector<int> motor_ids;
    motor_ids.reserve(current_deg.size());
    for (const auto& [motor_id, unused] : current_deg)
        motor_ids.push_back(motor_id);
    return commandPosition(current_deg, motor_ids, stop_duration_ms, 0);
}

bool DynamixelMotionHardware::setTorqueEnabled(bool enabled) noexcept {
    // Torque OFF invalidates motion readiness even if the write later fails.
    // The open-port preflight result remains useful and is intentionally kept.
    if (!enabled) initialized_ = false;
    if (!active_dxl_->IsReady()) {
        setError("DYNAMIXEL port is not ready");
        return false;
    }
    if (!active_controller_->SetTorqueEnabled(enabled)) {
        setError(enabled ? "failed to enable torque"
                         : "failed to disable torque");
        return false;
    }
    last_error_.clear();
    return true;
}

std::string_view DynamixelMotionHardware::lastError() const noexcept {
    return last_error_;
}

}  // namespace irc_step
