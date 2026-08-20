#pragma once

#include "dynamixel_controller.hpp"
#include "motion_hardware.hpp"

#include <string>

namespace irc_step {

class DynamixelMotionHardware final : public IMotionHardware {
public:
    DynamixelMotionHardware();
    explicit DynamixelMotionHardware(DynamixelMotionHardwareConfig config);
    DynamixelMotionHardware(Dxl& dxl, Dxl_Controller& controller);

    [[nodiscard]] const DynamixelMotionHardwareConfig& config() const noexcept;

    bool preflight() noexcept;
    [[nodiscard]] bool preflightReady() const noexcept;
    bool initialize() noexcept override;
    [[nodiscard]] bool ready() const noexcept override;
    bool commandPosition(
        const JointAngles& goal_deg,
        const std::vector<int>& motor_ids,
        std::uint32_t duration_ms,
        std::uint32_t acceleration_ms) noexcept override;
    bool commandDirectPosition(
        const JointAngles& goal_deg,
        const std::vector<int>& motor_ids) noexcept override;
    bool restoreDirectPlaybackProfile() noexcept override;
    bool readPresentPositions(JointAngles& positions_deg) noexcept override;
    bool holdCurrentPosition(
        std::uint32_t stop_duration_ms) noexcept override;
    bool setTorqueEnabled(bool enabled) noexcept override;
    [[nodiscard]] std::string_view lastError() const noexcept override;

private:
    void setError(std::string message) noexcept;

    Dxl dxl_;
    Dxl_Controller controller_;
    Dxl* active_dxl_;
    Dxl_Controller* active_controller_;
    Eigen::VectorXd desired_rad_;
    std::string last_error_;
    bool initialized_{false};
    bool preflight_ready_{false};
};

}  // namespace irc_step
