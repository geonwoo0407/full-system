#include "dynamixel_motion_hardware.hpp"

#include <cassert>
#include <stdexcept>
#include <string>

namespace {

class FakeDxl final : public Dxl {
public:
    bool Preflight() noexcept override {
        ++preflight_calls;
        motion_configured = false;
        if (!preflight_result) {
            ready = false;
            error = preflight_error;
            return false;
        }
        ready = true;
        error.clear();
        return true;
    }

    bool Initialize() noexcept override {
        ++initialize_calls;
        if (motion_configured && ready) {
            error.clear();
            return true;
        }
        if (!initialize_result || !ready) {
            motion_configured = false;
            error = initialize_error;
            return false;
        }
        ++motion_configuration_calls;
        motion_configured = true;
        error.clear();
        return true;
    }

    bool IsReady() const override { return ready; }
    bool IsMotionConfigured() const override {
        return ready && motion_configured;
    }
    std::string_view LastError() const noexcept override { return error; }

    int preflight_calls{0};
    int initialize_calls{0};
    int motion_configuration_calls{0};
    bool preflight_result{true};
    bool initialize_result{true};
    bool ready{false};
    bool motion_configured{false};
    std::string preflight_error{"fake preflight failure"};
    std::string initialize_error{"fake initialize failure"};
    std::string error;
};

class FakeController final : public Dxl_Controller {
public:
    explicit FakeController(Dxl* dxl) : Dxl_Controller(dxl) {}

    VectorXd GetJointTheta() override {
        ++position_read_calls;
        if (!position_read_result)
            throw std::runtime_error("fake position read failure");
        return VectorXd::Zero(NUMBER_OF_DYNAMIXELS);
    }

    bool ConfigureTimeBasedProfile() override {
        ++profile_calls;
        return profile_result;
    }

    bool SetTorqueEnabled(bool enabled) override {
        if (enabled) {
            ++torque_on_calls;
            if (!torque_on_result) {
                ++torque_off_calls;
                return false;
            }
        } else {
            ++torque_off_calls;
        }
        return enabled ? torque_on_result : torque_off_result;
    }

    int position_read_calls{0};
    int profile_calls{0};
    int torque_on_calls{0};
    int torque_off_calls{0};
    bool position_read_result{true};
    bool profile_result{true};
    bool torque_on_result{true};
    bool torque_off_result{true};
};

void testConstructionDoesNotInitializeHardware()
{
    FakeDxl dxl;
    FakeController controller(&dxl);
    irc_step::DynamixelMotionHardware hardware(dxl, controller);

    assert(dxl.preflight_calls == 0);
    assert(dxl.initialize_calls == 0);
    assert(controller.position_read_calls == 0);
    assert(controller.profile_calls == 0);
    assert(controller.torque_on_calls == 0);
    assert(!hardware.preflightReady());
    assert(!hardware.ready());
}

void testPreflightDoesNotEnableTorque()
{
    FakeDxl dxl;
    FakeController controller(&dxl);
    irc_step::DynamixelMotionHardware hardware(dxl, controller);

    assert(hardware.preflight());
    assert(hardware.preflightReady());
    assert(!hardware.ready());
    assert(dxl.preflight_calls == 1);
    assert(dxl.initialize_calls == 0);
    assert(!dxl.IsMotionConfigured());
    assert(controller.position_read_calls == 1);
    assert(controller.profile_calls == 0);
    assert(controller.torque_on_calls == 0);

    assert(hardware.preflight());
    assert(dxl.preflight_calls == 2);
    assert(dxl.initialize_calls == 0);
    assert(controller.position_read_calls == 2);
    assert(controller.torque_on_calls == 0);
}

void testInitializeReusesPreflightAndEnablesTorqueOnce()
{
    FakeDxl dxl;
    FakeController controller(&dxl);
    irc_step::DynamixelMotionHardware hardware(dxl, controller);

    assert(hardware.preflight());
    assert(hardware.initialize());
    assert(hardware.ready());
    assert(dxl.preflight_calls == 1);
    assert(dxl.initialize_calls == 1);
    assert(dxl.motion_configuration_calls == 1);
    assert(controller.position_read_calls == 2);
    assert(controller.profile_calls == 1);
    assert(controller.torque_on_calls == 1);

    assert(hardware.initialize());
    assert(dxl.initialize_calls == 1);
    assert(dxl.motion_configuration_calls == 1);
    assert(controller.profile_calls == 1);
    assert(controller.torque_on_calls == 1);
}

void testPreflightFailurePreservesErrorAndNeverEnablesTorque()
{
    FakeDxl dxl;
    dxl.preflight_result = false;
    FakeController controller(&dxl);
    irc_step::DynamixelMotionHardware hardware(dxl, controller);

    assert(!hardware.preflight());
    assert(!hardware.preflightReady());
    assert(!hardware.ready());
    assert(hardware.lastError() == dxl.preflight_error);
    assert(controller.torque_on_calls == 0);

    assert(!hardware.initialize());
    assert(controller.torque_on_calls == 0);
}

void testPositionResponseFailureNeverEnablesTorque()
{
    FakeDxl dxl;
    FakeController controller(&dxl);
    controller.position_read_result = false;
    irc_step::DynamixelMotionHardware hardware(dxl, controller);

    assert(!hardware.preflight());
    assert(!hardware.preflightReady());
    assert(!hardware.ready());
    assert(hardware.lastError() == "fake position read failure");
    assert(controller.torque_on_calls == 0);
}

void testTorqueOffKeepsOpenPortPreflightMeaning()
{
    FakeDxl dxl;
    FakeController controller(&dxl);
    irc_step::DynamixelMotionHardware hardware(dxl, controller);

    assert(hardware.initialize());
    assert(hardware.setTorqueEnabled(false));
    assert(!hardware.ready());
    assert(hardware.preflightReady());
    assert(controller.torque_on_calls == 1);
    assert(controller.torque_off_calls == 1);

    assert(hardware.initialize());
    assert(dxl.initialize_calls == 2);
    assert(dxl.motion_configuration_calls == 1);
    assert(controller.torque_on_calls == 2);
}

void testPreflightFromMotionReadyDisablesTorque()
{
    FakeDxl dxl;
    FakeController controller(&dxl);
    irc_step::DynamixelMotionHardware hardware(dxl, controller);

    assert(hardware.initialize());
    assert(hardware.ready());
    assert(hardware.preflight());
    assert(hardware.preflightReady());
    assert(!hardware.ready());
    assert(dxl.preflight_calls == 2);
    assert(dxl.initialize_calls == 1);
    assert(controller.torque_on_calls == 1);
    assert(controller.torque_off_calls == 0);
}

void testMotionConfigurationFailureNeverEnablesTorque()
{
    FakeDxl dxl;
    dxl.initialize_result = false;
    FakeController controller(&dxl);
    irc_step::DynamixelMotionHardware hardware(dxl, controller);

    assert(!hardware.initialize());
    assert(hardware.preflightReady());
    assert(!hardware.ready());
    assert(!dxl.IsMotionConfigured());
    assert(controller.profile_calls == 0);
    assert(controller.torque_on_calls == 0);
}

void testTorqueOnFailureRollsBackAndDoesNotBecomeReady()
{
    FakeDxl dxl;
    FakeController controller(&dxl);
    controller.torque_on_result = false;
    irc_step::DynamixelMotionHardware hardware(dxl, controller);

    assert(!hardware.initialize());
    assert(!hardware.ready());
    assert(controller.profile_calls == 1);
    assert(controller.position_read_calls == 2);
    assert(controller.torque_on_calls == 1);
    assert(controller.torque_off_calls == 1);
}

}  // namespace

int main()
{
    testConstructionDoesNotInitializeHardware();
    testPreflightDoesNotEnableTorque();
    testInitializeReusesPreflightAndEnablesTorqueOnce();
    testPreflightFailurePreservesErrorAndNeverEnablesTorque();
    testPositionResponseFailureNeverEnablesTorque();
    testTorqueOffKeepsOpenPortPreflightMeaning();
    testPreflightFromMotionReadyDisablesTorque();
    testMotionConfigurationFailureNeverEnablesTorque();
    testTorqueOnFailureRollsBackAndDoesNotBecomeReady();
}
