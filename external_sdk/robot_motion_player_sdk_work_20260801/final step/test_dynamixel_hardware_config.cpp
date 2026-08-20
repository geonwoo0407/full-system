#include "dynamixel_motion_hardware.hpp"

#include <cassert>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

int main()
{
    const auto legacy = irc_step::LegacyDynamixelMotionHardwareConfig();
    assert(legacy.device_path == "/dev/ttyUSB0");
    assert(legacy.baud_rate == 4000000);
    assert(legacy.motor_ids.size() == 23);
    for (int id = 0; id <= 22; ++id)
        assert(legacy.motor_ids[static_cast<std::size_t>(id)] == id);

    irc_step::DynamixelMotionHardwareConfig custom;
    custom.device_path = "/dev/custom-dynamixel";
    custom.baud_rate = 1000000;
    for (int id = 7; id < 30; ++id) custom.motor_ids.push_back(id);
    irc_step::DynamixelMotionHardware hardware(custom);
    assert(hardware.config().device_path == custom.device_path);
    assert(hardware.config().baud_rate == custom.baud_rate);
    assert(hardware.config().motor_ids == custom.motor_ids);

    std::string error;
    assert(irc_step::ValidateDynamixelMotionHardwareConfig(custom, error));
    custom.device_path.clear();
    assert(!irc_step::ValidateDynamixelMotionHardwareConfig(custom, error));
    custom = legacy;
    custom.device_path = "/dev/custom-dynamixel";
    custom.baud_rate = 0;
    assert(!irc_step::ValidateDynamixelMotionHardwareConfig(custom, error));
    custom.baud_rate = -1;
    assert(!irc_step::ValidateDynamixelMotionHardwareConfig(custom, error));
    custom.baud_rate =
        static_cast<std::int64_t>(std::numeric_limits<int>::max()) + 1;
    assert(!irc_step::ValidateDynamixelMotionHardwareConfig(custom, error));
    custom = legacy;
    custom.device_path = "/dev/custom-dynamixel";
    custom.motor_ids.resize(22);
    assert(!irc_step::ValidateDynamixelMotionHardwareConfig(custom, error));
    custom = legacy;
    custom.device_path = "/dev/custom-dynamixel";
    custom.motor_ids.push_back(23);
    assert(!irc_step::ValidateDynamixelMotionHardwareConfig(custom, error));
    custom = legacy;
    custom.device_path = "/dev/custom-dynamixel";
    custom.motor_ids.back() = 253;
    assert(!irc_step::ValidateDynamixelMotionHardwareConfig(custom, error));
    custom = legacy;
    custom.device_path = "/dev/custom-dynamixel";
    custom.motor_ids.back() = custom.motor_ids.front();
    assert(!irc_step::ValidateDynamixelMotionHardwareConfig(custom, error));
    custom = legacy;
    custom.device_path = "/dev/custom-dynamixel";
    assert(irc_step::ValidateDynamixelMotionHardwareConfig(custom, error));
}
