#include "rotating_lidar_accumulator/utils/conversion_ros2.hpp"

namespace utils {

core::Timestamp fromROS2(const builtin_interfaces::msg::Time& msg) {
    core::Timestamp t(msg.sec, msg.nanosec);
    return t;
}

core::Duration fromROS2(const builtin_interfaces::msg::Duration& msg) {
    core::Duration t(msg.sec, msg.nanosec);
    return t;
}

builtin_interfaces::msg::Time toROS2(const core::Timestamp& t) {
    builtin_interfaces::msg::Time msg;
    msg.sec = static_cast<int32_t>(t.getSeconds());
    msg.nanosec = static_cast<uint32_t>(t.getNanoseconds());
    return msg;
}

}  // namespace utils