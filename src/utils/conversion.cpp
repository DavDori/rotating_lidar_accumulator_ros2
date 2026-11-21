#include "rotating_lidar_accumulator/utils/conversion.hpp"

namespace rla::utils {
std::vector<float> convToFloat(const std::vector<double>& input) {
    return std::vector<float>(input.begin(), input.end());
}
}  // namespace rla::utils