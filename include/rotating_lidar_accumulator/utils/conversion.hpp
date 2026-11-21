#pragma once

#include <vector>

namespace rla::utils {

[[nodiscard]] std::vector<float> convToFloat(const std::vector<double>& input);

}