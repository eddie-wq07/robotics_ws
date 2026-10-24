#pragma once

#include <array>
#include <cmath>
#include <stdexcept>

namespace flexiv_hardware {

inline std::array<double, 6> cartesian_stiffness(
    const std::array<double, 6>& nominal, double linear_stiffness)
{
    if (!std::isfinite(linear_stiffness) || linear_stiffness <= 0.0)
        throw std::invalid_argument("Cartesian linear stiffness must be finite and positive");
    auto result = nominal;
    for (std::size_t i = 0; i < result.size(); ++i) {
        if (!std::isfinite(nominal[i]) || nominal[i] <= 0.0)
            throw std::invalid_argument("Invalid nominal Cartesian stiffness");
        if (i < 3) {
            if (linear_stiffness > nominal[i])
                throw std::invalid_argument("Cartesian linear stiffness exceeds robot nominal limit");
            result[i] = linear_stiffness;
        }
    }
    return result;
}

}  // namespace flexiv_hardware
