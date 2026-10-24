#include "flexiv_hardware/cartesian_stiffness.hpp"
#include <gtest/gtest.h>
#include <limits>

TEST(CartesianStiffness, ChangesTranslationAndPreservesNominalRotation) {
    const std::array<double, 6> nominal{10000, 10000, 10000, 1500, 1400, 1300};
    const std::array<double, 6> expected{6000, 6000, 6000, 1500, 1400, 1300};
    EXPECT_EQ(flexiv_hardware::cartesian_stiffness(nominal, 6000), expected);
}

TEST(CartesianStiffness, RejectsUnsafeSettingsAndInvalidRobotLimits) {
    const std::array<double, 6> nominal{10000, 10000, 10000, 1500, 1500, 1500};
    for (double value : {0.0, -1.0, 10001.0, std::numeric_limits<double>::quiet_NaN()})
        EXPECT_THROW(flexiv_hardware::cartesian_stiffness(nominal, value), std::invalid_argument);
    auto limited = nominal;
    limited[2] = 5000;
    EXPECT_THROW(flexiv_hardware::cartesian_stiffness(limited, 6000), std::invalid_argument);
    limited = nominal;
    limited[3] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(flexiv_hardware::cartesian_stiffness(limited, 6000), std::invalid_argument);
}
