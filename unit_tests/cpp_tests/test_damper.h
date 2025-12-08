#ifndef TEST_DAMPER_H
#define TEST_DAMPER_H

#include <blaze/Math.h>
#include <cmath>
#include <vector>
#include "../kin_cpp/primary_elements/node.h"
#include "../kin_cpp/secondary_elements/damper.h"
#include "../kin_cpp/assets/misc_linalg.h"
#include "../kin_cpp/third_party/googletest/googletest/include/gtest/gtest.h"

using namespace blaze;

class TestDamper : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup code if needed
    }
    
    void TearDown() override {
        // Cleanup code if needed
    }
    
    // Helper function to compare vectors with tolerance
    void expectVectorNear(const std::vector<double> &actual,
                         const std::vector<double> &expected,
                         double tolerance = 1e-6) {
        ASSERT_EQ(actual.size(), expected.size()) << "Vector sizes don't match";
        for (size_t i = 0; i < actual.size(); ++i) {
            EXPECT_NEAR(actual[i], expected[i], tolerance)
                << "Mismatch at index " << i;
        }
    }
    
    // Helper function to compare vectors exactly
    void expectVectorEqual(const std::vector<double> &actual,
                          const std::vector<double> &expected) {
        ASSERT_EQ(actual.size(), expected.size()) << "Vector sizes don't match";
        for (size_t i = 0; i < actual.size(); ++i) {
            EXPECT_DOUBLE_EQ(actual[i], expected[i])
                << "Mismatch at index " << i;
        }
    }
    
    // Helper function to create a standard damping curve for testing
    std::vector<std::pair<double, double>> createStandardDampingCurve() {
        return {
            {0, 0},
            {1, 1},
            {2, std::sqrt(2)},
            {3, std::sqrt(3)},
            {4, 2}
        };
    }
};

#endif // TEST_DAMPER_H

