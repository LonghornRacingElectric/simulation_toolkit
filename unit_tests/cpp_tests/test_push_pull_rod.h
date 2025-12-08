#ifndef TEST_PUSH_PULL_ROD_H
#define TEST_PUSH_PULL_ROD_H

#include <blaze/Math.h>
#include <cmath>
#include <vector>
#include "../kin_cpp/primary_elements/node.h"
#include "../kin_cpp/primary_elements/beam.h"
#include "../kin_cpp/secondary_elements/damper.h"
#include "../kin_cpp/tertiary_elements/push_pull_rod.h"
#include "../kin_cpp/assets/misc_linalg.h"
#include "../kin_cpp/third_party/googletest/googletest/include/gtest/gtest.h"

using namespace blaze;

class TestPushPullRod : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup code if needed
    }
    
    void TearDown() override {
        // Cleanup code if needed
    }
    
    // Helper function to compare values with tolerance
    void expectNear(double actual, double expected, double tolerance = 1e-5) {
        EXPECT_NEAR(actual, expected, tolerance);
    }
    
    // Helper to create a simple damping curve
    std::vector<std::pair<double, double>> createSimpleDampingCurve() {
        return {
            {0.0, 0.0},
            {1.0, 1.0},
            {2.0, 2.0}
        };
    }
};

#endif // TEST_PUSH_PULL_ROD_H

