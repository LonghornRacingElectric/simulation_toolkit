#ifndef TEST_PUSHROD_H
#define TEST_PUSHROD_H

#include <blaze/Math.h>
#include <cmath>
#include "../kin_cpp/primary_elements/node.h"
#include "../kin_cpp/primary_elements/beam.h"
#include "../kin_cpp/secondary_elements/pushrod.h"
#include "../kin_cpp/third_party/googletest/googletest/include/gtest/gtest.h"

using namespace blaze;

class TestPushrod : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup code if needed
    }
    
    void TearDown() override {
        // Cleanup code if needed
    }
    
    // Helper function to compare values with tolerance
    void expectNear(double actual, double expected, double tolerance = 1e-6) {
        EXPECT_NEAR(actual, expected, tolerance);
    }
};

#endif // TEST_PUSHROD_H

