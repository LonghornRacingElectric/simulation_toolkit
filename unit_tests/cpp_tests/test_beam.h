#ifndef TEST_BEAM_H
#define TEST_BEAM_H

#include <blaze/Math.h>
#include <cmath>
#include "../kin_cpp/primary_elements/node.h"
#include "../kin_cpp/primary_elements/beam.h"
#include "../kin_cpp/assets/misc_linalg.h"
#include "../kin_cpp/third_party/googletest/googletest/include/gtest/gtest.h"

using namespace blaze;

class TestBeam : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup code if needed
    }
    
    void TearDown() override {
        // Cleanup code if needed
    }
    
    // Helper function to compare StaticVectors with tolerance
    void expectVectorNear(const StaticVector<double, 3UL> &actual,
                         const StaticVector<double, 3UL> &expected,
                         double tolerance = 1e-6) {
        for (size_t i = 0; i < 3; ++i) {
            EXPECT_NEAR(actual[i], expected[i], tolerance)
                << "Mismatch at index " << i;
        }
    }
    
    // Helper function to compare StaticVectors exactly
    void expectVectorEqual(const StaticVector<double, 3UL> &actual,
                          const StaticVector<double, 3UL> &expected) {
        EXPECT_EQ(actual, expected);
    }
    
    // Helper function to check if a value is infinity
    bool isInfinity(double value) {
        return std::isinf(value);
    }
};

#endif // TEST_BEAM_H
