#ifndef TEST_BELLCRANK_H
#define TEST_BELLCRANK_H

#include <blaze/Math.h>
#include <cmath>
#include <vector>
#include "../kin_cpp/primary_elements/node.h"
#include "../kin_cpp/secondary_elements/bellcrank.h"
#include "../kin_cpp/assets/misc_linalg.h"
#include "../kin_cpp/third_party/googletest/googletest/include/gtest/gtest.h"

using namespace blaze;

class TestBellcrank : public ::testing::Test {
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
    
    // Helper function to round vector values for comparison (like Python's round)
    StaticVector<double, 3UL> roundVector(const StaticVector<double, 3UL> &vec, int decimals = 3) {
        double factor = std::pow(10.0, decimals);
        return StaticVector<double, 3UL>{
            std::round(vec[0] * factor) / factor,
            std::round(vec[1] * factor) / factor,
            std::round(vec[2] * factor) / factor
        };
    }
};

#endif // TEST_BELLCRANK_H

