#ifndef TEST_TIE_H
#define TEST_TIE_H

#include <blaze/Math.h>
#include <cmath>
#include "../kin_cpp/primary_elements/node.h"
#include "../kin_cpp/primary_elements/beam.h"
#include "../kin_cpp/secondary_elements/kingpin.h"
#include "../kin_cpp/secondary_elements/tie.h"
#include "../kin_cpp/assets/misc_linalg.h"
#include "../kin_cpp/third_party/googletest/googletest/include/gtest/gtest.h"

using namespace blaze;

class TestTie : public ::testing::Test {
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
    
    // Helper function to compare StaticVectors with tolerance
    void expectVectorNear(const StaticVector<double, 3UL> &actual,
                         const StaticVector<double, 3UL> &expected,
                         double tolerance = 1e-6) {
        for (size_t i = 0; i < 3; ++i) {
            EXPECT_NEAR(actual[i], expected[i], tolerance)
                << "Mismatch at index " << i;
        }
    }
    
    // Helper function to calculate distance between two points
    double distance(const StaticVector<double, 3UL> &p1, 
                   const StaticVector<double, 3UL> &p2) {
        StaticVector<double, 3UL> diff = p2 - p1;
        return norm(diff);
    }
};

#endif // TEST_TIE_H

