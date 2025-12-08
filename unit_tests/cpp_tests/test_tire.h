#ifndef TEST_TIRE_H
#define TEST_TIRE_H

#include <blaze/Math.h>
#include <cmath>
#include "../kin_cpp/primary_elements/node.h"
#include "../kin_cpp/primary_elements/beam.h"
#include "../kin_cpp/secondary_elements/kingpin.h"
#include "../kin_cpp/tertiary_elements/tire.h"
#include "../kin_cpp/assets/misc_linalg.h"
#include "../kin_cpp/third_party/googletest/googletest/include/gtest/gtest.h"

using namespace blaze;

class TestTire : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup code if needed
    }
    
    void TearDown() override {
        // Cleanup code if needed
    }
    
    // Helper function to compare vectors with tolerance
    void expectVectorNear(const StaticVector<double, 3UL> &actual,
                         const StaticVector<double, 3UL> &expected,
                         double tolerance = 1e-6) {
        for (size_t i = 0; i < 3; ++i) {
            EXPECT_NEAR(actual[i], expected[i], tolerance)
                << "Vector mismatch at index " << i;
        }
    }
    
    // Helper function to compare values with tolerance
    void expectNear(double actual, double expected, double tolerance = 1e-6) {
        EXPECT_NEAR(actual, expected, tolerance);
    }
    
    // Helper to create a simple vertical kingpin for testing
    Kingpin* createVerticalKingpin(double height = 0.3) {
        Node* lower = new Node(StaticVector<double, 3UL>{0, 0, 0});
        Node* upper = new Node(StaticVector<double, 3UL>{0, 0, height});
        Beam* kingpin_beam = new Beam(lower, upper);
        return new Kingpin(kingpin_beam);
    }
};

#endif // TEST_TIRE_H

