#ifndef TEST_STABAR_H
#define TEST_STABAR_H

#include <blaze/Math.h>
#include <cmath>
#include "../kin_cpp/primary_elements/node.h"
#include "../kin_cpp/primary_elements/beam.h"
#include "../kin_cpp/tertiary_elements/stabar.h"
#include "../kin_cpp/assets/misc_linalg.h"
#include "../kin_cpp/third_party/googletest/googletest/include/gtest/gtest.h"

using namespace blaze;

class TestStabar : public ::testing::Test {
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
    
    // Helper function to compare vectors with tolerance
    void expectVectorNear(const StaticVector<double, 3UL> &actual,
                         const StaticVector<double, 3UL> &expected,
                         double tolerance = 1e-5) {
        for (size_t i = 0; i < 3; ++i) {
            EXPECT_NEAR(actual[i], expected[i], tolerance)
                << "Vector mismatch at index " << i;
        }
    }
    
    // Helper to create a standard stabar setup for testing
    Stabar* createStandardStabar() {
        Node* left_arm_end = new Node(StaticVector<double, 3UL>{1, 2, 0});
        Node* right_arm_end = new Node(StaticVector<double, 3UL>{1, -2, 0});
        Node* left_droplink_end = new Node(StaticVector<double, 3UL>{1, 2, -0.5});
        Node* right_droplink_end = new Node(StaticVector<double, 3UL>{1, -2, -0.5});
        Node* bar_left_end = new Node(StaticVector<double, 3UL>{0, 2, 0});
        Node* bar_right_end = new Node(StaticVector<double, 3UL>{0, -2, 0});
        double torsional_stiffness = 1.0;  // Nm/rad
        
        return new Stabar(left_arm_end, right_arm_end, left_droplink_end, 
                         right_droplink_end, bar_left_end, bar_right_end, 
                         torsional_stiffness);
    }
};

#endif // TEST_STABAR_H

