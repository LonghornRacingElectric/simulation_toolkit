#ifndef TEST_DOUBLE_WISHBONE_H
#define TEST_DOUBLE_WISHBONE_H

#include <blaze/Math.h>
#include <cmath>
#include "../kin_cpp/primary_elements/node.h"
#include "../kin_cpp/primary_elements/beam.h"
#include "../kin_cpp/secondary_elements/kingpin.h"
#include "../kin_cpp/secondary_elements/wishbone.h"
#include "../kin_cpp/secondary_elements/tie.h"
#include "../kin_cpp/secondary_elements/cg.h"
#include "../kin_cpp/tertiary_elements/push_pull_rod.h"
#include "../kin_cpp/tertiary_elements/tire.h"
#include "../kin_cpp/tertiary_elements/double_wishbone.h"
#include "../kin_cpp/assets/misc_linalg.h"
#include "../kin_cpp/third_party/googletest/googletest/include/gtest/gtest.h"

using namespace blaze;

class TestDoubleWishbone : public ::testing::Test {
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
                         double tolerance = 1e-5) {
        for (size_t i = 0; i < 3; ++i) {
            EXPECT_NEAR(actual[i], expected[i], tolerance)
                << "Vector mismatch at index " << i;
        }
    }
    
    // Helper function to compare values with tolerance
    void expectNear(double actual, double expected, double tolerance = 1e-5) {
        EXPECT_NEAR(actual, expected, tolerance);
    }
    
    // Helper to create a simple DoubleWishbone setup for testing
    // Based on Python test structure
    DoubleWishbone* createSimpleDoubleWishbone() {
        // Inboard points: [upper_fore, upper_aft, lower_fore, lower_aft, tie_inboard, rod_inboard]
        StaticMatrix<double, 3UL, 6UL> inboard_points;
        column(inboard_points, 0) = StaticVector<double, 3UL>{2, 0, 1.5};  // upper_fore_inboard
        column(inboard_points, 1) = StaticVector<double, 3UL>{0, 0, 1.5};  // upper_aft_inboard
        column(inboard_points, 2) = StaticVector<double, 3UL>{2, 0, 0.5};  // lower_fore_inboard
        column(inboard_points, 3) = StaticVector<double, 3UL>{0, 0, 0.5};  // lower_aft_inboard
        column(inboard_points, 4) = StaticVector<double, 3UL>{2, 0, 0.5};  // tie_inboard
        column(inboard_points, 5) = StaticVector<double, 3UL>{1, 1, 2};    // rod_inboard
        
        // Outboard points: [upper_outboard, ?, lower_outboard, rod_outboard, tie_outboard, ?]
        StaticMatrix<double, 3UL, 6UL> outboard_points;
        column(outboard_points, 0) = StaticVector<double, 3UL>{1, 2, 1.5};  // upper_outboard
        column(outboard_points, 2) = StaticVector<double, 3UL>{1, 2, 0.5};  // lower_outboard
        column(outboard_points, 3) = StaticVector<double, 3UL>{1, 2, 1.5};  // rod_outboard
        column(outboard_points, 4) = StaticVector<double, 3UL>{2, 2, 1};     // tie_outboard
        
        // Bellcrank params: [pivot, direction, shock_outboard, shock_inboard]
        StaticMatrix<double, 3UL, 4UL> bellcrank_params;
        column(bellcrank_params, 0) = StaticVector<double, 3UL>{1, 1, 2};   // bellcrank_pivot
        column(bellcrank_params, 1) = StaticVector<double, 3UL>{0, 0, 1};    // bellcrank_direction
        column(bellcrank_params, 2) = StaticVector<double, 3UL>{1, 2, 1.5};  // shock_outboard
        column(bellcrank_params, 3) = StaticVector<double, 3UL>{1, 0, 2.5};  // shock_inboard
        
        double spring_rate = 1.0;
        double weight = 100.0;
        
        // Create CG
        Node* cg_node = new Node(StaticVector<double, 3UL>{1, 0, 1});
        CG* cg = new CG(cg_node);
        
        bool upper = true;  // Pushrod connects to upper wishbone
        StaticVector<double, 3UL> contact_patch{1, 2.5, 0};
        double inclination_angle = 0.0;  // No camber
        double toe = 0.0;                 // No toe
        double tire_radius = 0.3;         // 300mm radius
        double tire_width = 0.2;         // 200mm width
        bool show_ICs = false;
        
        return new DoubleWishbone(inboard_points, outboard_points, bellcrank_params,
                                  spring_rate, weight, cg, upper, contact_patch,
                                  inclination_angle, toe, tire_radius, tire_width, show_ICs);
    }
};

#endif // TEST_DOUBLE_WISHBONE_H

