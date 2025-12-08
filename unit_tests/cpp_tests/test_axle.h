#ifndef TEST_AXLE_H
#define TEST_AXLE_H

#include <blaze/Math.h>
#include <cmath>
#include "../kin_cpp/tertiary_elements/double_wishbone.h"
#include "../kin_cpp/secondary_elements/cg.h"
#include "../kin_cpp/quaternary_elements/axle.h"
#include "../kin_cpp/third_party/googletest/googletest/include/gtest/gtest.h"

using namespace blaze;

class TestAxle : public ::testing::Test {
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
    
    // Helper to create a simple axle setup for testing
    // This creates a basic axle with left and right DoubleWishbone assemblies
    Axle* createSimpleAxle() {
        // Create nodes for left side
        StaticMatrix<double, 3UL, 6UL> left_inboard;
        column(left_inboard, 0) = StaticVector<double, 3UL>{2, 2, 0};   // upper_fore
        column(left_inboard, 1) = StaticVector<double, 3UL>{0, 2, 0};   // upper_aft
        column(left_inboard, 2) = StaticVector<double, 3UL>{2, 2, 0};   // lower_fore
        column(left_inboard, 3) = StaticVector<double, 3UL>{0, 2, 0};   // lower_aft
        column(left_inboard, 4) = StaticVector<double, 3UL>{2, 2, 0};   // tie_inboard
        column(left_inboard, 5) = StaticVector<double, 3UL>{1, 2, 2};    // rod_inboard
        
        StaticMatrix<double, 3UL, 6UL> left_outboard;
        column(left_outboard, 0) = StaticVector<double, 3UL>{1, 4, 1};   // upper_outboard
        column(left_outboard, 1) = StaticVector<double, 3UL>{0, 0, 0};   // unused
        column(left_outboard, 2) = StaticVector<double, 3UL>{1, 4, 0};   // lower_outboard
        column(left_outboard, 3) = StaticVector<double, 3UL>{1, 4, 1};   // rod_outboard
        column(left_outboard, 4) = StaticVector<double, 3UL>{2, 4, 0.5}; // tie_outboard
        column(left_outboard, 5) = StaticVector<double, 3UL>{0, 0, 0};   // unused
        
        StaticMatrix<double, 3UL, 4UL> left_bellcrank;
        column(left_bellcrank, 0) = StaticVector<double, 3UL>{1, 2, 2};   // pivot
        column(left_bellcrank, 1) = StaticVector<double, 3UL>{1, 0, 0};   // direction
        column(left_bellcrank, 2) = StaticVector<double, 3UL>{1, 4, 1};   // shock_outboard
        column(left_bellcrank, 3) = StaticVector<double, 3UL>{1, 0, 2.5}; // shock_inboard
        
        double spring_rate = 1.0;
        double weight = 100.0;
        Node* cg_node = new Node(StaticVector<double, 3UL>{1, 0, 1});
        CG* cg = new CG(cg_node);
        
        bool upper = true;
        StaticVector<double, 3UL> left_cp{1, 4.5, -0.5};
        double inclination_angle = 0.0;
        double toe = 0.0;
        double tire_radius = 0.3;
        double tire_width = 0.2;
        bool show_ICs = false;
        
        DoubleWishbone* left_dw = new DoubleWishbone(left_inboard, left_outboard, left_bellcrank,
                                                      spring_rate, weight, cg, upper, left_cp,
                                                      inclination_angle, toe, tire_radius, tire_width, show_ICs);
        
        // Create nodes for right side (mirrored)
        StaticMatrix<double, 3UL, 6UL> right_inboard;
        column(right_inboard, 0) = StaticVector<double, 3UL>{2, -2, 0};
        column(right_inboard, 1) = StaticVector<double, 3UL>{0, -2, 0};
        column(right_inboard, 2) = StaticVector<double, 3UL>{2, -2, 0};
        column(right_inboard, 3) = StaticVector<double, 3UL>{0, -2, 0};
        column(right_inboard, 4) = StaticVector<double, 3UL>{2, -2, 0};
        column(right_inboard, 5) = StaticVector<double, 3UL>{1, -2, 2};
        
        StaticMatrix<double, 3UL, 6UL> right_outboard;
        column(right_outboard, 0) = StaticVector<double, 3UL>{1, -4, 1};
        column(right_outboard, 1) = StaticVector<double, 3UL>{0, 0, 0};   // unused
        column(right_outboard, 2) = StaticVector<double, 3UL>{1, -4, 0};
        column(right_outboard, 3) = StaticVector<double, 3UL>{1, -4, 1};
        column(right_outboard, 4) = StaticVector<double, 3UL>{2, -4, 0.5};
        column(right_outboard, 5) = StaticVector<double, 3UL>{0, 0, 0};   // unused
        
        StaticMatrix<double, 3UL, 4UL> right_bellcrank;
        column(right_bellcrank, 0) = StaticVector<double, 3UL>{1, -2, 2};
        column(right_bellcrank, 1) = StaticVector<double, 3UL>{1, 0, 0};
        column(right_bellcrank, 2) = StaticVector<double, 3UL>{1, -4, 1};
        column(right_bellcrank, 3) = StaticVector<double, 3UL>{1, 0, 2.5};
        
        StaticVector<double, 3UL> right_cp{1, -4.5, -0.5};
        
        DoubleWishbone* right_dw = new DoubleWishbone(right_inboard, right_outboard, right_bellcrank,
                                                       spring_rate, weight, cg, upper, right_cp,
                                                       inclination_angle, toe, tire_radius, tire_width, show_ICs);
        
        return new Axle(left_dw, right_dw, cg);
    }
};

#endif // TEST_AXLE_H

