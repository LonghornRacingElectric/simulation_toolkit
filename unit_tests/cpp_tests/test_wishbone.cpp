#include "test_wishbone.h"
#include <cmath>

using namespace blaze;

// Test: Wishbone base direction
TEST_F(TestWishbone, test_wishbone_base_direction) {
    Node inboard_fore(StaticVector<double, 3UL>{2, 0, 0});
    Node inboard_aft(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{1, 2, 0});
    
    Beam fore_link(&inboard_fore, &outboard);
    Beam aft_link(&inboard_aft, &outboard);
    
    Wishbone wishbone(&fore_link, &aft_link);
    
    StaticVector<double, 3UL> expected = {1, 0, 0};
    expectVectorEqual(wishbone.getDirection(), expected);
}

// Test: Wishbone base angle
TEST_F(TestWishbone, test_wishbone_base_angle) {
    Node inboard_fore(StaticVector<double, 3UL>{2, 0, 0});
    Node inboard_aft(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{1, 2, 0});
    
    Beam fore_link(&inboard_fore, &outboard);
    Beam aft_link(&inboard_aft, &outboard);
    
    Wishbone wishbone(&fore_link, &aft_link);
    
    EXPECT_DOUBLE_EQ(wishbone.getAngle(), 0.0);
}

// Test: Rotate wishbone
TEST_F(TestWishbone, test_rotate) {
    Node inboard_fore(StaticVector<double, 3UL>{2, 0, 0});
    Node inboard_aft(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{1, 2, 0});
    
    Beam fore_link(&inboard_fore, &outboard);
    Beam aft_link(&inboard_aft, &outboard);
    
    Wishbone wishbone(&fore_link, &aft_link);
    wishbone.rotate(M_PI / 2.0);
    
    StaticVector<double, 3UL> expected = {1, 0, 2};
    StaticVector<double, 3UL> actual = fore_link.getOutboardNode()->position;
    expectVectorNear(actual, expected, 1e-6);
}

// Test: Child node rotation
TEST_F(TestWishbone, test_child_rotate) {
    Node inboard_fore(StaticVector<double, 3UL>{2, 0, 0});
    Node inboard_aft(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{1, 2, 0});
    
    Beam fore_link(&inboard_fore, &outboard);
    Beam aft_link(&inboard_aft, &outboard);
    
    Wishbone wishbone(&fore_link, &aft_link);
    
    Node child_node(StaticVector<double, 3UL>{1, 1, 0});
    outboard.add_child(&child_node);
    
    wishbone.rotate(M_PI / 2.0);
    
    StaticVector<double, 3UL> expected = {1, 0, 1};
    StaticVector<double, 3UL> actual = child_node.position;
    expectVectorNear(actual, expected, 1e-6);
}

// Test: Plane calculation (XY plane)
TEST_F(TestWishbone, test_plane_xy) {
    Node inboard_fore(StaticVector<double, 3UL>{2, 0, 0});
    Node inboard_aft(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{1, 2, 0});
    
    Beam fore_link(&inboard_fore, &outboard);
    Beam aft_link(&inboard_aft, &outboard);
    
    Wishbone wishbone(&fore_link, &aft_link);
    StaticVector<double, 6UL> plane_params = wishbone.plane();
    
    double a = plane_params[0];
    double b = plane_params[1];
    double c = plane_params[2];
    double x_0 = plane_params[3];
    double y_0 = plane_params[4];
    double z_0 = plane_params[5];
    
    // Check if planes are equivalent - test points should satisfy plane equation
    double point_1 = a * (1 - x_0) + b * (0 - y_0) + c * (0 - z_0);
    double point_2 = a * (0 - x_0) + b * (1 - y_0) + c * (0 - z_0);
    double point_3 = a * (0 - x_0) + b * (0 - y_0) + c * (0 - z_0);
    
    EXPECT_NEAR(point_1, 0.0, 1e-6);
    EXPECT_NEAR(point_2, 0.0, 1e-6);
    EXPECT_NEAR(point_3, 0.0, 1e-6);
}

// Test: Direction vector
TEST_F(TestWishbone, test_direction_vec) {
    Node inboard_fore(StaticVector<double, 3UL>{2, 0, 0});
    Node inboard_aft(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{1, 2, 0});
    
    Beam fore_link(&inboard_fore, &outboard);
    Beam aft_link(&inboard_aft, &outboard);
    
    Wishbone wishbone(&fore_link, &aft_link);
    
    StaticVector<double, 3UL> expected = {1, 0, 0};
    StaticVector<double, 3UL> actual = wishbone.direction_vec();
    expectVectorEqual(actual, expected);
}

