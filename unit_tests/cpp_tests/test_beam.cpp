#include "test_beam.h"
#include <cmath>
#include <limits>

using namespace blaze;

// Test: Link initialization with direction value
TEST_F(TestBeam, test_link_init_direction_value) {
    Node inboard(StaticVector<double, 3UL>{-1, 0, 0});
    Node outboard(StaticVector<double, 3UL>{1, 0, 0});
    Beam test_link(&inboard, &outboard);
    
    StaticVector<double, 3UL> expected = {1, 0, 0};
    expectVectorEqual(test_link.direction(), expected);
}

// Test: Link initialization with center value
TEST_F(TestBeam, test_link_init_center_value) {
    Node inboard(StaticVector<double, 3UL>{-3, 0, 0});
    Node outboard(StaticVector<double, 3UL>{1, 0, 0});
    Beam test_link(&inboard, &outboard);
    
    StaticVector<double, 3UL> expected = {-1, 0, 0};
    expectVectorEqual(test_link.center(), expected);
}

// Test: Link initialization with length value
TEST_F(TestBeam, test_link_init_length_value) {
    Node inboard(StaticVector<double, 3UL>{-3, 0, 0});
    Node outboard(StaticVector<double, 3UL>{1, 0, 0});
    Beam test_link(&inboard, &outboard);
    
    EXPECT_NEAR(test_link.length(), 4.0, 1e-6);
}

// Test: Link yz intersection base case
TEST_F(TestBeam, test_link_yz_intersection_base) {
    Node inboard_1(StaticVector<double, 3UL>{0, 1, 1});
    Node outboard_1(StaticVector<double, 3UL>{0, 2, 2});
    Beam test_link_1(&inboard_1, &outboard_1);
    
    Node inboard_2(StaticVector<double, 3UL>{0, 0, 2});
    Node outboard_2(StaticVector<double, 3UL>{0, -1, 3});
    Beam test_link_2(&inboard_2, &outboard_2);
    
    Node intersection = test_link_1.yz_intersection(test_link_2);
    StaticVector<double, 3UL> expected = {0, 1, 1};
    expectVectorNear(intersection.position, expected);
}

// Test: Link yz intersection edge case one (parallel lines)
TEST_F(TestBeam, test_link_yz_intersection_edge_one) {
    Node inboard_1(StaticVector<double, 3UL>{0, 1, 1});
    Node outboard_1(StaticVector<double, 3UL>{0, 2, 1});
    Beam test_link_1(&inboard_1, &outboard_1);
    
    Node inboard_2(StaticVector<double, 3UL>{0, 1, 0});
    Node outboard_2(StaticVector<double, 3UL>{0, 2, 0});
    Beam test_link_2(&inboard_2, &outboard_2);
    
    Node intersection = test_link_1.yz_intersection(test_link_2);
    
    EXPECT_TRUE(isInfinity(intersection.position[1]));
    EXPECT_NEAR(intersection.position[2], 0.5, 1e-6);
    EXPECT_NEAR(intersection.position[0], 0.0, 1e-6);
}

// Test: Link yz intersection edge case two (identical lines)
TEST_F(TestBeam, test_link_yz_intersection_edge_two) {
    Node inboard_1(StaticVector<double, 3UL>{0, 1, 1});
    Node outboard_1(StaticVector<double, 3UL>{0, 2, 1});
    Beam test_link_1(&inboard_1, &outboard_1);
    
    Node inboard_2(StaticVector<double, 3UL>{0, 1, 1});
    Node outboard_2(StaticVector<double, 3UL>{0, 2, 1});
    Beam test_link_2(&inboard_2, &outboard_2);
    
    Node intersection = test_link_1.yz_intersection(test_link_2);
    
    EXPECT_TRUE(isInfinity(intersection.position[1]));
    EXPECT_NEAR(intersection.position[2], 1.0, 1e-6);
    EXPECT_NEAR(intersection.position[0], 0.0, 1e-6);
}

// Test: Link xz intersection base case
TEST_F(TestBeam, test_link_xz_intersection) {
    Node inboard_1(StaticVector<double, 3UL>{1, 0, 1});
    Node outboard_1(StaticVector<double, 3UL>{2, 0, 2});
    Beam test_link_1(&inboard_1, &outboard_1);
    
    Node inboard_2(StaticVector<double, 3UL>{0, 0, 2});
    Node outboard_2(StaticVector<double, 3UL>{-1, 0, 3});
    Beam test_link_2(&inboard_2, &outboard_2);
    
    Node intersection = test_link_1.xz_intersection(test_link_2);
    StaticVector<double, 3UL> expected = {1, 0, 1};
    expectVectorNear(intersection.position, expected);
}

// Test: Link xz intersection edge case one (parallel lines)
TEST_F(TestBeam, test_link_xz_intersection_edge_one) {
    Node inboard_1(StaticVector<double, 3UL>{1, 0, 1});
    Node outboard_1(StaticVector<double, 3UL>{2, 0, 1});
    Beam test_link_1(&inboard_1, &outboard_1);
    
    Node inboard_2(StaticVector<double, 3UL>{1, 0, 0});
    Node outboard_2(StaticVector<double, 3UL>{2, 0, 0});
    Beam test_link_2(&inboard_2, &outboard_2);
    
    Node intersection = test_link_1.xz_intersection(test_link_2);
    
    EXPECT_TRUE(isInfinity(intersection.position[0]));
    EXPECT_NEAR(intersection.position[2], 0.5, 1e-6);
    EXPECT_NEAR(intersection.position[1], 0.0, 1e-6);
}

// Test: Link xz intersection edge case two (identical lines)
TEST_F(TestBeam, test_link_xz_intersection_edge_two) {
    Node inboard_1(StaticVector<double, 3UL>{1, 0, 1});
    Node outboard_1(StaticVector<double, 3UL>{2, 0, 1});
    Beam test_link_1(&inboard_1, &outboard_1);
    
    Node inboard_2(StaticVector<double, 3UL>{1, 0, 1});
    Node outboard_2(StaticVector<double, 3UL>{2, 0, 1});
    Beam test_link_2(&inboard_2, &outboard_2);
    
    Node intersection = test_link_1.xz_intersection(test_link_2);
    
    EXPECT_TRUE(isInfinity(intersection.position[0]));
    EXPECT_NEAR(intersection.position[2], 1.0, 1e-6);
    EXPECT_NEAR(intersection.position[1], 0.0, 1e-6);
}

// Test: Link centered coordinates
TEST_F(TestBeam, test_link_centered_coords) {
    Node lower_link_node(StaticVector<double, 3UL>{1, 1, 0});
    Node upper_link_node(StaticVector<double, 3UL>{2, 2, 1});
    Node ref_node(StaticVector<double, 3UL>{2, 2, 1});
    
    Beam test_link(&lower_link_node, &upper_link_node);
    StaticVector<double, 3UL> test_coord_output = test_link.link_centered_coords(ref_node);
    
    double expected_z = std::sqrt(3.0);
    StaticVector<double, 3UL> expected = {0, 0, expected_z};
    expectVectorNear(test_coord_output, expected, 1e-3);
}

// Note: main() is defined in test_node.cpp to avoid multiple definitions
