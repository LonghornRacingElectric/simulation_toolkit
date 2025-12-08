#include "test_stabar.h"
#include <cmath>

using namespace blaze;

// Test: Stabar initialization
// Python equivalent: test_stabar_init
TEST_F(TestStabar, test_stabar_init) {
    Node left_arm_end(StaticVector<double, 3UL>{1, 2, 0});
    Node right_arm_end(StaticVector<double, 3UL>{1, -2, 0});
    Node left_droplink_end(StaticVector<double, 3UL>{1, 2, -0.5});
    Node right_droplink_end(StaticVector<double, 3UL>{1, -2, -0.5});
    Node bar_left_end(StaticVector<double, 3UL>{0, 2, 0});
    Node bar_right_end(StaticVector<double, 3UL>{0, -2, 0});
    double torsional_stiffness = 1.0;
    
    Stabar stabar(&left_arm_end, &right_arm_end, &left_droplink_end, 
                  &right_droplink_end, &bar_left_end, &bar_right_end, 
                  torsional_stiffness);
    
    // Check that components are created
    EXPECT_NE(stabar.getBar(), nullptr);
    EXPECT_NE(stabar.getLeftArm(), nullptr);
    EXPECT_NE(stabar.getRightArm(), nullptr);
    EXPECT_NE(stabar.getLeftDroplink(), nullptr);
    EXPECT_NE(stabar.getRightDroplink(), nullptr);
    
    // Initial rotations should be zero
    expectNear(stabar.getLeftRotation(), 0.0, 1e-5);
    expectNear(stabar.getRightRotation(), 0.0, 1e-5);
    
    // Add stabar as listener and translate
    left_droplink_end.add_listener(&stabar);
    right_droplink_end.add_listener(&stabar);
    
    double sqrt3_half = 0.5 * std::sqrt(3.0);
    double translation_x = -1.0 * (1.0 - sqrt3_half);
    StaticVector<double, 3UL> translation{translation_x, 0, 0.5};
    left_droplink_end.translate(translation);
    
    // Check position after translation
    expectNear(left_droplink_end.position[0], sqrt3_half, 1e-5);
}

// Test: Stabar residual equation
// Python equivalent: test_stabar_residual_eqn
TEST_F(TestStabar, test_stabar_residual_eqn) {
    Node left_arm_end(StaticVector<double, 3UL>{1, 2, 0});
    Node right_arm_end(StaticVector<double, 3UL>{1, -2, 0});
    Node left_droplink_end(StaticVector<double, 3UL>{1, 2, -0.5});
    Node right_droplink_end(StaticVector<double, 3UL>{1, -2, -0.5});
    Node bar_left_end(StaticVector<double, 3UL>{0, 2, 0});
    Node bar_right_end(StaticVector<double, 3UL>{0, -2, 0});
    double torsional_stiffness = 1.0;
    
    Stabar stabar(&left_arm_end, &right_arm_end, &left_droplink_end, 
                  &right_droplink_end, &bar_left_end, &bar_right_end, 
                  torsional_stiffness);
    
    left_droplink_end.add_listener(&stabar);
    right_droplink_end.add_listener(&stabar);
    
    // Test residual equation with pi/6 rotation
    // Note: _droplink_eqn modifies the node position, so we need to access it correctly
    double rotation = M_PI / 6.0;
    double residual = stabar._droplink_eqn(rotation, stabar.getLeftDroplink());
    (void)residual;  // Suppress unused variable warning
    
    // Check position after rotation
    double sqrt3_half = 0.5 * std::sqrt(3.0);
    StaticVector<double, 3UL> expected_pos{sqrt3_half, 2.0, 0.5};
    expectVectorNear(stabar.getLeftDroplink()->getOutboardNode()->position, 
                     expected_pos, 1e-5);
}

// Test: Stabar residual equation length preservation
// Python equivalent: test_stabar_residual_eqn_length
TEST_F(TestStabar, test_stabar_residual_eqn_length) {
    Node left_arm_end(StaticVector<double, 3UL>{1, 2, 0});
    Node right_arm_end(StaticVector<double, 3UL>{1, -2, 0});
    Node left_droplink_end(StaticVector<double, 3UL>{1, 2, -0.5});
    Node right_droplink_end(StaticVector<double, 3UL>{1, -2, -0.5});
    Node bar_left_end(StaticVector<double, 3UL>{0, 2, 0});
    Node bar_right_end(StaticVector<double, 3UL>{0, -2, 0});
    double torsional_stiffness = 1.0;
    
    Stabar stabar(&left_arm_end, &right_arm_end, &left_droplink_end, 
                  &right_droplink_end, &bar_left_end, &bar_right_end, 
                  torsional_stiffness);
    
    left_droplink_end.add_listener(&stabar);
    right_droplink_end.add_listener(&stabar);
    
    double rotation = M_PI / 6.0;
    stabar._droplink_eqn(rotation, stabar.getLeftDroplink());
    
    // Adjust droplink end position
    double sqrt3_half = 0.5 * std::sqrt(3.0);
    left_droplink_end.position[0] += -1.0 * (1.0 - sqrt3_half);
    left_droplink_end.position[1] += 0.0;
    left_droplink_end.position[2] += 0.5;
    
    // Length should match initial length
    expectNear(stabar.getLeftDroplink()->length(), 
               stabar.getLeftDroplink()->initial_length(), 1e-5);
}

// Test: Stabar update - left rotation
// Python equivalent: test_stabar_update_one
TEST_F(TestStabar, test_stabar_update_one) {
    Node left_arm_end(StaticVector<double, 3UL>{1, 2, 0});
    Node right_arm_end(StaticVector<double, 3UL>{1, -2, 0});
    Node left_droplink_end(StaticVector<double, 3UL>{1, 2, -0.5});
    Node right_droplink_end(StaticVector<double, 3UL>{1, -2, -0.5});
    Node bar_left_end(StaticVector<double, 3UL>{0, 2, 0});
    Node bar_right_end(StaticVector<double, 3UL>{0, -2, 0});
    double torsional_stiffness = 1.0;
    
    Stabar stabar(&left_arm_end, &right_arm_end, &left_droplink_end, 
                  &right_droplink_end, &bar_left_end, &bar_right_end, 
                  torsional_stiffness);
    
    left_droplink_end.add_listener(&stabar);
    right_droplink_end.add_listener(&stabar);
    
    double sqrt3_half = 0.5 * std::sqrt(3.0);
    double translation_x = -1.0 * (1.0 - sqrt3_half);
    StaticVector<double, 3UL> translation{translation_x, 0, 0.5};
    left_droplink_end.translate(translation);
    
    // Left rotation should be pi/6
    expectNear(stabar.getLeftRotation(), M_PI / 6.0, 1e-5);
}

// Test: Stabar update - right rotation stays zero
// Python equivalent: test_stabar_update_two
TEST_F(TestStabar, test_stabar_update_two) {
    Node left_arm_end(StaticVector<double, 3UL>{1, 2, 0});
    Node right_arm_end(StaticVector<double, 3UL>{1, -2, 0});
    Node left_droplink_end(StaticVector<double, 3UL>{1, 2, -0.5});
    Node right_droplink_end(StaticVector<double, 3UL>{1, -2, -0.5});
    Node bar_left_end(StaticVector<double, 3UL>{0, 2, 0});
    Node bar_right_end(StaticVector<double, 3UL>{0, -2, 0});
    double torsional_stiffness = 1.0;
    
    Stabar stabar(&left_arm_end, &right_arm_end, &left_droplink_end, 
                  &right_droplink_end, &bar_left_end, &bar_right_end, 
                  torsional_stiffness);
    
    left_droplink_end.add_listener(&stabar);
    right_droplink_end.add_listener(&stabar);
    
    double sqrt3_half = 0.5 * std::sqrt(3.0);
    double translation_x = -1.0 * (1.0 - sqrt3_half);
    StaticVector<double, 3UL> translation{translation_x, 0, 0.5};
    left_droplink_end.translate(translation);
    
    // Right rotation should remain zero
    expectNear(stabar.getRightRotation(), 0.0, 1e-5);
}

// Test: Stabar update - right rotation
// Python equivalent: test_stabar_update_three
TEST_F(TestStabar, test_stabar_update_three) {
    Node left_arm_end(StaticVector<double, 3UL>{1, 2, 0});
    Node right_arm_end(StaticVector<double, 3UL>{1, -2, 0});
    Node left_droplink_end(StaticVector<double, 3UL>{1, 2, -0.5});
    Node right_droplink_end(StaticVector<double, 3UL>{1, -2, -0.5});
    Node bar_left_end(StaticVector<double, 3UL>{0, 2, 0});
    Node bar_right_end(StaticVector<double, 3UL>{0, -2, 0});
    double torsional_stiffness = 1.0;
    
    Stabar stabar(&left_arm_end, &right_arm_end, &left_droplink_end, 
                  &right_droplink_end, &bar_left_end, &bar_right_end, 
                  torsional_stiffness);
    
    left_droplink_end.add_listener(&stabar);
    right_droplink_end.add_listener(&stabar);
    
    double sqrt3_half = 0.5 * std::sqrt(3.0);
    double translation_x = -1.0 * (1.0 - sqrt3_half);
    StaticVector<double, 3UL> translation{translation_x, 0, 0.5};
    right_droplink_end.translate(translation);
    
    // Right rotation should be pi/6
    expectNear(stabar.getRightRotation(), M_PI / 6.0, 1e-5);
}

// Test: Stabar rotation property
// Python equivalent: test_stabar_update_rotation_one
TEST_F(TestStabar, test_stabar_update_rotation_one) {
    Node left_arm_end(StaticVector<double, 3UL>{1, 2, 0});
    Node right_arm_end(StaticVector<double, 3UL>{1, -2, 0});
    Node left_droplink_end(StaticVector<double, 3UL>{1, 2, -0.5});
    Node right_droplink_end(StaticVector<double, 3UL>{1, -2, -0.5});
    Node bar_left_end(StaticVector<double, 3UL>{0, 2, 0});
    Node bar_right_end(StaticVector<double, 3UL>{0, -2, 0});
    double torsional_stiffness = 1.0;
    
    Stabar stabar(&left_arm_end, &right_arm_end, &left_droplink_end, 
                  &right_droplink_end, &bar_left_end, &bar_right_end, 
                  torsional_stiffness);
    
    left_droplink_end.add_listener(&stabar);
    right_droplink_end.add_listener(&stabar);
    
    double sqrt3_half = 0.5 * std::sqrt(3.0);
    double translation_x = -1.0 * (1.0 - sqrt3_half);
    StaticVector<double, 3UL> translation1{translation_x, 0, 0.5};
    StaticVector<double, 3UL> translation2{translation_x, 0, -0.5};
    left_droplink_end.translate(translation1);
    right_droplink_end.translate(translation2);
    
    // Rotation should be pi/3 (pi/6 - (-pi/6))
    expectNear(stabar.rotation(), M_PI / 3.0, 1e-5);
}

// Test: Stabar torque calculation
// Python equivalent: test_stabar_torque_one
TEST_F(TestStabar, test_stabar_torque_one) {
    Node left_arm_end(StaticVector<double, 3UL>{1, 2, 0});
    Node right_arm_end(StaticVector<double, 3UL>{1, -2, 0});
    Node left_droplink_end(StaticVector<double, 3UL>{1, 2, -0.5});
    Node right_droplink_end(StaticVector<double, 3UL>{1, -2, -0.5});
    Node bar_left_end(StaticVector<double, 3UL>{0, 2, 0});
    Node bar_right_end(StaticVector<double, 3UL>{0, -2, 0});
    double torsional_stiffness = 1.0;
    
    Stabar stabar(&left_arm_end, &right_arm_end, &left_droplink_end, 
                  &right_droplink_end, &bar_left_end, &bar_right_end, 
                  torsional_stiffness);
    
    left_droplink_end.add_listener(&stabar);
    right_droplink_end.add_listener(&stabar);
    
    double sqrt3_half = 0.5 * std::sqrt(3.0);
    double translation_x = -1.0 * (1.0 - sqrt3_half);
    StaticVector<double, 3UL> translation{translation_x, 0, 0.5};
    left_droplink_end.translate(translation);
    right_droplink_end.translate(translation);
    
    // Torque should be zero (no relative rotation)
    expectNear(stabar.torque(), 0.0, 1e-5);
}

// Test: Stabar torque calculation with rotation
// Python equivalent: test_stabar_torque_two
TEST_F(TestStabar, test_stabar_torque_two) {
    Node left_arm_end(StaticVector<double, 3UL>{1, 2, 0});
    Node right_arm_end(StaticVector<double, 3UL>{1, -2, 0});
    Node left_droplink_end(StaticVector<double, 3UL>{1, 2, -0.5});
    Node right_droplink_end(StaticVector<double, 3UL>{1, -2, -0.5});
    Node bar_left_end(StaticVector<double, 3UL>{0, 2, 0});
    Node bar_right_end(StaticVector<double, 3UL>{0, -2, 0});
    double torsional_stiffness = 1.0;
    
    Stabar stabar(&left_arm_end, &right_arm_end, &left_droplink_end, 
                  &right_droplink_end, &bar_left_end, &bar_right_end, 
                  torsional_stiffness);
    
    left_droplink_end.add_listener(&stabar);
    right_droplink_end.add_listener(&stabar);
    
    double sqrt3_half = 0.5 * std::sqrt(3.0);
    double translation_x = -1.0 * (1.0 - sqrt3_half);
    StaticVector<double, 3UL> translation1{translation_x, 0, 0.5};
    StaticVector<double, 3UL> translation2{translation_x, 0, -0.5};
    left_droplink_end.translate(translation1);
    right_droplink_end.translate(translation2);
    
    // Torque should be torsional_stiffness * rotation = 1 * pi/3
    expectNear(stabar.torque(), M_PI / 3.0, 1e-5);
}

