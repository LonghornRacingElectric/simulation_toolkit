#include "test_kin_rc.h"
#include <cmath>

using namespace blaze;

// Test: KinRC initialization
TEST_F(TestKinRC, test_kin_rc_init) {
    Node left_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node left_outboard(StaticVector<double, 3UL>{0, 1, 1});
    Beam left_beam(&left_inboard, &left_outboard);
    
    Node right_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node right_outboard(StaticVector<double, 3UL>{0, -1, 1});
    Beam right_beam(&right_inboard, &right_outboard);
    
    Node cg_node(StaticVector<double, 3UL>{0, 0, 0.5});
    CG cg(&cg_node);
    
    KinRC kin_rc(&left_beam, &right_beam, &cg);
    
    // Should be initialized
    EXPECT_NE(kin_rc.getTrueKinRC(), nullptr);
    EXPECT_NE(kin_rc.getCGAxisKinRC(), nullptr);
}

// Test: True roll center position calculation (yz intersection)
TEST_F(TestKinRC, test_kin_rc_true_position) {
    // Left beam: from (0, 0, 0) to (0, 2, 2) - diagonal in yz plane
    Node left_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node left_outboard(StaticVector<double, 3UL>{0, 2, 2});
    Beam left_beam(&left_inboard, &left_outboard);
    
    // Right beam: from (0, 0, 0) to (0, -2, 2) - diagonal in yz plane
    Node right_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node right_outboard(StaticVector<double, 3UL>{0, -2, 2});
    Beam right_beam(&right_inboard, &right_outboard);
    
    Node cg_node(StaticVector<double, 3UL>{0, 0, 1});
    CG cg(&cg_node);
    
    KinRC kin_rc(&left_beam, &right_beam, &cg);
    
    // Both beams start at origin and go in opposite y directions
    // They should intersect at (0, 0, 0) in yz plane
    StaticVector<double, 3UL> true_pos = kin_rc.true_kinRC_pos();
    expectVectorNear(true_pos, StaticVector<double, 3UL>{0, 0, 0}, 1e-5);
}

// Test: CG axis roll center position projection
TEST_F(TestKinRC, test_kin_rc_cg_axis_position) {
    Node left_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node left_outboard(StaticVector<double, 3UL>{0, 1, 1});
    Beam left_beam(&left_inboard, &left_outboard);
    
    Node right_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node right_outboard(StaticVector<double, 3UL>{0, -1, 1});
    Beam right_beam(&right_inboard, &right_outboard);
    
    // CG at (0, 5, 2) - different y position
    Node cg_node(StaticVector<double, 3UL>{0, 5, 2});
    CG cg(&cg_node);
    
    KinRC kin_rc(&left_beam, &right_beam, &cg);
    
    // True RC should be at origin
    StaticVector<double, 3UL> true_pos = kin_rc.true_kinRC_pos();
    expectVectorNear(true_pos, StaticVector<double, 3UL>{0, 0, 0}, 1e-5);
    
    // CG-axis RC should project y to CG y position (5), but keep x and z from true RC
    StaticVector<double, 3UL> cg_axis_pos = kin_rc.cg_axis_KinRC_pos();
    expectNear(cg_axis_pos[0], 0.0, 1e-5);  // x should match true RC x
    expectNear(cg_axis_pos[1], 5.0, 1e-5);  // y should be CG y
    expectNear(cg_axis_pos[2], 0.0, 1e-5);  // z should match true RC z
}

// Test: Update method recalculates positions
TEST_F(TestKinRC, test_kin_rc_update) {
    // Use beams with different inboard points so intersection can change
    Node left_inboard(StaticVector<double, 3UL>{0, 1, 0});
    Node left_outboard(StaticVector<double, 3UL>{0, 2, 1});
    Beam left_beam(&left_inboard, &left_outboard);
    
    Node right_inboard(StaticVector<double, 3UL>{0, -1, 0});
    Node right_outboard(StaticVector<double, 3UL>{0, -2, 1});
    Beam right_beam(&right_inboard, &right_outboard);
    
    Node cg_node(StaticVector<double, 3UL>{0, 0, 1});
    CG cg(&cg_node);
    
    KinRC kin_rc(&left_beam, &right_beam, &cg);
    
    StaticVector<double, 3UL> initial_pos = kin_rc.getTrueKinRC()->position;
    
    // Move left beam outboard to a different slope - this will change the intersection
    // Original: (0,1,0) to (0,2,1) has slope 1
    // New: (0,1,0) to (0,3,0.5) has slope 0.5/2 = 0.25 (different slope)
    left_outboard.position = StaticVector<double, 3UL>{0, 3, 0.5};
    
    // Update should recalculate from new beam positions
    kin_rc.update();
    
    StaticVector<double, 3UL> updated_pos = kin_rc.getTrueKinRC()->position;
    
    // Position should have changed
    double dist = norm(updated_pos - initial_pos);
    EXPECT_GT(dist, 0.1) << "Roll center should change when beam positions change";
}

// Test: Translate method
TEST_F(TestKinRC, test_kin_rc_translate) {
    Node left_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node left_outboard(StaticVector<double, 3UL>{0, 1, 1});
    Beam left_beam(&left_inboard, &left_outboard);
    
    Node right_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node right_outboard(StaticVector<double, 3UL>{0, -1, 1});
    Beam right_beam(&right_inboard, &right_outboard);
    
    Node cg_node(StaticVector<double, 3UL>{0, 0, 1});
    CG cg(&cg_node);
    
    KinRC kin_rc(&left_beam, &right_beam, &cg);
    
    StaticVector<double, 3UL> initial_pos = kin_rc.getTrueKinRC()->position;
    
    // Translate
    StaticVector<double, 3UL> translation{1, 2, 3};
    kin_rc.translate(translation);
    
    StaticVector<double, 3UL> translated_pos = kin_rc.getTrueKinRC()->position;
    expectVectorNear(translated_pos, initial_pos + translation, 1e-5);
}

// Test: Get true KinRC node
TEST_F(TestKinRC, test_kin_rc_get_true_kin_rc) {
    Node left_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node left_outboard(StaticVector<double, 3UL>{0, 1, 1});
    Beam left_beam(&left_inboard, &left_outboard);
    
    Node right_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node right_outboard(StaticVector<double, 3UL>{0, -1, 1});
    Beam right_beam(&right_inboard, &right_outboard);
    
    Node cg_node(StaticVector<double, 3UL>{0, 0, 1});
    CG cg(&cg_node);
    
    KinRC kin_rc(&left_beam, &right_beam, &cg);
    
    Node* true_rc = kin_rc.getTrueKinRC();
    EXPECT_NE(true_rc, nullptr);
    
    // Position should match true_kinRC_pos()
    StaticVector<double, 3UL> method_pos = kin_rc.true_kinRC_pos();
    expectVectorNear(true_rc->position, method_pos, 1e-6);
}

// Test: Get CG-axis KinRC node
TEST_F(TestKinRC, test_kin_rc_get_cg_axis_kin_rc) {
    Node left_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node left_outboard(StaticVector<double, 3UL>{0, 1, 1});
    Beam left_beam(&left_inboard, &left_outboard);
    
    Node right_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node right_outboard(StaticVector<double, 3UL>{0, -1, 1});
    Beam right_beam(&right_inboard, &right_outboard);
    
    Node cg_node(StaticVector<double, 3UL>{0, 5, 1});
    CG cg(&cg_node);
    
    KinRC kin_rc(&left_beam, &right_beam, &cg);
    
    Node* cg_axis_rc = kin_rc.getCGAxisKinRC();
    EXPECT_NE(cg_axis_rc, nullptr);
    
    // Position should match cg_axis_KinRC_pos()
    StaticVector<double, 3UL> method_pos = kin_rc.cg_axis_KinRC_pos();
    expectVectorNear(cg_axis_rc->position, method_pos, 1e-6);
}

// Test: Roll center with different beam configurations
TEST_F(TestKinRC, test_kin_rc_different_configurations) {
    // Test with beams that don't intersect at origin
    Node left_inboard(StaticVector<double, 3UL>{0, 1, 0});
    Node left_outboard(StaticVector<double, 3UL>{0, 2, 1});
    Beam left_beam(&left_inboard, &left_outboard);
    
    Node right_inboard(StaticVector<double, 3UL>{0, -1, 0});
    Node right_outboard(StaticVector<double, 3UL>{0, -2, 1});
    Beam right_beam(&right_inboard, &right_outboard);
    
    Node cg_node(StaticVector<double, 3UL>{0, 0, 0.5});
    CG cg(&cg_node);
    
    KinRC kin_rc(&left_beam, &right_beam, &cg);
    
    // Should still calculate intersection
    StaticVector<double, 3UL> true_pos = kin_rc.true_kinRC_pos();
    // Both beams have slope 1 in yz plane, should intersect somewhere
    EXPECT_FALSE(std::isinf(true_pos[1]) && std::isinf(true_pos[2]));
}

