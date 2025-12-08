#include "test_kin_pc.h"
#include <cmath>

using namespace blaze;

// Test: KinPC initialization
TEST_F(TestKinPC, test_kin_pc_init) {
    Node front_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node front_outboard(StaticVector<double, 3UL>{1, 0, 1});
    Beam front_beam(&front_inboard, &front_outboard);
    
    Node rear_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node rear_outboard(StaticVector<double, 3UL>{-1, 0, 1});
    Beam rear_beam(&rear_inboard, &rear_outboard);
    
    Node cg_node(StaticVector<double, 3UL>{0, 0, 0.5});
    CG cg(&cg_node);
    
    KinPC kin_pc(&front_beam, &rear_beam, &cg);
    
    // Should be initialized
    EXPECT_NE(kin_pc.getTrueKinPC(), nullptr);
    EXPECT_NE(kin_pc.getCGAxisKinPC(), nullptr);
}

// Test: True pitch center position calculation (xz intersection)
TEST_F(TestKinPC, test_kin_pc_true_position) {
    // Front beam: from (0, 0, 0) to (2, 0, 2) - diagonal in xz plane
    Node front_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node front_outboard(StaticVector<double, 3UL>{2, 0, 2});
    Beam front_beam(&front_inboard, &front_outboard);
    
    // Rear beam: from (0, 0, 0) to (-2, 0, 2) - diagonal in xz plane
    Node rear_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node rear_outboard(StaticVector<double, 3UL>{-2, 0, 2});
    Beam rear_beam(&rear_inboard, &rear_outboard);
    
    Node cg_node(StaticVector<double, 3UL>{0, 0, 1});
    CG cg(&cg_node);
    
    KinPC kin_pc(&front_beam, &rear_beam, &cg);
    
    // Both beams start at origin and go in opposite x directions
    // They should intersect at (0, 0, 0) in xz plane
    StaticVector<double, 3UL> true_pos = kin_pc.true_KinPC_pos();
    expectVectorNear(true_pos, StaticVector<double, 3UL>{0, 0, 0}, 1e-5);
}

// Test: CG axis pitch center position projection
TEST_F(TestKinPC, test_kin_pc_cg_axis_position) {
    Node front_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node front_outboard(StaticVector<double, 3UL>{1, 0, 1});
    Beam front_beam(&front_inboard, &front_outboard);
    
    Node rear_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node rear_outboard(StaticVector<double, 3UL>{-1, 0, 1});
    Beam rear_beam(&rear_inboard, &rear_outboard);
    
    // CG at (5, 0, 2) - different x position
    Node cg_node(StaticVector<double, 3UL>{5, 0, 2});
    CG cg(&cg_node);
    
    KinPC kin_pc(&front_beam, &rear_beam, &cg);
    
    // True PC should be at origin
    StaticVector<double, 3UL> true_pos = kin_pc.true_KinPC_pos();
    expectVectorNear(true_pos, StaticVector<double, 3UL>{0, 0, 0}, 1e-5);
    
    // CG-axis PC should project x to CG x position (5), but keep y and z from true PC
    StaticVector<double, 3UL> cg_axis_pos = kin_pc.cg_axis_KinPC_pos();
    expectNear(cg_axis_pos[0], 5.0, 1e-5);  // x should be CG x
    expectNear(cg_axis_pos[1], 0.0, 1e-5);  // y should match true PC y
    expectNear(cg_axis_pos[2], 0.0, 1e-5);  // z should match true PC z
}

// Test: Update method recalculates positions
TEST_F(TestKinPC, test_kin_pc_update) {
    // Use beams with different inboard points so intersection can change
    Node front_inboard(StaticVector<double, 3UL>{1, 0, 0});
    Node front_outboard(StaticVector<double, 3UL>{2, 0, 1});
    Beam front_beam(&front_inboard, &front_outboard);
    
    Node rear_inboard(StaticVector<double, 3UL>{-1, 0, 0});
    Node rear_outboard(StaticVector<double, 3UL>{-2, 0, 1});
    Beam rear_beam(&rear_inboard, &rear_outboard);
    
    Node cg_node(StaticVector<double, 3UL>{0, 0, 1});
    CG cg(&cg_node);
    
    KinPC kin_pc(&front_beam, &rear_beam, &cg);
    
    StaticVector<double, 3UL> initial_pos = kin_pc.getTrueKinPC()->position;
    
    // Move front beam outboard to a different slope - this will change the intersection
    // Original: (1,0,0) to (2,0,1) has slope 1
    // New: (1,0,0) to (3,0,0.5) has slope 0.5/2 = 0.25 (different slope)
    front_outboard.position = StaticVector<double, 3UL>{3, 0, 0.5};
    
    // Update should recalculate from new beam positions
    kin_pc.update();
    
    StaticVector<double, 3UL> updated_pos = kin_pc.getTrueKinPC()->position;
    
    // Position should have changed
    double dist = norm(updated_pos - initial_pos);
    EXPECT_GT(dist, 0.1) << "Pitch center should change when beam positions change";
}

// Test: Translate method
TEST_F(TestKinPC, test_kin_pc_translate) {
    Node front_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node front_outboard(StaticVector<double, 3UL>{1, 0, 1});
    Beam front_beam(&front_inboard, &front_outboard);
    
    Node rear_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node rear_outboard(StaticVector<double, 3UL>{-1, 0, 1});
    Beam rear_beam(&rear_inboard, &rear_outboard);
    
    Node cg_node(StaticVector<double, 3UL>{0, 0, 1});
    CG cg(&cg_node);
    
    KinPC kin_pc(&front_beam, &rear_beam, &cg);
    
    StaticVector<double, 3UL> initial_pos = kin_pc.getTrueKinPC()->position;
    
    // Translate
    StaticVector<double, 3UL> translation{1, 2, 3};
    kin_pc.translate(translation);
    
    StaticVector<double, 3UL> translated_pos = kin_pc.getTrueKinPC()->position;
    expectVectorNear(translated_pos, initial_pos + translation, 1e-5);
}

// Test: Get true KinPC node
TEST_F(TestKinPC, test_kin_pc_get_true_kin_pc) {
    Node front_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node front_outboard(StaticVector<double, 3UL>{1, 0, 1});
    Beam front_beam(&front_inboard, &front_outboard);
    
    Node rear_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node rear_outboard(StaticVector<double, 3UL>{-1, 0, 1});
    Beam rear_beam(&rear_inboard, &rear_outboard);
    
    Node cg_node(StaticVector<double, 3UL>{0, 0, 1});
    CG cg(&cg_node);
    
    KinPC kin_pc(&front_beam, &rear_beam, &cg);
    
    Node* true_pc = kin_pc.getTrueKinPC();
    EXPECT_NE(true_pc, nullptr);
    
    // Position should match true_KinPC_pos()
    StaticVector<double, 3UL> method_pos = kin_pc.true_KinPC_pos();
    expectVectorNear(true_pc->position, method_pos, 1e-6);
}

// Test: Get CG-axis KinPC node
TEST_F(TestKinPC, test_kin_pc_get_cg_axis_kin_pc) {
    Node front_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node front_outboard(StaticVector<double, 3UL>{1, 0, 1});
    Beam front_beam(&front_inboard, &front_outboard);
    
    Node rear_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node rear_outboard(StaticVector<double, 3UL>{-1, 0, 1});
    Beam rear_beam(&rear_inboard, &rear_outboard);
    
    Node cg_node(StaticVector<double, 3UL>{5, 0, 1});
    CG cg(&cg_node);
    
    KinPC kin_pc(&front_beam, &rear_beam, &cg);
    
    Node* cg_axis_pc = kin_pc.getCGAxisKinPC();
    EXPECT_NE(cg_axis_pc, nullptr);
    
    // Position should match cg_axis_KinPC_pos()
    StaticVector<double, 3UL> method_pos = kin_pc.cg_axis_KinPC_pos();
    expectVectorNear(cg_axis_pc->position, method_pos, 1e-6);
}

// Test: Pitch center with different beam configurations
TEST_F(TestKinPC, test_kin_pc_different_configurations) {
    // Test with beams that don't intersect at origin
    Node front_inboard(StaticVector<double, 3UL>{1, 0, 0});
    Node front_outboard(StaticVector<double, 3UL>{2, 0, 1});
    Beam front_beam(&front_inboard, &front_outboard);
    
    Node rear_inboard(StaticVector<double, 3UL>{-1, 0, 0});
    Node rear_outboard(StaticVector<double, 3UL>{-2, 0, 1});
    Beam rear_beam(&rear_inboard, &rear_outboard);
    
    Node cg_node(StaticVector<double, 3UL>{0, 0, 0.5});
    CG cg(&cg_node);
    
    KinPC kin_pc(&front_beam, &rear_beam, &cg);
    
    // Should still calculate intersection
    StaticVector<double, 3UL> true_pos = kin_pc.true_KinPC_pos();
    // Both beams have slope 1 in xz plane, should intersect somewhere
    EXPECT_FALSE(std::isinf(true_pos[0]) && std::isinf(true_pos[2]));
}

