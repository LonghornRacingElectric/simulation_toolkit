#include "test_cg.h"
#include <cmath>

using namespace blaze;

// Test: CG initialization
TEST_F(TestCG, test_cg_init) {
    Node cg_node(StaticVector<double, 3UL>{1.0, 2.0, 3.0});
    CG cg(&cg_node);
    
    // CG should be initialized
    EXPECT_NE(cg.getNode(), nullptr);
    EXPECT_EQ(cg.getNode(), &cg_node);
}

// Test: CG getNode returns correct node
TEST_F(TestCG, test_cg_get_node) {
    Node cg_node(StaticVector<double, 3UL>{1.5, 2.5, 3.5});
    CG cg(&cg_node);
    
    EXPECT_EQ(cg.getNode(), &cg_node);
    EXPECT_EQ(cg.getCGNode(), &cg_node);  // Alias method
}

// Test: CG getPosition returns correct position
TEST_F(TestCG, test_cg_get_position) {
    StaticVector<double, 3UL> expected_pos{1.0, 2.0, 3.0};
    Node cg_node(expected_pos);
    CG cg(&cg_node);
    
    const StaticVector<double, 3UL>& position = cg.getPosition();
    expectVectorEqual(position, expected_pos);
}

// Test: CG position matches node position
TEST_F(TestCG, test_cg_position_matches_node) {
    StaticVector<double, 3UL> pos{2.5, 3.5, 4.5};
    Node cg_node(pos);
    CG cg(&cg_node);
    
    const StaticVector<double, 3UL>& cg_position = cg.getPosition();
    expectVectorEqual(cg_position, cg_node.position);
}

// Test: CG position changes when node position changes
TEST_F(TestCG, test_cg_position_updates) {
    Node cg_node(StaticVector<double, 3UL>{0.0, 0.0, 0.0});
    CG cg(&cg_node);
    
    StaticVector<double, 3UL> initial_pos = cg.getPosition();
    expectVectorEqual(initial_pos, StaticVector<double, 3UL>{0.0, 0.0, 0.0});
    
    // Change node position
    cg_node.position = StaticVector<double, 3UL>{1.0, 2.0, 3.0};
    
    // CG position should reflect the change
    const StaticVector<double, 3UL>& updated_pos = cg.getPosition();
    expectVectorEqual(updated_pos, StaticVector<double, 3UL>{1.0, 2.0, 3.0});
}

// Test: CG with different positions
TEST_F(TestCG, test_cg_different_positions) {
    // Test CG at origin
    Node cg_node1(StaticVector<double, 3UL>{0.0, 0.0, 0.0});
    CG cg1(&cg_node1);
    expectVectorEqual(cg1.getPosition(), StaticVector<double, 3UL>{0.0, 0.0, 0.0});
    
    // Test CG at positive coordinates
    Node cg_node2(StaticVector<double, 3UL>{1.0, 2.0, 3.0});
    CG cg2(&cg_node2);
    expectVectorEqual(cg2.getPosition(), StaticVector<double, 3UL>{1.0, 2.0, 3.0});
    
    // Test CG at negative coordinates
    Node cg_node3(StaticVector<double, 3UL>{-1.0, -2.0, -3.0});
    CG cg3(&cg_node3);
    expectVectorEqual(cg3.getPosition(), StaticVector<double, 3UL>{-1.0, -2.0, -3.0});
}

// Test: CG getCGNode alias method
TEST_F(TestCG, test_cg_get_cg_node) {
    Node cg_node(StaticVector<double, 3UL>{1.0, 2.0, 3.0});
    CG cg(&cg_node);
    
    // Both methods should return the same node
    EXPECT_EQ(cg.getNode(), cg.getCGNode());
    EXPECT_EQ(cg.getCGNode(), &cg_node);
}

// Test: CG with node translation
TEST_F(TestCG, test_cg_node_translation) {
    Node cg_node(StaticVector<double, 3UL>{0.0, 0.0, 0.0});
    CG cg(&cg_node);
    
    // Translate the node
    StaticVector<double, 3UL> translation{1.0, 2.0, 3.0};
    cg_node.translate(translation);
    
    // CG position should reflect the translation
    const StaticVector<double, 3UL>& translated_pos = cg.getPosition();
    expectVectorEqual(translated_pos, StaticVector<double, 3UL>{1.0, 2.0, 3.0});
}

// Test: CG with calculated position (simulating CG calculation from tire positions)
TEST_F(TestCG, test_cg_calculated_position) {
    // Simulate CG calculation similar to Python:
    // cg_x = (FL_tire[0] - RL_tire[0]) * CGBX + RL_tire[0]
    // cg_y = (FL_tire[1] - FR_tire[1]) * CGBY + FR_tire[1]
    // cg_z = CGZ
    
    double FL_tire_x = 1.0, RL_tire_x = 0.0;
    double FL_tire_y = 1.0, FR_tire_y = 0.0;
    double CGBX = 0.6;  // 60% forward
    double CGBY = 0.5;  // 50% left
    double CGZ = 0.5;   // Height
    
    double cg_x = (FL_tire_x - RL_tire_x) * CGBX + RL_tire_x;
    double cg_y = (FL_tire_y - FR_tire_y) * CGBY + FR_tire_y;
    double cg_z = CGZ;
    
    Node cg_node(StaticVector<double, 3UL>{cg_x, cg_y, cg_z});
    CG cg(&cg_node);
    
    StaticVector<double, 3UL> expected_pos{cg_x, cg_y, cg_z};
    expectVectorEqual(cg.getPosition(), expected_pos);
    
    // Verify calculated values
    expectNear(cg.getPosition()[0], 0.6, 1e-6);  // (1.0 - 0.0) * 0.6 + 0.0 = 0.6
    expectNear(cg.getPosition()[1], 0.5, 1e-6);  // (1.0 - 0.0) * 0.5 + 0.0 = 0.5
    expectNear(cg.getPosition()[2], 0.5, 1e-6);  // CGZ = 0.5
}

// Test: Multiple CG instances with different nodes
TEST_F(TestCG, test_cg_multiple_instances) {
    Node cg_node1(StaticVector<double, 3UL>{1.0, 1.0, 1.0});
    Node cg_node2(StaticVector<double, 3UL>{2.0, 2.0, 2.0});
    
    CG cg1(&cg_node1);
    CG cg2(&cg_node2);
    
    // Each CG should have its own node
    EXPECT_EQ(cg1.getNode(), &cg_node1);
    EXPECT_EQ(cg2.getNode(), &cg_node2);
    
    // Positions should be different
    expectVectorEqual(cg1.getPosition(), StaticVector<double, 3UL>{1.0, 1.0, 1.0});
    expectVectorEqual(cg2.getPosition(), StaticVector<double, 3UL>{2.0, 2.0, 2.0});
}

// Test: CG position access through node pointer
TEST_F(TestCG, test_cg_node_pointer_access) {
    Node cg_node(StaticVector<double, 3UL>{1.5, 2.5, 3.5});
    CG cg(&cg_node);
    
    // Access position through CG methods
    const StaticVector<double, 3UL>& pos1 = cg.getPosition();
    
    // Access position through node pointer
    Node* node_ptr = cg.getNode();
    const StaticVector<double, 3UL>& pos2 = node_ptr->position;
    
    // Both should be the same
    expectVectorEqual(pos1, pos2);
}

