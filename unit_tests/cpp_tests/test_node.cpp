#include "test_node.h"
#include <cmath>
#include <vector>

using namespace blaze;

// Test: Node initialization with position
TEST_F(TestNode, test_node_init_position) {
    Node test_node(StaticVector<double, 3UL>{1, 1, 1});
    StaticVector<double, 3UL> expected = {1, 1, 1};
    expectVectorEqual(test_node.position, expected);
}

// Test: Node translation
TEST_F(TestNode, test_node_translate_position) {
    Node test_node(StaticVector<double, 3UL>{1, 1, 1});
    test_node.translate(StaticVector<double, 3UL>{-1, -1, -1});
    StaticVector<double, 3UL> expected = {0, 0, 0};
    expectVectorEqual(test_node.position, expected);
}

// Test: Initial position is preserved after translation
TEST_F(TestNode, test_node_translate_initial_position) {
    Node test_node(StaticVector<double, 3UL>{1, 1, 1});
    test_node.translate(StaticVector<double, 3UL>{-1, -1, -1});
    StaticVector<double, 3UL> expected = {1, 1, 1};
    expectVectorEqual(test_node.initial_position, expected);
}

// Test: Node rotation with base angles (ang_x, ang_y, ang_z)
TEST_F(TestNode, test_node_rotate_base_angles) {
    Node origin_node(StaticVector<double, 3UL>{0, 0, 0});
    Node update_node(StaticVector<double, 3UL>{1, 1, 0});
    
    update_node.rotate(origin_node, false, std::nullopt, std::nullopt, 0.0, 0.0, M_PI);
    StaticVector<double, 3UL> expected = {-1, -1, 0};
    expectVectorNear(update_node.position, expected);
}

// Test: Node rotation edge case (rotating node at origin)
TEST_F(TestNode, test_node_rotate_edge_angles) {
    Node origin_node(StaticVector<double, 3UL>{0, 0, 0});
    Node update_node(StaticVector<double, 3UL>{0, 0, 0});
    
    update_node.rotate(origin_node, false, std::nullopt, std::nullopt, 0.0, 0.0, M_PI);
    StaticVector<double, 3UL> expected = {0, 0, 0};
    expectVectorNear(update_node.position, expected);
}

// Test: Node rotation with non-default angles
TEST_F(TestNode, test_node_rotate_ND_angles) {
    Node origin_node(StaticVector<double, 3UL>{1, 1, 1});
    Node update_node(StaticVector<double, 3UL>{0, 0, 0});
    
    update_node.rotate(origin_node, false, std::nullopt, std::nullopt, M_PI, M_PI/2, 0.0);
    StaticVector<double, 3UL> expected = {2, 2, 2};
    expectVectorNear(update_node.position, expected);
}

// Test: Node rotation with base vector (direction and angle)
TEST_F(TestNode, test_node_rotate_base_vector) {
    Node origin_node(StaticVector<double, 3UL>{0, 0, 0});
    Node update_node(StaticVector<double, 3UL>{1, 1, 0});
    
    update_node.rotate(origin_node, false, StaticVector<double, 3UL>{0, 0, 1}, M_PI);
    StaticVector<double, 3UL> expected = {-1, -1, 0};
    expectVectorNear(update_node.position, expected);
}

// Test: Node rotation edge case with vector (rotating node at origin)
TEST_F(TestNode, test_node_rotate_edge_vector) {
    Node origin_node(StaticVector<double, 3UL>{0, 0, 0});
    Node update_node(StaticVector<double, 3UL>{0, 0, 0});
    
    update_node.rotate(origin_node, false, StaticVector<double, 3UL>{0, 0, 1}, 0.0);
    StaticVector<double, 3UL> expected = {0, 0, 0};
    expectVectorNear(update_node.position, expected);
}

// Test: Node rotation repeated (two rotations)
TEST_F(TestNode, test_node_rotate_repeated_one) {
    Node origin_node(StaticVector<double, 3UL>{0, 0, 0});
    Node update_node(StaticVector<double, 3UL>{0, 1, 0});
    
    update_node.rotate(origin_node, false, StaticVector<double, 3UL>{0, 0, 1}, M_PI/2);
    update_node.rotate(origin_node, false, StaticVector<double, 3UL>{0, 0, 1}, M_PI/2);
    StaticVector<double, 3UL> expected = {-1, 0, 0};
    expectVectorNear(update_node.position, expected);
}

// Test: Node rotation repeated (rotation and reverse rotation)
TEST_F(TestNode, test_node_rotate_repeated_two) {
    Node origin_node(StaticVector<double, 3UL>{0, 0, 0});
    Node update_node(StaticVector<double, 3UL>{0, 1, 0});
    
    update_node.rotate(origin_node, false, StaticVector<double, 3UL>{0, 0, 1}, M_PI/2);
    update_node.rotate(origin_node, false, StaticVector<double, 3UL>{0, 0, 1}, -M_PI/2);
    StaticVector<double, 3UL> expected = {1, 0, 0};
    expectVectorNear(update_node.position, expected);
}

// Test: Node rotation repeated (rotation and zero rotation)
TEST_F(TestNode, test_node_rotate_repeated_three) {
    Node origin_node(StaticVector<double, 3UL>{0, 0, 0});
    Node update_node(StaticVector<double, 3UL>{0, 1, 0});
    
    update_node.rotate(origin_node, false, StaticVector<double, 3UL>{0, 0, 1}, M_PI/2);
    update_node.rotate(origin_node, false, StaticVector<double, 3UL>{0, 0, 1}, 0.0);
    StaticVector<double, 3UL> expected = {0, 1, 0};
    expectVectorNear(update_node.position, expected);
}

// Test: Node reset position
TEST_F(TestNode, test_node_reset_position) {
    Node origin_node(StaticVector<double, 3UL>{1, 1, 1});
    Node update_node(StaticVector<double, 3UL>{0, 0, 0});
    
    update_node.translate(StaticVector<double, 3UL>{-1, -1, -1});
    update_node.rotate(origin_node, false, std::nullopt, std::nullopt, M_PI, M_PI/2, 0.0);
    update_node.reset();
    StaticVector<double, 3UL> expected = {0, 0, 0};
    expectVectorEqual(update_node.position, expected);
}

// Test: Node addition
TEST_F(TestNode, test_node_addition) {
    Node first_node(StaticVector<double, 3UL>{1, 1, 1});
    Node second_node(StaticVector<double, 3UL>{1, 1, 1});
    
    Node node_sum = first_node + second_node;
    StaticVector<double, 3UL> expected = {2, 2, 2};
    expectVectorEqual(node_sum.position, expected);
}

// Test: Node subtraction
TEST_F(TestNode, test_node_subtraction) {
    Node first_node(StaticVector<double, 3UL>{3, 3, 3});
    Node second_node(StaticVector<double, 3UL>{2, 2, 2});
    
    Node node_diff = first_node - second_node;
    StaticVector<double, 3UL> expected = {1, 1, 1};
    expectVectorEqual(node_diff.position, expected);
}

// Test: Node multiplication
TEST_F(TestNode, test_node_multiplication) {
    Node node(StaticVector<double, 3UL>{1, 1, 1});
    
    Node node_product = node * 2;
    StaticVector<double, 3UL> expected = {2, 2, 2};
    expectVectorEqual(node_product.position, expected);
}

// Test: Node division
TEST_F(TestNode, test_node_division) {
    Node node(StaticVector<double, 3UL>{2, 2, 2});
    
    Node node_quotient = node / 2;
    StaticVector<double, 3UL> expected = {1, 1, 1};
    expectVectorEqual(node_quotient.position, expected);
}

// Test: Node indexing (getitem)
TEST_F(TestNode, test_node_getitem) {
    Node node(StaticVector<double, 3UL>{1, 2, 3});
    
    EXPECT_EQ(node[2], 3);
}

// Test: Node indexing (setitem)
TEST_F(TestNode, test_node_setitem) {
    Node node(StaticVector<double, 3UL>{1, 2, 3});
    node[2] = 1;
    
    EXPECT_EQ(node[2], 1);
}

// Test: Node child addition
TEST_F(TestNode, test_node_child) {
    Node node(StaticVector<double, 3UL>{0, 0, 0});
    Node node_child(StaticVector<double, 3UL>{0, 0, 1});
    
    node.add_child(&node_child);
    
    // Check that child is in child_nodes (we can't directly access private members,
    // but we can verify behavior through translate)
    EXPECT_EQ(node_child[2], 1);
}

// Test: Node child translation
TEST_F(TestNode, test_node_child_translate) {
    Node node(StaticVector<double, 3UL>{0, 0, 0});
    Node node_child(StaticVector<double, 3UL>{0, 0, 1});
    
    node.add_child(&node_child);
    node.translate(StaticVector<double, 3UL>{0, 0, 1});
    
    EXPECT_NEAR(node_child[2], 2.0, 1e-6);
}

// Test: Node child reset
TEST_F(TestNode, test_node_child_reset) {
    Node node(StaticVector<double, 3UL>{0, 0, 0});
    Node node_child(StaticVector<double, 3UL>{0, 0, 1});
    
    node.add_child(&node_child);
    node.translate(StaticVector<double, 3UL>{0, 0, 1});
    node.reset();
    
    EXPECT_NEAR(node_child[2], 1.0, 1e-6);
}

// Test: Node child rotation with angles
TEST_F(TestNode, test_node_child_rotate_angles) {
    Node origin_node(StaticVector<double, 3UL>{0, 0, 0});
    Node update_node(StaticVector<double, 3UL>{1, 1, 0});
    Node node_child(StaticVector<double, 3UL>{-1, -1, 0});
    
    update_node.add_child(&node_child);
    update_node.rotate(origin_node, false, StaticVector<double, 3UL>{0, 0, 1}, M_PI);
    StaticVector<double, 3UL> expected = {1, 1, 0};
    expectVectorNear(node_child.position, expected);
}

// Test: Node child rotation with vector
TEST_F(TestNode, test_node_child_rotate_vector) {
    Node origin_node(StaticVector<double, 3UL>{0, 0, 0});
    Node update_node(StaticVector<double, 3UL>{1, 1, 0});
    Node node_child(StaticVector<double, 3UL>{-1, -1, 0});
    
    update_node.add_child(&node_child);
    update_node.rotate(origin_node, false, std::nullopt, std::nullopt, 0.0, 0.0, M_PI);
    StaticVector<double, 3UL> expected = {1, 1, 0};
    expectVectorNear(node_child.position, expected);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
