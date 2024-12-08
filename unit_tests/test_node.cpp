#include <gtest/gtest.h>
#include <blaze/Math.h>
#include <kin_cpp/primary_elements/node.h>
#include "../assets/misc_linalg.h"
#include "test_node.h"

using namespace blaze;


void testNode::testNodePosition () {
    Node test_node(StaticVector<double, 3UL>({1, 1, 1}));
    StaticVector<double, 3UL> expected_position = {-1.0, 1.0, -1.0};
    EXPECT_EQ(test_node.position, expected_position);
}

void testNode::testNodeType () {
    Node test_node(StaticVector<double, 3UL>({1, 1, 1}));
    ASSERT_TRUE((std::is_same<decltype(test_node.position), StaticVector<double, 3UL>>::value))
        << "Expected position to be of type Static Vector";
}

void testNode::testNodeTranslatePosition () {
    Node test_node(StaticVector<double, 3UL>({1, 1, 1}));
    test_node.translate(StaticVector<double, 3UL>({-1, -1, -1}));
    StaticVector<double, 3UL> expected_position = {0, 0, 0};
    EXPECT_EQ(test_node.position, expected_position);
}

void testNode::testNodeTranslateType () {
    Node test_node(StaticVector<double, 3UL>({1, 1, 1}));
    test_node.translate(StaticVector<double, 3UL>({-1, -1, -1}));
    ASSERT_TRUE((std::is_same<decltype(test_node.position), StaticVector<double, 3UL>>::value))
        << "Expected position to be of type Static Vector";
}

void testNode::testNodeTranslateInitialPosition () {
    Node test_node(StaticVector<double, 3UL>({1, 1, 1}));
    test_node.translate(StaticVector<double, 3UL>({-1, -1, -1}));
    StaticVector<double, 3UL> expected_position = {0, 0, 0};
    EXPECT_EQ(test_node.position, expected_position);
}

void testNode::testNodeRotateBase () {
    Node test_node(StaticVector<double, 3UL>({0, 0, 0}));
    StaticVector<double, 3UL> rotation = {M_PI/2, M_PI/2, M_PI/2};
    test_node.flatten_rotate(rotation);
    StaticVector<double, 3UL> expected_position = {-1.0, 1.0, -1.0};

    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(test_node.position[i], expected_position[i], 1e-6) 
        << "Mismatch at index" << i;
    }
}

void testNode::testNodeResetPosition () {
    Node test_node(StaticVector<double, 3UL>({0, 0, 0}));
    StaticVector<double, 3UL> rotation = {M_PI/2, M_PI/2, M_PI/2};
    test_node.flatten_rotate(rotation);
    test_node.translate(StaticVector<double, 3UL>({-1, -1, -1}));
    test_node.reset();
    StaticVector<double, 3UL> expected_position = {0, 0, 0};
    EXPECT_EQ(test_node.position, expected_position);
}

void testNode::testNodeResetType () {
    Node test_node(StaticVector<double, 3UL>({0, 0, 0}));
    StaticVector<double, 3UL> rotation = {M_PI/2, M_PI/2, M_PI/2};
    test_node.flatten_rotate(rotation);
    test_node.translate(StaticVector<double, 3UL>({-1, -1, -1}));
    test_node.reset();
    ASSERT_TRUE((std::is_same<decltype(test_node.position), StaticVector<double, 3UL>>::value))
        << "Expected position to be of type Static Vector";
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}