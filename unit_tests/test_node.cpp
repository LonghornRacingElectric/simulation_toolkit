#include <gtest/gtest.h>
#include <blaze/Math.h>
#include <kin_cpp/primary_elements/node.h>
#include "../assets/misc_linalg.h"
#include "test_node.h"


void testNode::testNodePosition () {
    Node test_node({1, 1, 1});
    EXPECT_EQ(test_node.position, std::vector<double>({1.0, 1.0, 1.0}));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}