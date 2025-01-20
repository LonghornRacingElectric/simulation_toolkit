/*
#include "../kin_cpp/third_party/googletest/googletest/include/gtest/gtest.h"
#include <iostream>
#include <vector>
#include "../kin_cpp/third_party/blaze/blaze/math/Math.h"
#include "../kin_cpp/assets/misc_linalg.h" 
#include "../kin_cpp/primary_elements/node.h"
*/
#include "../kin_cpp/third_party/googletest/googletest/include/gtest/gtest.h"
#include <iostream>
#include <vector>
#include <algorithm>

TEST(VectorTest, EqualityCheck) {
    std::vector<int> expected_position = {0, 0, 0};
    std::vector<int> initial_position = {3, 2, 0};

    // Google Test supports direct vector comparison
    EXPECT_EQ(initial_position, expected_position);
}

int main(int argc, char **argv) {
    std::cout << "hello" << std::endl;
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS(); // Runs all Google Test cases
}
