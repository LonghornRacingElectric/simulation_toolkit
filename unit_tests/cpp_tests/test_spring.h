#ifndef TEST_SPRING_H
#define TEST_SPRING_H

#include <blaze/Math.h>
#include <cmath>
#include "../kin_cpp/primary_elements/node.h"
#include "../kin_cpp/secondary_elements/spring.h"
#include "../kin_cpp/assets/misc_linalg.h"
#include "../kin_cpp/third_party/googletest/googletest/include/gtest/gtest.h"

using namespace blaze;

class TestSpring : public ::testing::Test {
protected:
    void SetUp() override {
        // Setup code if needed
    }
    
    void TearDown() override {
        // Cleanup code if needed
    }
};

#endif // TEST_SPRING_H

