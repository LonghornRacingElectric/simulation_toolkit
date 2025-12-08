#include "test_spring.h"
#include <cmath>

using namespace blaze;

// Test: Spring initialization
// Python: test_spring_init
TEST_F(TestSpring, test_spring_init) {
    Node inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{0, 0.5, 0});
    
    double free_length = 1.0;
    double rate = 0.5;
    
    Spring spring(&inboard, &outboard, free_length, rate);
    
    // Check that free_length is stored correctly
    // Python: self.assertEqual(spring.free_length, 1)
    EXPECT_DOUBLE_EQ(spring.getFreeLength(), 1.0);
}

// Test: Spring compression calculation (initial state)
// Python: test_spring_compression_one
TEST_F(TestSpring, test_spring_compression_one) {
    Node inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{0, 0.5, 0});
    
    double free_length = 1.0;
    double rate = 0.5;
    
    Spring spring(&inboard, &outboard, free_length, rate);
    
    // Compression = free_length - current_length
    // Current length = distance between [0,0,0] and [0,0.5,0] = 0.5
    // Compression = 1.0 - 0.5 = 0.5
    // Python: self.assertEqual(spring.compression, 0.5)
    EXPECT_DOUBLE_EQ(spring.compression(), 0.5);
}

// Test: Spring compression calculation (after node movement)
// Python: test_spring_compression_two
TEST_F(TestSpring, test_spring_compression_two) {
    Node inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{0, 0.5, 0});
    
    double free_length = 1.0;
    double rate = 0.5;
    
    Spring spring(&inboard, &outboard, free_length, rate);
    
    // Move outboard node: spring.outboard_node[1] = 0.25
    // Python: spring.outboard_node[1] = 0.25
    outboard.position[1] = 0.25;
    
    // Compression = free_length - current_length
    // Current length = distance between [0,0,0] and [0,0.25,0] = 0.25
    // Compression = 1.0 - 0.25 = 0.75
    // Python: self.assertEqual(spring.compression, 0.75)
    EXPECT_DOUBLE_EQ(spring.compression(), 0.75);
}

// Test: Spring force calculation (initial state)
// Python: test_spring_force_one
TEST_F(TestSpring, test_spring_force_one) {
    Node inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{0, 0.5, 0});
    
    double free_length = 1.0;
    double rate = 0.5;
    
    Spring spring(&inboard, &outboard, free_length, rate);
    
    // Force = rate * compression
    // Compression = 1.0 - 0.5 = 0.5
    // Force = 0.5 * 0.5 = 0.25
    // Python: self.assertEqual(spring.force, 0.25)
    EXPECT_DOUBLE_EQ(spring.force(), 0.25);
}

// Test: Spring force calculation (when spring is at free length)
// Python: test_spring_force_two
TEST_F(TestSpring, test_spring_force_two) {
    Node inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{0, 0.5, 0});
    
    double free_length = 1.0;
    double rate = 0.5;
    
    Spring spring(&inboard, &outboard, free_length, rate);
    
    // Move outboard node: spring.outboard_node[1] = 1
    // Python: spring.outboard_node[1] = 1
    outboard.position[1] = 1.0;
    
    // Force = rate * compression
    // Current length = distance between [0,0,0] and [0,1,0] = 1.0
    // Compression = 1.0 - 1.0 = 0.0
    // Force = 0.5 * 0.0 = 0.0
    // Python: self.assertEqual(spring.force, 0)
    EXPECT_DOUBLE_EQ(spring.force(), 0.0);
}

