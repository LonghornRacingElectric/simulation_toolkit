#include "test_pushrod.h"
#include <cmath>

using namespace blaze;

// Test: Pushrod initialization with length
TEST_F(TestPushrod, test_pushrod_init_length) {
    Node inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{0, 0, 5});
    Beam pushrod_beam(&inboard, &outboard);
    
    Pushrod pushrod(&pushrod_beam);
    
    expectNear(pushrod.getLength(), 5.0, 1e-6);
}

// Test: Pushrod initial angle
TEST_F(TestPushrod, test_pushrod_init_angle) {
    Node inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{0, 0, 5});
    Beam pushrod_beam(&inboard, &outboard);
    
    Pushrod pushrod(&pushrod_beam);
    
    expectNear(pushrod.getAngle(), 0.0, 1e-6);
}

// Test: Pushrod getBeam returns correct beam
TEST_F(TestPushrod, test_pushrod_get_beam) {
    Node inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{0, 0, 5});
    Beam pushrod_beam(&inboard, &outboard);
    
    Pushrod pushrod(&pushrod_beam);
    
    EXPECT_EQ(pushrod.getBeam(), &pushrod_beam);
}

// Test: Pushrod length calculation with different positions
TEST_F(TestPushrod, test_pushrod_length_calculation) {
    Node inboard(StaticVector<double, 3UL>{1, 1, 1});
    Node outboard(StaticVector<double, 3UL>{2, 2, 2});
    Beam pushrod_beam(&inboard, &outboard);
    
    Pushrod pushrod(&pushrod_beam);
    
    // Distance between (1,1,1) and (2,2,2) = sqrt(3)
    double expected_length = std::sqrt(3.0);
    expectNear(pushrod.getLength(), expected_length, 1e-6);
}

// Test: Pushrod length updates when beam changes
TEST_F(TestPushrod, test_pushrod_length_updates) {
    Node inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{0, 0, 3});
    Beam pushrod_beam(&inboard, &outboard);
    
    Pushrod pushrod(&pushrod_beam);
    
    // Initial length
    expectNear(pushrod.getLength(), 3.0, 1e-6);
    
    // Translate outboard node
    outboard.translate(StaticVector<double, 3UL>{0, 0, 2});
    
    // Recalculate length
    double new_length = pushrod.calculateLength();
    expectNear(new_length, 5.0, 1e-6);
}

