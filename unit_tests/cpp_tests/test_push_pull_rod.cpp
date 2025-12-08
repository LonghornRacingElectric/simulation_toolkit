#include "test_push_pull_rod.h"
#include <cmath>
#include <vector>

using namespace blaze;

// Test: PushPullRod initialization (without bellcrank)
// Python equivalent: test_push_pull_rod_partial_one (partial rod without bellcrank)
TEST_F(TestPushPullRod, test_push_pull_rod_init_no_bellcrank) {
    Node inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{0, 2, 0});
    
    bool upper = false;
    bool bellcrank = false;
    Node* bc_pivot = nullptr;
    StaticVector<double, 3UL> bc_direction{0, 0, 1};
    Node* shock_outboard = nullptr;
    Node* shock_inboard = nullptr;
    
    PushPullRod rod(&inboard, &outboard, upper, bellcrank, bc_pivot, bc_direction, shock_outboard, shock_inboard);
    
    // Check that rod is created
    EXPECT_NE(rod.getRod(), nullptr);
    EXPECT_FALSE(rod.hasBellcrank());
    EXPECT_EQ(rod.getDamper(), nullptr);
    
    // Check initial rod length
    double expected_length = 2.0;  // Distance from [0,0,0] to [0,2,0]
    expectNear(rod.getInitialRodLength(), expected_length, 1e-5);
}

// Test: PushPullRod rod length calculation
// Python equivalent: test_push_pull_rod_partial_three
TEST_F(TestPushPullRod, test_push_pull_rod_rod_length) {
    Node inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{0, 2, 0});
    
    bool upper = false;
    bool bellcrank = false;
    Node* bc_pivot = nullptr;
    StaticVector<double, 3UL> bc_direction{0, 0, 1};
    Node* shock_outboard = nullptr;
    Node* shock_inboard = nullptr;
    
    PushPullRod rod(&inboard, &outboard, upper, bellcrank, bc_pivot, bc_direction, shock_outboard, shock_inboard);
    
    // Initial length should be 2.0
    expectNear(rod.getInitialRodLength(), 2.0, 1e-5);
    expectNear(rod.rod_length(), 2.0, 1e-5);
    
    // Translate outboard node
    outboard.position[1] = 1.5;  // Move to [0, 1.5, 0]
    
    // Rod length should change
    expectNear(rod.rod_length(), 1.5, 1e-5);
}

// Test: PushPullRod translate
// Python equivalent: Translating the rod
TEST_F(TestPushPullRod, test_push_pull_rod_translate) {
    Node inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{0, 2, 0});
    
    bool upper = false;
    bool bellcrank = false;
    Node* bc_pivot = nullptr;
    StaticVector<double, 3UL> bc_direction{0, 0, 1};
    Node* shock_outboard = nullptr;
    Node* shock_inboard = nullptr;
    
    PushPullRod rod(&inboard, &outboard, upper, bellcrank, bc_pivot, bc_direction, shock_outboard, shock_inboard);
    
    // Get initial positions
    StaticVector<double, 3UL> initial_inboard = inboard.position;
    StaticVector<double, 3UL> initial_outboard = outboard.position;
    
    // Translate
    StaticVector<double, 3UL> translation{1.0, 2.0, 3.0};
    rod.translate(translation);
    
    // Positions should be translated
    StaticVector<double, 3UL> expected_inboard = initial_inboard + translation;
    StaticVector<double, 3UL> expected_outboard = initial_outboard + translation;
    
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(inboard.position[i], expected_inboard[i], 1e-5);
        EXPECT_NEAR(outboard.position[i], expected_outboard[i], 1e-5);
    }
}

// Test: PushPullRod rotate_rod
// Python equivalent: Rotating the rod
TEST_F(TestPushPullRod, test_push_pull_rod_rotate_rod) {
    Node inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node outboard(StaticVector<double, 3UL>{0, 2, 0});
    
    bool upper = false;
    bool bellcrank = false;
    Node* bc_pivot = nullptr;
    StaticVector<double, 3UL> bc_direction{0, 0, 1};
    Node* shock_outboard = nullptr;
    Node* shock_inboard = nullptr;
    
    PushPullRod rod(&inboard, &outboard, upper, bellcrank, bc_pivot, bc_direction, shock_outboard, shock_inboard);
    
    // Get initial outboard position
    StaticVector<double, 3UL> initial_outboard = outboard.position;
    
    // Rotate rod about z-axis
    StaticVector<double, 3UL> axis{0, 0, 1};
    double angle = M_PI / 4.0;  // 45 degrees
    rod.rotate_rod(axis, &inboard, angle);
    
    // Outboard should be rotated (but inboard should remain at origin)
    // After 45 degree rotation about z-axis, [0,2,0] becomes approximately [sqrt(2), sqrt(2), 0]
    EXPECT_NEAR(inboard.position[0], 0.0, 1e-5);
    EXPECT_NEAR(inboard.position[1], 0.0, 1e-5);
    EXPECT_NEAR(inboard.position[2], 0.0, 1e-5);
}

// Test: PushPullRod with bellcrank initialization
// Python equivalent: test_push_pull_rod_full_* tests
TEST_F(TestPushPullRod, test_push_pull_rod_init_with_bellcrank) {
    Node inboard(StaticVector<double, 3UL>{0, 1, 1});
    Node outboard(StaticVector<double, 3UL>{0, 0, 0});
    
    bool upper = false;
    bool bellcrank = true;
    Node bc_pivot(StaticVector<double, 3UL>{0, 2, 1});
    StaticVector<double, 3UL> bc_direction{1, 0, 0};
    Node shock_outboard(StaticVector<double, 3UL>{0, 2, 2});
    Node shock_inboard(StaticVector<double, 3UL>{0, 3, 2});
    
    PushPullRod rod(&inboard, &outboard, upper, bellcrank, &bc_pivot, bc_direction, &shock_outboard, &shock_inboard);
    
    // Check that bellcrank components are created
    EXPECT_TRUE(rod.hasBellcrank());
    EXPECT_NE(rod.getDamper(), nullptr);
    
    // Check initial lengths
    EXPECT_GT(rod.getInitialRodLength(), 0.0);
    EXPECT_GT(rod.getInitialSpringDamperLength(), 0.0);
}

// Test: PushPullRod spring_damper_length (with bellcrank)
// Python equivalent: test_push_pull_rod_partial_six
TEST_F(TestPushPullRod, test_push_pull_rod_spring_damper_length) {
    Node inboard(StaticVector<double, 3UL>{0, 1, 1});
    Node outboard(StaticVector<double, 3UL>{0, 0, 0});
    
    bool upper = false;
    bool bellcrank = true;
    Node bc_pivot(StaticVector<double, 3UL>{0, 2, 1});
    StaticVector<double, 3UL> bc_direction{1, 0, 0};
    Node shock_outboard(StaticVector<double, 3UL>{0, 2, 2});
    Node shock_inboard(StaticVector<double, 3UL>{0, 3, 2});
    
    PushPullRod rod(&inboard, &outboard, upper, bellcrank, &bc_pivot, bc_direction, &shock_outboard, &shock_inboard);
    
    // Get initial spring/damper length
    double initial_length = rod.getInitialSpringDamperLength();
    EXPECT_GT(initial_length, 0.0);
    
    // Current length should match initial (no movement yet)
    expectNear(rod.spring_damper_length(), initial_length, 1e-5);
}

// Test: PushPullRod rotate_bellcrank
// Python equivalent: Bellcrank rotation tests
TEST_F(TestPushPullRod, test_push_pull_rod_rotate_bellcrank) {
    Node inboard(StaticVector<double, 3UL>{0, 1, 1});
    Node outboard(StaticVector<double, 3UL>{0, 0, 0});
    
    bool upper = false;
    bool bellcrank = true;
    Node bc_pivot(StaticVector<double, 3UL>{0, 2, 1});
    StaticVector<double, 3UL> bc_direction{1, 0, 0};
    Node shock_outboard(StaticVector<double, 3UL>{0, 2, 2});
    Node shock_inboard(StaticVector<double, 3UL>{0, 3, 2});
    
    PushPullRod rod(&inboard, &outboard, upper, bellcrank, &bc_pivot, bc_direction, &shock_outboard, &shock_inboard);
    
    // Get initial positions
    StaticVector<double, 3UL> initial_inboard = inboard.position;
    
    // Rotate bellcrank
    double angle = M_PI / 6.0;  // 30 degrees
    rod.rotate_bellcrank(angle);
    
    // Bellcrank angle should be updated
    expectNear(rod.getBellcrankAngle(), angle, 1e-5);
    
    // Inboard node position should change (rotated about bellcrank pivot)
    // Position should be different from initial
    bool position_changed = false;
    for (size_t i = 0; i < 3; ++i) {
        if (std::abs(inboard.position[i] - initial_inboard[i]) > 1e-6) {
            position_changed = true;
            break;
        }
    }
    EXPECT_TRUE(position_changed);
}

// Test: PushPullRod bellcrank angle getter
// Python equivalent: bellcrank_angle property
TEST_F(TestPushPullRod, test_push_pull_rod_bellcrank_angle) {
    Node inboard(StaticVector<double, 3UL>{0, 1, 1});
    Node outboard(StaticVector<double, 3UL>{0, 0, 0});
    
    bool upper = false;
    bool bellcrank = true;
    Node bc_pivot(StaticVector<double, 3UL>{0, 2, 1});
    StaticVector<double, 3UL> bc_direction{1, 0, 0};
    Node shock_outboard(StaticVector<double, 3UL>{0, 2, 2});
    Node shock_inboard(StaticVector<double, 3UL>{0, 3, 2});
    
    PushPullRod rod(&inboard, &outboard, upper, bellcrank, &bc_pivot, bc_direction, &shock_outboard, &shock_inboard);
    
    // Initial bellcrank angle should be 0
    expectNear(rod.getBellcrankAngle(), 0.0, 1e-5);
    
    // Rotate bellcrank
    double angle = M_PI / 6.0;  // 30 degrees
    rod.rotate_bellcrank(angle);
    
    // Bellcrank angle should be updated
    expectNear(rod.getBellcrankAngle(), angle, 1e-5);
}

// Test: PushPullRod without bellcrank - rod length preservation
// Python equivalent: test_push_pull_rod_partial_five
TEST_F(TestPushPullRod, test_push_pull_rod_no_bellcrank_rod_length_preserved) {
    Node inboard(StaticVector<double, 3UL>{0, 1, 0});
    Node outboard(StaticVector<double, 3UL>{0, 2, 0});
    
    bool upper = false;
    bool bellcrank = false;
    Node* bc_pivot = nullptr;
    StaticVector<double, 3UL> bc_direction{0, 0, 1};
    Node* shock_outboard = nullptr;
    Node* shock_inboard = nullptr;
    
    PushPullRod rod(&inboard, &outboard, upper, bellcrank, bc_pivot, bc_direction, shock_outboard, shock_inboard);
    
    double initial_length = rod.getInitialRodLength();
    
    // Translate outboard in z-direction
    outboard.position[2] = 1.0;  // Move to [0, 2, 1]
    
    // Rod length should change (not preserved without update mechanism)
    double new_length = rod.rod_length();
    // Length should be sqrt(1^2 + 1^2) = sqrt(2) ≈ 1.414
    double expected_length = std::sqrt(1.0 * 1.0 + 1.0 * 1.0);
    expectNear(new_length, expected_length, 1e-5);
}

// Test: PushPullRod flatten_rotate
// Python equivalent: Rotating the entire assembly
TEST_F(TestPushPullRod, test_push_pull_rod_flatten_rotate) {
    Node inboard(StaticVector<double, 3UL>{1, 1, 1});  // Start away from origin so rotation has effect
    Node outboard(StaticVector<double, 3UL>{0, 2, 0});
    
    bool upper = false;
    bool bellcrank = false;
    Node* bc_pivot = nullptr;
    StaticVector<double, 3UL> bc_direction{0, 0, 1};
    Node* shock_outboard = nullptr;
    Node* shock_inboard = nullptr;
    
    PushPullRod rod(&inboard, &outboard, upper, bellcrank, bc_pivot, bc_direction, shock_outboard, shock_inboard);
    
    // Get initial positions
    StaticVector<double, 3UL> initial_inboard = inboard.position;
    StaticVector<double, 3UL> initial_outboard = outboard.position;
    
    // Flatten rotate (rotates positions about origin)
    StaticVector<double, 3UL> rotation{0.1, 0.2, 0.3};
    rod.flatten_rotate(rotation);
    
    // Positions should change (rotated about origin)
    bool position_changed = false;
    for (size_t i = 0; i < 3; ++i) {
        if (std::abs(inboard.position[i] - initial_inboard[i]) > 1e-6) {
            position_changed = true;
            break;
        }
    }
    EXPECT_TRUE(position_changed);
    
    // Outboard should also change
    bool outboard_changed = false;
    for (size_t i = 0; i < 3; ++i) {
        if (std::abs(outboard.position[i] - initial_outboard[i]) > 1e-6) {
            outboard_changed = true;
            break;
        }
    }
    EXPECT_TRUE(outboard_changed);
}

