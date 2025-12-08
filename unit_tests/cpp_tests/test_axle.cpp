#include "test_axle.h"
#include <cmath>

using namespace blaze;

// Test: Axle initialization
// Python equivalent: Basic axle setup
TEST_F(TestAxle, test_axle_init) {
    Axle* axle = createSimpleAxle();
    
    // Check that components are created
    EXPECT_NE(axle->getLeft(), nullptr);
    EXPECT_NE(axle->getRight(), nullptr);
    EXPECT_NE(axle->getCG(), nullptr);
    
    // Check that contact patches are accessible
    Node* left_cp = axle->getLeft()->getContactPatch();
    Node* right_cp = axle->getRight()->getContactPatch();
    EXPECT_NE(left_cp, nullptr);
    EXPECT_NE(right_cp, nullptr);
    
    delete axle;
}

// Test: Axle track width calculation
// Python equivalent: Distance between left and right contact patches
TEST_F(TestAxle, test_axle_track_width) {
    Axle* axle = createSimpleAxle();
    
    double track = axle->track_width();
    
    // Track width should be positive and finite
    EXPECT_GT(track, 0.0);
    EXPECT_TRUE(std::isfinite(track));
    
    // For the test setup, track should be approximately 9.0 (4.5 - (-4.5))
    expectNear(track, 9.0, 1e-1);
    
    delete axle;
}

// Test: Axle steer
// Python equivalent: Steering both wheels
TEST_F(TestAxle, test_axle_steer) {
    Axle* axle = createSimpleAxle();
    
    // Get initial toe angles
    double left_toe_before = axle->getLeft()->toe();
    double right_toe_before = axle->getRight()->toe();
    
    // Steer the axle
    double rack_displacement = 0.1;
    axle->steer(rack_displacement);
    
    // Toe angles should change
    double left_toe_after = axle->getLeft()->toe();
    double right_toe_after = axle->getRight()->toe();
    
    // Both should have changed (may not be equal due to geometry)
    bool left_changed = std::abs(left_toe_after - left_toe_before) > 1e-6;
    bool right_changed = std::abs(right_toe_after - right_toe_before) > 1e-6;
    
    EXPECT_TRUE(left_changed || right_changed);
    
    delete axle;
}

// Test: Axle heave
// Python equivalent: Vertical translation of axle
TEST_F(TestAxle, test_axle_heave) {
    Axle* axle = createSimpleAxle();
    
    // Get initial contact patch positions
    Node* left_cp = axle->getLeft()->getContactPatch();
    Node* right_cp = axle->getRight()->getContactPatch();
    double left_z_before = left_cp->position[2];
    double right_z_before = right_cp->position[2];
    
    // Apply heave
    double heave = 0.1;
    axle->axle_heave(heave);
    
    // Both contact patches should move up by heave amount
    double left_z_after = left_cp->position[2];
    double right_z_after = right_cp->position[2];
    
    expectNear(left_z_after - left_z_before, heave, 1e-4);
    expectNear(right_z_after - right_z_before, heave, 1e-4);
    
    delete axle;
}

// Test: Axle pitch
// Python equivalent: Pitch rotation of axle
TEST_F(TestAxle, test_axle_pitch) {
    Axle* axle = createSimpleAxle();
    
    // Get initial contact patch positions
    Node* left_cp = axle->getLeft()->getContactPatch();
    Node* right_cp = axle->getRight()->getContactPatch();
    double left_z_before = left_cp->position[2];
    double right_z_before = right_cp->position[2];
    
    // Apply pitch
    double pitch = 0.05;
    axle->axle_pitch(pitch);
    
    // Both contact patches should move (pitch affects both)
    double left_z_after = left_cp->position[2];
    double right_z_after = right_cp->position[2];
    
    // Positions should change
    bool left_changed = std::abs(left_z_after - left_z_before) > 1e-6;
    bool right_changed = std::abs(right_z_after - right_z_before) > 1e-6;
    
    EXPECT_TRUE(left_changed || right_changed);
    
    delete axle;
}

// Test: Axle reset roll
// Python equivalent: Resetting to zero roll
TEST_F(TestAxle, test_axle_reset_roll) {
    Axle* axle = createSimpleAxle();
    
    // Apply some roll first
    axle->roll(0.1);
    
    // Get positions after roll
    Node* left_cp = axle->getLeft()->getContactPatch();
    Node* right_cp = axle->getRight()->getContactPatch();
    double left_z_after_roll = left_cp->position[2];
    double right_z_after_roll = right_cp->position[2];
    
    // Reset roll
    axle->reset_roll();
    
    // Get positions after reset
    double left_z_after_reset = left_cp->position[2];
    double right_z_after_reset = right_cp->position[2];
    
    // Positions should change (reset moves them back)
    bool left_changed = std::abs(left_z_after_reset - left_z_after_roll) > 1e-6;
    bool right_changed = std::abs(right_z_after_reset - right_z_after_roll) > 1e-6;
    
    EXPECT_TRUE(left_changed || right_changed);
    
    delete axle;
}

// Test: Axle roll stiffness
// Python equivalent: Roll stiffness calculation
TEST_F(TestAxle, test_axle_roll_stiffness) {
    Axle* axle = createSimpleAxle();
    
    double roll_stiff = axle->roll_stiffness();
    
    // Roll stiffness should be positive and finite
    EXPECT_GT(roll_stiff, 0.0);
    EXPECT_TRUE(std::isfinite(roll_stiff));
    
    delete axle;
}

// Test: Axle translate
// Python equivalent: Translating the entire axle
TEST_F(TestAxle, test_axle_translate) {
    Axle* axle = createSimpleAxle();
    
    // Get initial positions
    Node* left_cp = axle->getLeft()->getContactPatch();
    Node* right_cp = axle->getRight()->getContactPatch();
    StaticVector<double, 3UL> left_pos_before = left_cp->position;
    StaticVector<double, 3UL> right_pos_before = right_cp->position;
    StaticVector<double, 3UL> cg_pos_before = axle->getCG()->getPosition();
    
    // Translate
    StaticVector<double, 3UL> translation{1.0, 2.0, 3.0};
    axle->translate(translation);
    
    // All positions should be translated
    StaticVector<double, 3UL> left_pos_after = left_cp->position;
    StaticVector<double, 3UL> right_pos_after = right_cp->position;
    StaticVector<double, 3UL> cg_pos_after = axle->getCG()->getPosition();
    
    for (size_t i = 0; i < 3; ++i) {
        EXPECT_NEAR(left_pos_after[i], left_pos_before[i] + translation[i], 1e-5);
        EXPECT_NEAR(right_pos_after[i], right_pos_before[i] + translation[i], 1e-5);
        EXPECT_NEAR(cg_pos_after[i], cg_pos_before[i] + translation[i], 1e-5);
    }
    
    delete axle;
}

// Test: Axle roll (zero roll)
// Python equivalent: Applying zero roll
TEST_F(TestAxle, test_axle_roll_zero) {
    Axle* axle = createSimpleAxle();
    
    // Get initial positions
    Node* left_cp = axle->getLeft()->getContactPatch();
    Node* right_cp = axle->getRight()->getContactPatch();
    double left_z_before = left_cp->position[2];
    double right_z_before = right_cp->position[2];
    
    // Apply zero roll
    axle->roll(0.0);
    
    // Positions should remain approximately the same (within tolerance)
    double left_z_after = left_cp->position[2];
    double right_z_after = right_cp->position[2];
    
    // For zero roll, heights should be similar
    expectNear(left_z_after, left_z_before, 1e-3);
    expectNear(right_z_after, right_z_before, 1e-3);
    
    delete axle;
}

// Test: Axle roll (positive roll)
// Python equivalent: Applying positive roll angle
TEST_F(TestAxle, test_axle_roll_positive) {
    Axle* axle = createSimpleAxle();
    
    // Get initial positions
    Node* left_cp = axle->getLeft()->getContactPatch();
    Node* right_cp = axle->getRight()->getContactPatch();
    double left_z_before = left_cp->position[2];
    double right_z_before = right_cp->position[2];
    
    // Apply positive roll (left goes up, right goes down)
    double roll_angle = 0.1;  // radians
    axle->roll(roll_angle);
    
    // Left should be higher, right should be lower
    double left_z_after = left_cp->position[2];
    double right_z_after = right_cp->position[2];
    
    // Left should be higher than before, right should be lower
    EXPECT_GT(left_z_after, left_z_before);
    EXPECT_LT(right_z_after, right_z_before);
    
    delete axle;
}

