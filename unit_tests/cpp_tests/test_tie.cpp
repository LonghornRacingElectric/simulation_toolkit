#include "test_tie.h"
#include <cmath>

using namespace blaze;

// Test: Tie rod initialization
TEST_F(TestTie, test_tie_init) {
    Node kingpin_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node kingpin_outboard(StaticVector<double, 3UL>{0, 0, 1});
    Beam kingpin_beam(&kingpin_inboard, &kingpin_outboard);
    Kingpin kingpin(&kingpin_beam);
    
    Node tie_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node tie_outboard(StaticVector<double, 3UL>{1, 0, 0});
    Beam tie_beam(&tie_inboard, &tie_outboard);
    
    Tie tie(&tie_beam, &kingpin);
    
    // Check that tie rod was created
    EXPECT_NE(tie.getTieBeam(), nullptr);
    EXPECT_EQ(tie.getTieBeam(), &tie_beam);
}

// Test: Initial length is stored correctly
TEST_F(TestTie, test_tie_init_length) {
    Node kingpin_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node kingpin_outboard(StaticVector<double, 3UL>{0, 0, 1});
    Beam kingpin_beam(&kingpin_inboard, &kingpin_outboard);
    Kingpin kingpin(&kingpin_beam);
    
    Node tie_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node tie_outboard(StaticVector<double, 3UL>{3, 4, 0});
    Beam tie_beam(&tie_inboard, &tie_outboard);
    
    Tie tie(&tie_beam, &kingpin);
    
    // Distance between (0,0,0) and (3,4,0) = 5
    double expected_length = 5.0;
    expectNear(tie.getInitialLength(), expected_length, 1e-6);
    expectNear(tie.getLength(), expected_length, 1e-6);
}

// Test: Initial angle is zero
TEST_F(TestTie, test_tie_init_angle) {
    Node kingpin_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node kingpin_outboard(StaticVector<double, 3UL>{0, 0, 1});
    Beam kingpin_beam(&kingpin_inboard, &kingpin_outboard);
    Kingpin kingpin(&kingpin_beam);
    
    Node tie_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node tie_outboard(StaticVector<double, 3UL>{1, 0, 0});
    Beam tie_beam(&tie_inboard, &tie_outboard);
    
    Tie tie(&tie_beam, &kingpin);
    
    expectNear(tie.getAngle(), 0.0, 1e-6);
}

// Test: Get tie beam returns correct beam
TEST_F(TestTie, test_tie_get_beam) {
    Node kingpin_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node kingpin_outboard(StaticVector<double, 3UL>{0, 0, 1});
    Beam kingpin_beam(&kingpin_inboard, &kingpin_outboard);
    Kingpin kingpin(&kingpin_beam);
    
    Node tie_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node tie_outboard(StaticVector<double, 3UL>{1, 0, 0});
    Beam tie_beam(&tie_inboard, &tie_outboard);
    
    Tie tie(&tie_beam, &kingpin);
    
    EXPECT_EQ(tie.getTieBeam(), &tie_beam);
}

// Test: Steering pickup to kingpin calculation (vertical kingpin, horizontal tie)
TEST_F(TestTie, test_tie_steering_pickup_to_kingpin_vertical) {
    // Vertical kingpin along z-axis
    Node kingpin_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node kingpin_outboard(StaticVector<double, 3UL>{0, 0, 1});
    Beam kingpin_beam(&kingpin_inboard, &kingpin_outboard);
    Kingpin kingpin(&kingpin_beam);
    
    // Horizontal tie rod along x-axis
    Node tie_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node tie_outboard(StaticVector<double, 3UL>{1, 0, 0});
    Beam tie_beam(&tie_inboard, &tie_outboard);
    
    Tie tie(&tie_beam, &kingpin);
    
    // For a vertical kingpin and horizontal tie, the relative position
    // in kingpin-local coords should be (1, 0, 0) since kingpin is already aligned with z
    StaticVector<double, 3UL> relative_pos = tie._steering_pickup_to_kingpin();
    
    // The relative position should be the tie outboard relative to kingpin inboard
    // In this simple case, it should be close to (1, 0, 0)
    expectVectorNear(relative_pos, StaticVector<double, 3UL>{1, 0, 0}, 1e-5);
}

// Test: Length calculation after position change
TEST_F(TestTie, test_tie_length_calculation) {
    Node kingpin_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node kingpin_outboard(StaticVector<double, 3UL>{0, 0, 1});
    Beam kingpin_beam(&kingpin_inboard, &kingpin_outboard);
    Kingpin kingpin(&kingpin_beam);
    
    Node tie_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node tie_outboard(StaticVector<double, 3UL>{1, 0, 0});
    Beam tie_beam(&tie_inboard, &tie_outboard);
    
    Tie tie(&tie_beam, &kingpin);
    
    double initial_length = tie.getLength();
    expectNear(initial_length, 1.0, 1e-6);
    
    // Change tie outboard position
    tie_beam.getOutboardNode()->position = StaticVector<double, 3UL>{2, 0, 0};
    
    // Length should update (though we need to recalculate)
    // The tie class should handle this in update()
}

// Test: Rotate tie rod to new angle
TEST_F(TestTie, test_tie_rotate) {
    Node kingpin_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node kingpin_outboard(StaticVector<double, 3UL>{0, 0, 1});
    Beam kingpin_beam(&kingpin_inboard, &kingpin_outboard);
    Kingpin kingpin(&kingpin_beam);
    
    Node tie_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node tie_outboard(StaticVector<double, 3UL>{1, 0, 0});
    Beam tie_beam(&tie_inboard, &tie_outboard);
    
    Tie tie(&tie_beam, &kingpin);
    
    // Store initial position
    StaticVector<double, 3UL> initial_pos = tie_beam.getOutboardNode()->position;
    
    // Rotate by 90 degrees (π/2 radians)
    double rotation_angle = M_PI / 2.0;
    tie.rotate(rotation_angle);
    
    // Check that angle was updated
    expectNear(tie.getAngle(), rotation_angle, 1e-6);
    
    // Position should have changed
    StaticVector<double, 3UL> new_pos = tie_beam.getOutboardNode()->position;
    // For a vertical kingpin rotating about z-axis, the tie should rotate in xy plane
    // Initial: (1, 0, 0), after 90° rotation: should be approximately (0, 1, 0)
    expectVectorNear(new_pos, StaticVector<double, 3UL>{0, 1, 0}, 1e-5);
}

// Test: Rotate tie rod to negative angle
TEST_F(TestTie, test_tie_rotate_negative) {
    Node kingpin_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node kingpin_outboard(StaticVector<double, 3UL>{0, 0, 1});
    Beam kingpin_beam(&kingpin_inboard, &kingpin_outboard);
    Kingpin kingpin(&kingpin_beam);
    
    Node tie_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node tie_outboard(StaticVector<double, 3UL>{1, 0, 0});
    Beam tie_beam(&tie_inboard, &tie_outboard);
    
    Tie tie(&tie_beam, &kingpin);
    
    // Rotate by -90 degrees (-π/2 radians)
    double rotation_angle = -M_PI / 2.0;
    tie.rotate(rotation_angle);
    
    // Check that angle was updated
    expectNear(tie.getAngle(), rotation_angle, 1e-6);
    
    // Position should have changed
    StaticVector<double, 3UL> new_pos = tie_beam.getOutboardNode()->position;
    // After -90° rotation: should be approximately (0, -1, 0)
    expectVectorNear(new_pos, StaticVector<double, 3UL>{0, -1, 0}, 1e-5);
}

// Test: Multiple rotations
TEST_F(TestTie, test_tie_rotate_multiple) {
    Node kingpin_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node kingpin_outboard(StaticVector<double, 3UL>{0, 0, 1});
    Beam kingpin_beam(&kingpin_inboard, &kingpin_outboard);
    Kingpin kingpin(&kingpin_beam);
    
    Node tie_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node tie_outboard(StaticVector<double, 3UL>{1, 0, 0});
    Beam tie_beam(&tie_inboard, &tie_outboard);
    
    Tie tie(&tie_beam, &kingpin);
    
    // First rotation: 90 degrees
    tie.rotate(M_PI / 2.0);
    StaticVector<double, 3UL> pos1 = tie_beam.getOutboardNode()->position;
    expectVectorNear(pos1, StaticVector<double, 3UL>{0, 1, 0}, 1e-5);
    
    // Second rotation: another 90 degrees (total 180)
    tie.rotate(M_PI);
    StaticVector<double, 3UL> pos2 = tie_beam.getOutboardNode()->position;
    // After 180° rotation from initial: should be approximately (-1, 0, 0)
    expectVectorNear(pos2, StaticVector<double, 3UL>{-1, 0, 0}, 1e-5);
    
    // Check final angle
    expectNear(tie.getAngle(), M_PI, 1e-6);
}

// Test: Set initial position resets to zero angle
TEST_F(TestTie, test_tie_set_initial_position) {
    Node kingpin_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node kingpin_outboard(StaticVector<double, 3UL>{0, 0, 1});
    Beam kingpin_beam(&kingpin_inboard, &kingpin_outboard);
    Kingpin kingpin(&kingpin_beam);
    
    Node tie_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node tie_outboard(StaticVector<double, 3UL>{1, 0, 0});
    Beam tie_beam(&tie_inboard, &tie_outboard);
    
    Tie tie(&tie_beam, &kingpin);
    
    // Store initial position
    StaticVector<double, 3UL> initial_pos = tie_beam.getOutboardNode()->position;
    
    // Rotate to some angle
    tie.rotate(M_PI / 4.0);
    StaticVector<double, 3UL> rotated_pos = tie_beam.getOutboardNode()->position;
    
    // Verify position changed
    double dist_after_rotation = distance(initial_pos, rotated_pos);
    EXPECT_GT(dist_after_rotation, 0.1);  // Should have moved
    
    // Reset to initial position
    tie.set_initial_position();
    
    // Position should be back to initial (approximately)
    StaticVector<double, 3UL> reset_pos = tie_beam.getOutboardNode()->position;
    expectVectorNear(reset_pos, initial_pos, 1e-5);
}

// Test: Length remains constant after rotation (for vertical kingpin)
TEST_F(TestTie, test_tie_length_after_rotation) {
    Node kingpin_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node kingpin_outboard(StaticVector<double, 3UL>{0, 0, 1});
    Beam kingpin_beam(&kingpin_inboard, &kingpin_outboard);
    Kingpin kingpin(&kingpin_beam);
    
    Node tie_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node tie_outboard(StaticVector<double, 3UL>{1, 0, 0});
    Beam tie_beam(&tie_inboard, &tie_outboard);
    
    Tie tie(&tie_beam, &kingpin);
    
    double initial_length = tie.getLength();
    
    // Rotate by various angles
    tie.rotate(M_PI / 4.0);
    expectNear(tie.getLength(), initial_length, 1e-5);
    
    tie.rotate(M_PI / 2.0);
    expectNear(tie.getLength(), initial_length, 1e-5);
    
    tie.rotate(M_PI);
    expectNear(tie.getLength(), initial_length, 1e-5);
}

// Test: Update method changes position
TEST_F(TestTie, test_tie_update) {
    Node kingpin_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node kingpin_outboard(StaticVector<double, 3UL>{0, 0, 1});
    Beam kingpin_beam(&kingpin_inboard, &kingpin_outboard);
    Kingpin kingpin(&kingpin_beam);
    
    Node tie_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node tie_outboard(StaticVector<double, 3UL>{1, 0, 0});
    Beam tie_beam(&tie_inboard, &tie_outboard);
    
    Tie tie(&tie_beam, &kingpin);
    
    StaticVector<double, 3UL> initial_pos = tie_beam.getOutboardNode()->position;
    
    // Manually set angle and update (simulating internal state change)
    // Note: We can't directly set angle, so we use rotate which calls update
    tie.rotate(M_PI / 2.0);
    
    StaticVector<double, 3UL> updated_pos = tie_beam.getOutboardNode()->position;
    
    // Position should have changed
    double dist = distance(initial_pos, updated_pos);
    EXPECT_GT(dist, 0.1);
}

// Test: Rotate and return to zero
TEST_F(TestTie, test_tie_rotate_return_to_zero) {
    Node kingpin_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node kingpin_outboard(StaticVector<double, 3UL>{0, 0, 1});
    Beam kingpin_beam(&kingpin_inboard, &kingpin_outboard);
    Kingpin kingpin(&kingpin_beam);
    
    Node tie_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node tie_outboard(StaticVector<double, 3UL>{1, 0, 0});
    Beam tie_beam(&tie_inboard, &tie_outboard);
    
    Tie tie(&tie_beam, &kingpin);
    
    StaticVector<double, 3UL> initial_pos = tie_beam.getOutboardNode()->position;
    
    // Rotate to some angle
    tie.rotate(M_PI / 3.0);
    
    // Rotate back to zero
    tie.rotate(0.0);
    
    // Should be back to initial position
    StaticVector<double, 3UL> final_pos = tie_beam.getOutboardNode()->position;
    expectVectorNear(final_pos, initial_pos, 1e-5);
    expectNear(tie.getAngle(), 0.0, 1e-6);
}

// Test: Tie rod with angled kingpin
TEST_F(TestTie, test_tie_angled_kingpin) {
    // Angled kingpin (not vertical)
    Node kingpin_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node kingpin_outboard(StaticVector<double, 3UL>{1, 1, 1});
    Beam kingpin_beam(&kingpin_inboard, &kingpin_outboard);
    Kingpin kingpin(&kingpin_beam);
    
    Node tie_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node tie_outboard(StaticVector<double, 3UL>{1, 0, 0});
    Beam tie_beam(&tie_inboard, &tie_outboard);
    
    Tie tie(&tie_beam, &kingpin);
    
    // Should still initialize correctly
    double initial_length = tie.getLength();
    EXPECT_GT(initial_length, 0.0);
    
    // Should be able to rotate
    tie.rotate(M_PI / 4.0);
    expectNear(tie.getAngle(), M_PI / 4.0, 1e-6);
}

// Test: Initial length is preserved after rotations
TEST_F(TestTie, test_tie_initial_length_preserved) {
    Node kingpin_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node kingpin_outboard(StaticVector<double, 3UL>{0, 0, 1});
    Beam kingpin_beam(&kingpin_inboard, &kingpin_outboard);
    Kingpin kingpin(&kingpin_beam);
    
    Node tie_inboard(StaticVector<double, 3UL>{0, 0, 0});
    Node tie_outboard(StaticVector<double, 3UL>{2, 0, 0});
    Beam tie_beam(&tie_inboard, &tie_outboard);
    
    Tie tie(&tie_beam, &kingpin);
    
    double initial_length = tie.getInitialLength();
    expectNear(initial_length, 2.0, 1e-6);
    
    // Rotate multiple times
    tie.rotate(M_PI / 4.0);
    tie.rotate(M_PI / 2.0);
    tie.rotate(M_PI);
    
    // Initial length should remain unchanged
    expectNear(tie.getInitialLength(), initial_length, 1e-6);
}

