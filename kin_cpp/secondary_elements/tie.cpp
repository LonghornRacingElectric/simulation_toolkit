#include "tie.h"
#include "../assets/misc_linalg.h"
#include <cmath>

using namespace blaze;

/**
 * Helper function to compute steering pickup to kingpin position.
 * 
 * This transforms the steering pickup point (tie rod outboard node) from world
 * coordinates to the kingpin's local coordinate system, where the kingpin beam
 * is aligned with the z-axis.
 * 
 * @param tie_beam Pointer to the tie rod beam
 * @param kingpin Pointer to the kingpin
 * @return 3D vector representing the relative position in kingpin-local coords
 */
static StaticVector<double, 3UL> computeSteeringPickupToKingpin(
    const Beam* tie_beam, 
    const Kingpin* kingpin
) {
    // Get the rotation angles needed to align kingpin beam with z-axis
    // These are the angles that rotate the kingpin beam to point in +z direction
    StaticVector<double, 2UL> angles = kingpin->getBeam()->rotation_angles();
    
    // Get the steering pickup position relative to kingpin inboard node
    StaticVector<double, 3UL> steering_pickup_pos_shifted = 
        tie_beam->getOutboardNode()->position - 
        kingpin->getBeam()->getInboardNode()->position;
    
    // Apply inverse rotations to transform to kingpin-local coordinates
    // First rotate about x-axis by angles[0]
    StaticMatrix<double, 3UL, 3UL> x_rot = rotation_matrix(
        StaticVector<double, 3UL>{1, 0, 0}, 
        angles[0]
    );
    
    // Then rotate about y-axis by -angles[1] (inverse)
    StaticMatrix<double, 3UL, 3UL> y_rot = rotation_matrix(
        StaticVector<double, 3UL>{0, 1, 0}, 
        -angles[1]
    );
    
    // Apply transformations: y_rot * x_rot * position
    return y_rot * (x_rot * steering_pickup_pos_shifted);
}

/**
 * Constructor
 * 
 * Initializes the Tie with a Beam and Kingpin, computes initial geometry,
 * and stores the relative position in kingpin-local coordinates.
 */
Tie::Tie(Beam *beam, Kingpin *kp) {
    tie_beam = beam;
    kingpin = kp;
    angle = 0.0;
    
    // Calculate initial length
    length = calculateLength();
    initial_length = length;
    
    // Compute and store the relative position in kingpin-local coordinates
    steering_pickup_to_kingpin = computeSteeringPickupToKingpin(tie_beam, kingpin);
}

/**
 * Get the relative position from steering pickup to kingpin in kingpin-local coordinates.
 * 
 * This returns the stored relative position that was computed during construction.
 * The position represents the steering pickup point in the kingpin's local coordinate
 * system, where the kingpin beam is aligned with the z-axis.
 * 
 * @return 3D vector representing the relative position in kingpin-local coords
 */
StaticVector<double, 3UL> Tie::_steering_pickup_to_kingpin() const {
    return steering_pickup_to_kingpin;
}

/**
 * Get the current length of the tie rod
 * 
 * @return Current length
 */
double Tie::getLength() const {
    return length;
}

/**
 * Get the current steering angle
 * 
 * @return Current steering angle in radians
 */
double Tie::getAngle() const {
    return angle;
}

/**
 * Get the initial length of the tie rod
 * 
 * @return Initial length
 */
double Tie::getInitialLength() const {
    return initial_length;
}

/**
 * Get pointer to the tie rod Beam
 * 
 * @return Pointer to tie_beam
 */
Beam *Tie::getTieBeam() const {
    return tie_beam;
}

/**
 * Calculate the current length of the tie rod from its beam
 * 
 * @return Current length
 */
double Tie::calculateLength() {
    length = tie_beam->length();
    return length;
}

/**
 * Update the tie rod's outboard node position based on current steering angle.
 * 
 * This method transforms the steering pickup position from kingpin-local coordinates
 * back to world coordinates, applying the current steering rotation about the kingpin axis.
 * 
 * The transformation process:
 * 1. Rotate the local position by the steering angle about the z-axis (kingpin axis)
 * 2. Transform back to world coordinates by applying inverse of kingpin alignment rotations
 * 3. Translate to world position by adding kingpin inboard node position
 */
void Tie::update() {
    // Get the rotation angles needed to align kingpin beam with z-axis
    StaticVector<double, 2UL> angles = kingpin->getBeam()->rotation_angles();
    
    // Create rotation matrices to transform from kingpin-local to world coordinates
    // Inverse of the transformations used in computeSteeringPickupToKingpin
    StaticMatrix<double, 3UL, 3UL> x_rot = rotation_matrix(
        StaticVector<double, 3UL>{1, 0, 0}, 
        -angles[0]  // Inverse rotation
    );
    
    StaticMatrix<double, 3UL, 3UL> y_rot = rotation_matrix(
        StaticVector<double, 3UL>{0, 1, 0}, 
        angles[1]   // Inverse rotation (was -angles[1] before, so now positive)
    );
    
    // Rotate about z-axis (kingpin axis) by the steering angle
    StaticMatrix<double, 3UL, 3UL> z_rot = rotation_matrix(
        StaticVector<double, 3UL>{0, 0, 1}, 
        angle
    );
    
    // Apply transformations in order:
    // 1. Rotate by steering angle about z-axis (in local coords)
    // 2. Transform to world coordinates: x_rot * y_rot
    // 3. Translate to world position
    StaticVector<double, 3UL> steering_pickup_position = 
        x_rot * (y_rot * (z_rot * steering_pickup_to_kingpin)) + 
        kingpin->getBeam()->getInboardNode()->position;
    
    // Update the tie rod's outboard node position
    tie_beam->getOutboardNode()->position = steering_pickup_position;
    
    // Recalculate length after position update
    calculateLength();
}

/**
 * Reset the tie rod to its initial position (zero steering angle).
 * 
 * This rotates the tie rod back to its initial orientation by applying
 * the inverse of the current steering rotation about the kingpin axis.
 */
void Tie::set_initial_position() {
    // Get the kingpin beam direction (rotation axis)
    StaticVector<double, 3UL> kingpin_direction = kingpin->getBeam()->direction();
    
    // Rotate back by negative of current angle to return to initial position
    StaticMatrix<double, 3UL, 3UL> kingpin_rot = rotation_matrix(
        kingpin_direction, 
        -angle
    );
    
    // Get current position relative to kingpin inboard node
    StaticVector<double, 3UL> steer_about_origin = 
        tie_beam->getOutboardNode()->position - 
        kingpin->getBeam()->getInboardNode()->position;
    
    // Apply rotation and translate back
    tie_beam->getOutboardNode()->position = 
        kingpin_rot * steer_about_origin + 
        kingpin->getBeam()->getInboardNode()->position;
    
    // Recalculate length after position update
    calculateLength();
}

/**
 * Rotate the tie rod to a new steering angle.
 * 
 * This method:
 * 1. Resets the tie rod to its initial position
 * 2. Updates the steering angle
 * 3. Applies the new rotation to update the outboard node position
 * 
 * @param new_angle New steering angle in radians
 */
void Tie::rotate(double new_angle) {
    // Reset to initial position first
    set_initial_position();
    
    // Update the steering angle
    angle = new_angle; // remove 2.0 to test if tests catch it
    
    // Update the position based on the new angle
    update();
}
