#include "kin_rc.h"
#include "../primary_elements/node.h"
#include "../assets/misc_linalg.h"

using namespace blaze;

/**
 * Constructor
 * 
 * Initializes the kinematic roll center by calculating the intersection
 * of left and right swing arms in the y-z plane.
 */
KinRC::KinRC(Beam *_left_swing_arm, Beam *_right_swing_arm, CG *_cg) {
    left_swing_arm = _left_swing_arm;
    right_swing_arm = _right_swing_arm;
    cg = _cg;
    
    // Calculate true kinematic roll center (intersection in yz plane)
    StaticVector<double, 3UL> rc_pos = true_kinRC_pos();
    true_KinRC = new Node(rc_pos);
    lateral_position = true_KinRC->position[1];
    vertical_position = true_KinRC->position[2];
    
    // Calculate CG-axis projected roll center
    StaticVector<double, 3UL> cg_axis_pos = cg_axis_KinRC_pos();
    cg_axis_KinRC = new Node(cg_axis_pos);
    
    elements[0] = all_elements[0] = true_KinRC;
}

/**
 * Update the roll center positions based on current beam positions
 */
void KinRC::update() {
    // Recalculate true roll center from current beam positions
    true_KinRC->position = true_kinRC_pos();
    lateral_position = true_KinRC->position[1];
    vertical_position = true_KinRC->position[2];
    
    // Recalculate CG-axis projected roll center
    cg_axis_KinRC->position = cg_axis_KinRC_pos();
}

/**
 * Get the position of the roll center projected onto CG axis
 * 
 * Projects the true roll center onto a vertical line through the CG.
 * Since CG in Python is just a Node (no direction), we use a default
 * vertical direction [0, 0, 1] for the projection.
 * 
 * @return 3D position vector of CG-axis projected roll center
 */
StaticVector<double, 3UL> KinRC::cg_axis_KinRC_pos() const {
    // Recalculate true roll center from current beam positions (don't use stored position)
    StaticVector<double, 3UL> krc_pos = true_kinRC_pos();
    const StaticVector<double, 3UL> &cg_pos = cg->getPosition();
    
    // Use vertical direction [0, 0, 1] for projection (CG doesn't have direction in Python)
    // Project onto vertical line through CG: y = (krc_z - cg_z) * dir_y / dir_z + cg_y
    // Since direction is [0, 0, 1], dir_y = 0, dir_z = 1, so y = cg_y
    StaticVector<double, 3UL> cg_dir{0, 0, 1};  // Vertical direction
    double y = (krc_pos[2] - cg_pos[2]) * cg_dir[1] / cg_dir[2] + cg_pos[1]; 
    
    return StaticVector<double, 3UL>{krc_pos[0], y, krc_pos[2]};
}

/**
 * Get the true kinematic roll center position
 * 
 * Calculates the intersection of left and right swing arms in the y-z plane.
 * This matches Python's left_link.yz_intersection(link=right_link).
 * 
 * @return 3D position vector of the true roll center
 */
StaticVector<double, 3UL> KinRC::true_kinRC_pos() const {
    // Calculate intersection in yz plane (x is averaged between the two beams)
    Node rc_node = left_swing_arm->yz_intersection(*right_swing_arm);
    return rc_node.position;
}

/**
 * Translate the roll center
 * 
 * @param translation Translation vector
 */
void KinRC::translate(const StaticVector<double, 3UL> &translation) {
    true_KinRC->translate(translation);
}

/**
 * Rotate the roll center (legacy method)
 * 
 * @param angle Rotation angles
 */
void KinRC::flatten_rotate(const StaticVector<double, 3UL> &angle) {
    true_KinRC->flatten_rotate(angle);
}

/**
 * Get the true roll center node
 * 
 * @return Pointer to the true roll center Node
 */
Node* KinRC::getTrueKinRC() const {
    return true_KinRC;
}

/**
 * Get the CG-axis projected roll center node
 * 
 * @return Pointer to the CG-axis projected roll center Node
 */
Node* KinRC::getCGAxisKinRC() const {
    return cg_axis_KinRC;
}
