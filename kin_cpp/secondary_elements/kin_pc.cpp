#include "kin_pc.h"
#include "../primary_elements/node.h"
#include "../assets/misc_linalg.h"

using namespace blaze;

/**
 * Constructor
 * 
 * Initializes the kinematic pitch center by calculating the intersection
 * of front and rear swing arms in the x-z plane.
 */
KinPC::KinPC(Beam *_front_swing_arm, Beam *_rear_swing_arm, CG *_cg) {
    front_swing_arm = _front_swing_arm;
    rear_swing_arm = _rear_swing_arm;
    cg = _cg;
    
    // Calculate true kinematic pitch center (intersection in xz plane)
    StaticVector<double, 3UL> pc_pos = true_KinPC_pos();
    true_KinPC = new Node(pc_pos);
    long_position = true_KinPC->position[0];
    vertical_position = true_KinPC->position[2];
    
    // Calculate CG-axis projected pitch center
    StaticVector<double, 3UL> cg_axis_pos = cg_axis_KinPC_pos();
    cg_axis_KinPC = new Node(cg_axis_pos);
    
    elements[0] = all_elements[0] = true_KinPC;
}

/**
 * Update the pitch center positions based on current beam positions
 */
void KinPC::update() {
    // Recalculate true pitch center from current beam positions
    true_KinPC->position = true_KinPC_pos();
    long_position = true_KinPC->position[0];
    vertical_position = true_KinPC->position[2];
    
    // Recalculate CG-axis projected pitch center
    cg_axis_KinPC->position = cg_axis_KinPC_pos();
}

/**
 * Get the position of the pitch center projected onto CG axis
 * 
 * Projects the true pitch center onto a vertical line through the CG.
 * Since CG in Python is just a Node (no direction), we use a default
 * vertical direction [0, 0, 1] for the projection.
 * 
 * @return 3D position vector of CG-axis projected pitch center
 */
StaticVector<double, 3UL> KinPC::cg_axis_KinPC_pos() const {
    // Recalculate true pitch center from current beam positions (don't use stored position)
    StaticVector<double, 3UL> kpc_pos = true_KinPC_pos();
    const StaticVector<double, 3UL> &cg_pos = cg->getPosition();
    
    // Use vertical direction [0, 0, 1] for projection (CG doesn't have direction in Python)
    // Project onto vertical line through CG: x = (kpc_z - cg_z) * dir_x / dir_z + cg_x
    // Since direction is [0, 0, 1], dir_x = 0, dir_z = 1, so x = cg_x
    StaticVector<double, 3UL> cg_dir{0, 0, 1};  // Vertical direction
    double x = (kpc_pos[2] - cg_pos[2]) * cg_dir[0] / cg_dir[2] + cg_pos[0]; 
    
    return StaticVector<double, 3UL>{x, kpc_pos[1], kpc_pos[2]};
}

/**
 * Get the true kinematic pitch center position
 * 
 * Calculates the intersection of front and rear swing arms in the x-z plane.
 * This matches Python's front_link.xz_intersection(link=rear_link).
 * 
 * @return 3D position vector of the true pitch center
 */
StaticVector<double, 3UL> KinPC::true_KinPC_pos() const {
    // Calculate intersection in xz plane (y is averaged between the two beams)
    Node pc_node = front_swing_arm->xz_intersection(*rear_swing_arm);
    return pc_node.position;
}

/**
 * Translate the pitch center
 * 
 * @param translation Translation vector
 */
void KinPC::translate(const StaticVector<double, 3UL>& translation) {
    true_KinPC->translate(translation);
}

/**
 * Rotate the pitch center (legacy method)
 * 
 * @param angle Rotation angles
 */
void KinPC::flatten_rotate(const StaticVector<double, 3UL>& angle) {
    true_KinPC->flatten_rotate(angle);
}

/**
 * Get the true pitch center node
 * 
 * @return Pointer to the true pitch center Node
 */
Node* KinPC::getTrueKinPC() const {
    return true_KinPC;
}

/**
 * Get the CG-axis projected pitch center node
 * 
 * @return Pointer to the CG-axis projected pitch center Node
 */
Node* KinPC::getCGAxisKinPC() const {
    return cg_axis_KinPC;
}
