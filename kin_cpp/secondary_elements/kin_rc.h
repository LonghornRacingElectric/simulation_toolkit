#ifndef KIN_RC_H
#define KIN_RC_H

#include "../primary_elements/beam.h"
#include "cg.h"
#include <blaze/Math.h>

using namespace blaze;

/**
 * KinRC class represents the kinematic roll center of a vehicle.
 * 
 * The roll center is the intersection point of the left and right swing arms
 * (or links) in the y-z plane. This matches the Python implementation where
 * RC is calculated using left_link.yz_intersection(link=right_link).
 * 
 * The class maintains:
 * - true_KinRC: The actual kinematic roll center (intersection of left and right beams in yz plane)
 * - cg_axis_KinRC: Projection of the roll center onto a vertical line through the CG
 */
class KinRC {
public:
    /**
     * Constructor
     * 
     * @param _left_swing_arm Beam representing the left swing arm/link
     * @param _right_swing_arm Beam representing the right swing arm/link
     * @param _cg Pointer to the CG (center of gravity) object
     */
    KinRC(Beam *_left_swing_arm, Beam *_right_swing_arm, CG *_cg);
    
    /**
     * Update the roll center positions based on current beam positions
     */
    void update();
    
    /**
     * Get the position of the roll center projected onto CG axis
     * 
     * Projects the true roll center onto a vertical line through the CG.
     * 
     * @return 3D position vector of CG-axis projected roll center
     */
    StaticVector<double, 3UL> cg_axis_KinRC_pos() const;
    
    /**
     * Get the true kinematic roll center position
     * 
     * Calculates the intersection of left and right swing arms in the y-z plane.
     * This matches Python's left_link.yz_intersection(link=right_link).
     * 
     * @return 3D position vector of the true roll center
     */
    StaticVector<double, 3UL> true_kinRC_pos() const;
    
    /**
     * Translate the roll center
     * 
     * @param translation Translation vector
     */
    void translate(const StaticVector<double, 3UL> &translation);
    
    /**
     * Rotate the roll center (legacy method)
     * 
     * @param angle Rotation angles
     */
    void flatten_rotate(const StaticVector<double, 3UL> &angle);
    
    /**
     * Get the true roll center node
     * 
     * @return Pointer to the true roll center Node
     */
    Node* getTrueKinRC() const;
    
    /**
     * Get the CG-axis projected roll center node
     * 
     * @return Pointer to the CG-axis projected roll center Node
     */
    Node* getCGAxisKinRC() const;

private:
    Beam *left_swing_arm;   // Left swing arm/link beam
    Beam *right_swing_arm;  // Right swing arm/link beam
    CG *cg;                 // Center of gravity pointer
    
    Node *true_KinRC;       // True kinematic roll center node
    Node *cg_axis_KinRC;   // CG-axis projected roll center node
    
    double lateral_position;  // Lateral position (y-coordinate)
    double vertical_position;  // Vertical position (z-coordinate)
    
    // For backward compatibility
    Node *elements[1];
    Node *all_elements[1];
};

#endif // KIN_RC_H
