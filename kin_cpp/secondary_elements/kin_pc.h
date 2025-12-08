#ifndef KIN_PC_H
#define KIN_PC_H

#include "../primary_elements/beam.h"
#include "cg.h"
#include <blaze/Math.h>

using namespace blaze;

/**
 * KinPC class represents the kinematic pitch center of a vehicle.
 * 
 * The pitch center is the intersection point of the front and rear swing arms
 * (or links) in the x-z plane. This matches the Python implementation where
 * PC is calculated using front_link.xz_intersection(link=rear_link).
 * 
 * The class maintains:
 * - true_KinPC: The actual kinematic pitch center (intersection of front and rear beams in xz plane)
 * - cg_axis_KinPC: Projection of the pitch center onto a vertical line through the CG
 */
class KinPC {
public:
    /**
     * Constructor
     * 
     * @param _front_swing_arm Beam representing the front swing arm/link
     * @param _rear_swing_arm Beam representing the rear swing arm/link
     * @param _cg Pointer to the CG (center of gravity) object
     */
    KinPC(Beam *_front_swing_arm, Beam *_rear_swing_arm, CG* _cg);
    
    /**
     * Update the pitch center positions based on current beam positions
     */
    void update();
    
    /**
     * Get the position of the pitch center projected onto CG axis
     * 
     * Projects the true pitch center onto a vertical line through the CG.
     * 
     * @return 3D position vector of CG-axis projected pitch center
     */
    StaticVector<double, 3UL> cg_axis_KinPC_pos() const;
    
    /**
     * Get the true kinematic pitch center position
     * 
     * Calculates the intersection of front and rear swing arms in the x-z plane.
     * This matches Python's front_link.xz_intersection(link=rear_link).
     * 
     * @return 3D position vector of the true pitch center
     */
    StaticVector<double, 3UL> true_KinPC_pos() const;
    
    /**
     * Translate the pitch center
     * 
     * @param translation Translation vector
     */
    void translate(const StaticVector<double, 3UL>& translation);
    
    /**
     * Rotate the pitch center (legacy method)
     * 
     * @param angle Rotation angles
     */
    void flatten_rotate(const StaticVector<double, 3UL>& angle);
    
    /**
     * Get the true pitch center node
     * 
     * @return Pointer to the true pitch center Node
     */
    Node* getTrueKinPC() const;
    
    /**
     * Get the CG-axis projected pitch center node
     * 
     * @return Pointer to the CG-axis projected pitch center Node
     */
    Node* getCGAxisKinPC() const;

private:
    Beam *front_swing_arm;  // Front swing arm/link beam
    Beam *rear_swing_arm;   // Rear swing arm/link beam
    CG* cg;                 // Center of gravity pointer
    
    Node* true_KinPC;       // True kinematic pitch center node
    Node* cg_axis_KinPC;    // CG-axis projected pitch center node
    
    double long_position;   // Longitudinal position (x-coordinate)
    double vertical_position; // Vertical position (z-coordinate)
    
    // For backward compatibility
    Node *elements[1];
    Node *all_elements[1];
};

#endif // KIN_PC_H
