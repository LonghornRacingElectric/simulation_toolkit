#ifndef STABAR_H
#define STABAR_H

#include "../primary_elements/beam.h"
#include "../primary_elements/node.h"
#include "../assets/misc_linalg.h"
#include <blaze/Math.h>
#include <cmath>
#include <utility>

using namespace blaze;

/**
 * Stabar class represents an anti-roll bar (stabilizer bar) in a vehicle suspension.
 * 
 * The stabar connects the left and right sides of the suspension through a torsion bar,
 * providing roll stiffness. When one side moves up or down relative to the other,
 * the torsion bar twists, generating a restoring torque.
 */
class Stabar : public Updateable {
public:
    /**
     * Constructor
     * 
     * @param left_arm_end End (farthest radially from axis) of stabar lever in positive y
     * @param right_arm_end End (farthest radially from axis) of stabar lever in negative y
     * @param left_droplink_end Droplink mounting point to rest of car in positive y
     * @param right_droplink_end Droplink mounting point to rest of car in negative y
     * @param bar_left_end End of torsion bar in positive y
     * @param bar_right_end End of torsion bar in negative y
     * @param torsional_stiffness Torsional stiffness of entire stabar (Nm/rad)
     */
    Stabar(Node* left_arm_end,
           Node* right_arm_end,
           Node* left_droplink_end,
           Node* right_droplink_end,
           Node* bar_left_end,
           Node* bar_right_end,
           double torsional_stiffness);
    
    /**
     * Update the stabar to match initial geometry
     * Finds the rotation angles that maintain droplink lengths
     */
    void update();
    
    /**
     * Get the angular deformation of the stabar
     * 
     * @return Angular deformation in radians (left_rotation - right_rotation)
     */
    double rotation() const;
    
    /**
     * Get the torque reacted by the stabar
     * 
     * @return Torque in Nm
     */
    double torque() const;
    
    // Getters
    Beam* getBar() const;
    Beam* getLeftArm() const;
    Beam* getRightArm() const;
    Beam* getLeftDroplink() const;
    Beam* getRightDroplink() const;
    double getLeftRotation() const;
    double getRightRotation() const;
    double getTorsionalStiffness() const;
    
    // Make _droplink_eqn accessible for testing
    double _droplink_eqn(double rotation, Beam* droplink);
    
private:
    Beam* bar;              // Torsion bar
    Beam* left_arm;         // Left arm
    Beam* right_arm;        // Right arm
    Beam* left_droplink;    // Left droplink
    Beam* right_droplink;   // Right droplink
    double torsional_stiffness;
    double left_rotation;   // Left rotation angle
    double right_rotation;  // Right rotation angle
    
    /**
     * Simple root finder using bisection method
     * Finds root of droplink equation within bounds
     * 
     * @param x0 Initial guess
     * @param bounds Pair of (lower_bound, upper_bound)
     * @param tol Tolerance for convergence
     * @param droplink Pointer to droplink for residual function
     * @return Root value
     */
    double nearest_root(double x0, 
                        std::pair<double, double> bounds, double tol, Beam* droplink);
};

#endif // STABAR_H

