#ifndef AXLE_H
#define AXLE_H

#include "../tertiary_elements/double_wishbone.h"
#include "../secondary_elements/cg.h"
#include <blaze/Math.h>
#include <cmath>

using namespace blaze;

/**
 * Axle class represents a complete axle assembly with left and right suspension corners.
 * 
 * The Axle combines two DoubleWishbone instances (left and right) and provides
 * methods for coordinated movements like roll, steer, heave, and pitch.
 */
class Axle {
public:
    /**
     * Constructor
     * 
     * @param left_assy Left side DoubleWishbone assembly
     * @param right_assy Right side DoubleWishbone assembly
     * @param cg Center of gravity for the axle
     */
    Axle(DoubleWishbone *left_assy, DoubleWishbone *right_assy, CG *cg);
    
    /**
     * Apply roll angle to the axle
     * 
     * @param angle Roll angle in radians
     */
    void roll(double angle);
    
    /**
     * Reset roll to zero (both sides at same jounce)
     */
    void reset_roll();
    
    /**
     * Steer the axle (both wheels)
     * 
     * @param rack_displacement Lateral rack translation
     */
    void steer(double rack_displacement);
    
    /**
     * Apply heave (vertical translation) to the axle
     * 
     * @param heave Vertical displacement
     */
    void axle_heave(double heave);
    
    /**
     * Apply pitch (rotation about y-axis) to the axle
     * 
     * @param pitch Pitch displacement
     */
    void axle_pitch(double pitch);
    
    /**
     * Get track width (distance between left and right contact patches)
     * 
     * @return Track width in meters
     */
    double track_width() const;
    
    /**
     * Calculate roll stiffness of the axle
     * 
     * @return Roll stiffness in Nm/rad
     */
    double roll_stiffness() const;
    
    /**
     * Translate the entire axle
     * 
     * @param translation Translation vector
     */
    void translate(const StaticVector<double, 3UL> &translation);
    
    /**
     * Flatten rotate the entire axle
     * 
     * @param angle Rotation angles
     */
    void flatten_rotate(const StaticVector<double, 3UL> &angle);
    
    // Getters
    DoubleWishbone* getLeft() const { return left; }
    DoubleWishbone* getRight() const { return right; }
    CG* getCG() const { return cg; }
    
private:
    DoubleWishbone *left;
    DoubleWishbone *right;
    CG *cg;
    
    /**
     * Residual function for roll angle convergence
     * 
     * @param left_jounce Left side jounce guess
     * @param angle Target roll angle
     * @param LR_ratio Left/right ratio
     * @return Residual (calculated_roll - target_roll)
     */
    double _roll_resid_func(double left_jounce, double angle, double LR_ratio);
    
    /**
     * Simple root finder using bisection method for roll
     * 
     * @param angle Target roll angle
     * @param LR_ratio Left/right ratio
     * @param initial_guess Initial guess for left jounce
     * @return Left jounce that achieves target roll angle
     */
    double _find_roll_jounce(double angle, double LR_ratio, double initial_guess);
};

#endif //AXLE_H