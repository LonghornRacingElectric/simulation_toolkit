#ifndef TIE_H
#define TIE_H

#include "../primary_elements/beam.h"
#include "kingpin.h"
#include <blaze/Math.h>

using namespace blaze;

/**
 * Tie class represents a steering tie rod in a vehicle suspension model.
 * 
 * The tie rod connects a Beam (the tie rod itself) to a Kingpin (steering pivot).
 * It maintains the relative position of the steering pickup point in kingpin-local
 * coordinates and updates the tie rod's outboard node based on steering angle.
 */
class Tie {
public: 
    /**
     * Constructor
     * 
     * @param beam Pointer to the tie rod Beam
     * @param kp Pointer to the associated Kingpin
     */
    Tie(Beam *beam, Kingpin *kp);
    
    /**
     * Get the current length of the tie rod
     * 
     * @return Current length
     */
    double getLength() const;
    
    /**
     * Get the current steering angle
     * 
     * @return Current steering angle in radians
     */
    double getAngle() const;
    
    /**
     * Get the initial length of the tie rod (at construction)
     * 
     * @return Initial length
     */
    double getInitialLength() const;
    
    /**
     * Get pointer to the tie rod Beam
     * 
     * @return Pointer to tie_beam
     */
    Beam *getTieBeam() const;
    
    /**
     * Get the relative position from steering pickup to kingpin in kingpin-local coordinates
     * 
     * @return 3D vector in kingpin-local coordinate system
     */
    StaticVector<double, 3UL> _steering_pickup_to_kingpin() const;
    
    /**
     * Update the tie rod's outboard node position based on current steering angle
     * Transforms the steering pickup position from kingpin-local to world coordinates
     */
    void update();
    
    /**
     * Rotate the tie rod to a new steering angle
     * 
     * @param new_angle New steering angle in radians
     */
    void rotate(double new_angle);
    
    /**
     * Reset the tie rod to its initial position (zero steering angle)
     */
    void set_initial_position();
    
private:
    Beam *tie_beam;                              // Pointer to the tie rod Beam
    Kingpin *kingpin;                             // Pointer to the associated Kingpin
    double length;                                // Current length of the tie rod
    double initial_length;                        // Initial length (at construction)
    double angle;                                 // Current steering angle in radians
    StaticVector<double, 3UL> steering_pickup_to_kingpin;  // Relative position in kingpin-local coords
    
    /**
     * Calculate the current length of the tie rod from its beam
     * 
     * @return Current length
     */
    double calculateLength();
};

#endif // TIE_H
