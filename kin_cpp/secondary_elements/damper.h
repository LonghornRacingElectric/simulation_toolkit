#ifndef DAMPER_H
#define DAMPER_H

#include "../primary_elements/beam.h"
#include <vector>
#include <utility>

using namespace blaze;

class Damper {
public:
    /**
     * Damper constructor
     * 
     * @param inboard_node Node representing inboard end of damper
     * @param outboard_node Node representing outboard end of damper
     * @param damping_curve Lookup table for damping values, in the form: vector of pairs (velocity, force)
     */
    Damper(Node* inboard_node, Node* outboard_node, 
           const std::vector<std::pair<double, double>>& damping_curve);
    
    /**
     * Get the damper force at current velocity
     * 
     * @return Damper force (interpolated from damping curve)
     */
    double force() const;
    
    // Getters
    Beam* getBeam() const;
    Node* getInboardNode() const;
    Node* getOutboardNode() const;
    double getVelocity() const;
    const std::vector<double>& getVelocityReference() const;
    const std::vector<double>& getForceReference() const;
    
    // Setters
    void setVelocity(double velocity);
    
private:
    Beam* damper_beam;
    std::vector<double> velocity_reference;  // Velocity values from damping curve
    std::vector<double> force_reference;     // Force values from damping curve
    double velocity;                          // Current velocity
    
    /**
     * Linear interpolation helper function
     * 
     * @param x Value to interpolate at
     * @param x_data X-axis data points (must be sorted)
     * @param y_data Y-axis data points
     * @return Interpolated y value
     */
    double interpolate(double x, 
                      const std::vector<double>& x_data, 
                      const std::vector<double>& y_data) const;
};

#endif // DAMPER_H
