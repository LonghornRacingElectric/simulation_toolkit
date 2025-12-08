#include "damper.h"
#include "../primary_elements/node.h"
#include <algorithm>
#include <stdexcept>

Damper::Damper(Node* inboard_node, Node* outboard_node,
               const std::vector<std::pair<double, double>>& damping_curve)
    : velocity(0.0) {
    
    // Create the beam from the nodes
    damper_beam = new Beam(inboard_node, outboard_node);
    
    // Extract velocity and force arrays from damping curve
    // Python: self.damping_curve = list(zip(*damping_curve))
    // This transposes the pairs into two separate vectors
    velocity_reference.reserve(damping_curve.size());
    force_reference.reserve(damping_curve.size());
    
    for (const auto& pair : damping_curve) {
        velocity_reference.push_back(pair.first);   // velocity
        force_reference.push_back(pair.second);     // force
    }
    
    // Verify that velocity_reference is sorted (required for interpolation)
    // In practice, damping curves should be sorted by velocity
    if (!std::is_sorted(velocity_reference.begin(), velocity_reference.end())) {
        throw std::runtime_error("Damping curve velocities must be sorted in ascending order");
    }
}

Beam* Damper::getBeam() const {
    return damper_beam;
}

Node* Damper::getInboardNode() const {
    return damper_beam->getInboardNode();
}

Node* Damper::getOutboardNode() const {
    return damper_beam->getOutboardNode();
}

double Damper::getVelocity() const {
    return velocity;
}

const std::vector<double>& Damper::getVelocityReference() const {
    return velocity_reference;
}

const std::vector<double>& Damper::getForceReference() const {
    return force_reference;
}

void Damper::setVelocity(double vel) {
    velocity = vel;
}

double Damper::force() const {
    // Python: np.interp(self.velocity, self.velocity_reference, self.force_reference)
    return interpolate(velocity, velocity_reference, force_reference);
}

double Damper::interpolate(double x,
                          const std::vector<double>& x_data,
                          const std::vector<double>& y_data) const {
    if (x_data.empty() || y_data.empty()) {
        throw std::runtime_error("Cannot interpolate: empty data arrays");
    }
    
    if (x_data.size() != y_data.size()) {
        throw std::runtime_error("Cannot interpolate: x and y data sizes don't match");
    }
    
    // Handle edge cases: x is outside the range
    if (x <= x_data.front()) {
        return y_data.front();
    }
    if (x >= x_data.back()) {
        return y_data.back();
    }
    
    // Find the two points to interpolate between
    // x_data is assumed to be sorted (we check this in constructor)
    auto it = std::lower_bound(x_data.begin(), x_data.end(), x);
    
    if (it == x_data.end()) {
        return y_data.back();
    }
    
    size_t index = std::distance(x_data.begin(), it);
    
    // If x exactly matches a point, return the corresponding y
    if (x == x_data[index]) {
        return y_data[index];
    }
    
    // Linear interpolation between x_data[index-1] and x_data[index]
    if (index == 0) {
        return y_data[0];
    }
    
    double x0 = x_data[index - 1]; 
    double x1 = x_data[index];
    double y0 = y_data[index - 1];
    double y1 = y_data[index];
    
    // Linear interpolation: y = y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    double t = (x - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
}
