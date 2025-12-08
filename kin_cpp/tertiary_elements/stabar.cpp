#include "stabar.h"
#include <algorithm>
#include <cmath>

using namespace blaze;

Stabar::Stabar(Node* left_arm_end,
               Node* right_arm_end,
               Node* left_droplink_end,
               Node* right_droplink_end,
               Node* bar_left_end,
               Node* bar_right_end,
               double _torsional_stiffness) {
    // Torsion bar
    bar = new Beam(bar_left_end, bar_right_end);
    torsional_stiffness = _torsional_stiffness;
    
    // Arms
    left_arm = new Beam(bar_left_end, left_arm_end);
    right_arm = new Beam(bar_right_end, right_arm_end);
    
    // Droplinks
    left_droplink = new Beam(left_droplink_end, left_arm_end);
    right_droplink = new Beam(right_droplink_end, right_arm_end);
    
    // Rotations for tracking
    left_rotation = 0.0;
    right_rotation = 0.0;
}

void Stabar::update() {
    // Reset outboard nodes to initial positions before root finding
    // This ensures we start from a known state
    left_droplink->getOutboardNode()->position = left_droplink->getOutboardNode()->initial_position;
    right_droplink->getOutboardNode()->position = right_droplink->getOutboardNode()->initial_position;
    
    // Find rotation angles that maintain droplink lengths
    // Use same bounds as Python: (-π/2, π/2) and tighter tolerance
    left_rotation = nearest_root(0.0, 
                                 std::make_pair(-M_PI/2, M_PI/2), 1e-10, left_droplink);
    right_rotation = nearest_root(0.0, 
                                  std::make_pair(-M_PI/2, M_PI/2), 1e-10, right_droplink);
    
    // Apply rotations to droplink outboard nodes (matching Python behavior)
    // Python applies rotation AFTER finding it, not during
    StaticVector<double, 3UL> bar_dir = bar->direction();
    Node* bar_inboard = bar->getInboardNode();
    
    // Re-apply left rotation
    Node* left_node = left_droplink->getOutboardNode();
    StaticVector<double, 3UL> left_relative = left_node->initial_position - bar_inboard->initial_position;
    StaticMatrix<double, 3UL, 3UL> left_rot = rotation_matrix(bar_dir, left_rotation);
    left_node->position = left_rot * left_relative + bar_inboard->initial_position;
    
    // Re-apply right rotation
    Node* right_node = right_droplink->getOutboardNode();
    StaticVector<double, 3UL> right_relative = right_node->initial_position - bar_inboard->initial_position;
    StaticMatrix<double, 3UL, 3UL> right_rot = rotation_matrix(bar_dir, right_rotation);
    right_node->position = right_rot * right_relative + bar_inboard->initial_position;
}

double Stabar::_droplink_eqn(double rotation, Beam* droplink) {
    // Calculating transformations manually for runtime (matching Python fast version)
    Node* node = droplink->getOutboardNode();  // This is the arm_end node
    
    // Get rotation matrix about bar direction
    StaticMatrix<double, 3UL, 3UL> rot = rotation_matrix(bar->direction(), rotation);
    
    // Calculate position relative to bar inboard node (use initial positions as Python does)
    StaticVector<double, 3UL> node_initial = node->initial_position;
    StaticVector<double, 3UL> bar_initial = bar->getInboardNode()->initial_position;
    StaticVector<double, 3UL> relative_pos = node_initial - bar_initial;
    
    // Apply rotation
    StaticVector<double, 3UL> rotated_relative = rot * relative_pos;
    
    // Transform back to world coordinates using initial bar position (matching Python)
    node->position = rotated_relative + bar_initial;
    
    // Return residual (length - initial_length)
    return droplink->length() - droplink->initial_length();
}

double Stabar::nearest_root(double x0, 
                            std::pair<double, double> bounds, double tol, Beam* droplink) {
    // Reset outboard node before each root finding attempt
    Node* outboard = droplink->getOutboardNode();
    outboard->position = outboard->initial_position;
    
    // Simple bisection method to find root
    double a = bounds.first;
    double b = bounds.second;
    
    // Ensure function has opposite signs at bounds
    double fa = _droplink_eqn(a, droplink);
    double fb = _droplink_eqn(b, droplink);
    
    if (fa * fb > 0) {
        // No root in interval, try to expand bounds around initial guess
        // Try expanding symmetrically around x0
        double range = 0.1;
        for (int expand = 0; expand < 15; ++expand) {
            a = x0 - range;
            b = x0 + range;
            // Reset before each evaluation
            outboard->position = outboard->initial_position;
            fa = _droplink_eqn(a, droplink);
            outboard->position = outboard->initial_position;
            fb = _droplink_eqn(b, droplink);
            
            if (fa * fb <= 0) {
                break;  // Found interval with opposite signs
            }
            range *= 1.5;  // Expand range more gradually
            if (range > M_PI) break;  // Don't expand beyond reasonable bounds
        }
        
        // If still no sign change, try to find root near x0 using secant-like approach
        if (fa * fb > 0) {
            // Try a few points around x0 to find sign change
            double step = 0.05;
            for (int i = 1; i < 40; ++i) {
                double test_a = x0 - step * i;
                double test_b = x0 + step * i;
                outboard->position = outboard->initial_position;
                double test_fa = _droplink_eqn(test_a, droplink);
                outboard->position = outboard->initial_position;
                double test_fb = _droplink_eqn(test_b, droplink);
                
                if (test_fa * test_fb <= 0) {
                    a = test_a;
                    b = test_b;
                    fa = test_fa;
                    fb = test_fb;
                    break;
                }
            }
        }
        
        // If still no sign change, return best guess (closest to zero residual)
        if (fa * fb > 0) {
            outboard->position = outboard->initial_position;
            double fx0 = _droplink_eqn(x0, droplink);
            // Try a few angles to find the best one
            double best_angle = x0;
            double best_residual = std::abs(fx0);
            for (double angle = -M_PI/2; angle <= M_PI/2; angle += 0.1) {
                outboard->position = outboard->initial_position;
                double res = std::abs(_droplink_eqn(angle, droplink));
                if (res < best_residual) {
                    best_residual = res;
                    best_angle = angle;
                }
            }
            outboard->position = outboard->initial_position;
            _droplink_eqn(best_angle, droplink);  // Set final position
            return best_angle;
        }
    }
    
    // Reset before bisection
    outboard->position = outboard->initial_position;
    
    // Bisection method
    double c = (a + b) / 2.0;
    for (int i = 0; i < 100; ++i) {  // Max 100 iterations
        c = (a + b) / 2.0;
        outboard->position = outboard->initial_position;
        double fc = _droplink_eqn(c, droplink);
        
        // Check convergence - prioritize residual over interval size
        if (std::abs(fc) < tol) {
            return c;
        }
        
        // Also check if interval is very small
        if ((b - a) / 2.0 < tol * 10.0) {
            // If interval is small but residual isn't, try one more refinement
            if (i < 99) {
                // Continue for a few more iterations to improve accuracy
            } else {
                return c;
            }
        }
        
        if (fa * fc < 0) {
            b = c;
            fb = fc;
        } else {
            a = c;
            fa = fc;
        }
    }
    
    return c;
}

double Stabar::rotation() const {
    return left_rotation - right_rotation;
}

double Stabar::torque() const {
    return std::abs(torsional_stiffness * rotation());
}

Beam* Stabar::getBar() const {
    return bar;
}

Beam* Stabar::getLeftArm() const {
    return left_arm;
}

Beam* Stabar::getRightArm() const {
    return right_arm;
}

Beam* Stabar::getLeftDroplink() const {
    return left_droplink;
}

Beam* Stabar::getRightDroplink() const {
    return right_droplink;
}

double Stabar::getLeftRotation() const {
    return left_rotation;
}

double Stabar::getRightRotation() const {
    return right_rotation;
}

double Stabar::getTorsionalStiffness() const {
    return torsional_stiffness;
}

