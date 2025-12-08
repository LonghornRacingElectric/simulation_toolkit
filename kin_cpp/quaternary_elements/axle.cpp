#include "axle.h"
#include <algorithm>
#include <cmath>

using namespace blaze;

Axle::Axle(DoubleWishbone *left_assy, DoubleWishbone *right_assy, CG *cg) {
    left = left_assy;
    right = right_assy;
    this->cg = cg;
}

void Axle::roll(double angle) {
    Node* left_cp = left->getContactPatch();
    Node* right_cp = right->getContactPatch();

    // Calculate lateral positions relative to right contact patch
    double cg_lateral_pos = cg->getPosition()[1] - right_cp->position[1];
    double left_cp_pos = left_cp->position[1] - right_cp->position[1];

    // Calculate lever arms
    double left_arm = std::fabs(left_cp_pos - cg_lateral_pos);
    double right_arm = std::fabs(cg_lateral_pos);  // Distance from CG to right CP
    double LR_ratio = (right_arm > 1e-10) ? left_arm / right_arm : 1.0;
    
    // Initial guess for left jounce
    double left_jounce_guess = left_arm * std::tan(angle);

    // Find the jounce that achieves the target roll angle
    double left_jounce_soln = _find_roll_jounce(angle, LR_ratio, left_jounce_guess);
    
    // Apply jounce to both sides (left goes up, right goes down for positive roll)
    // For positive roll: left roll_jounce is positive (up), right is negative (down)
    left->jounce(0.0, 0.0, left_jounce_soln, 0.0);
    right->jounce(0.0, 0.0, -1 * left_jounce_soln / LR_ratio, 0.0);
}

double Axle::_roll_resid_func(double left_jounce_guess, double angle, double LR_ratio) {
    double right_jounce_guess = -1 * left_jounce_guess / LR_ratio;  // Negative for right side

    Node* left_cp = left->getContactPatch();
    Node* right_cp = right->getContactPatch();

    // Apply jounce to both sides (left up, right down for positive roll)
    left->jounce(0.0, 0.0, left_jounce_guess, 0.0);
    right->jounce(0.0, 0.0, right_jounce_guess, 0.0);

    // Calculate actual track width and roll angle
    double calculated_track = std::fabs(left_cp->position[1] - right_cp->position[1]);
    double left_z = left_cp->position[2];
    double right_z = right_cp->position[2];
    double z_diff = left_z - right_z;
    
    // Roll angle = atan(z_diff / track_width)
    double calculated_roll = std::atan2(z_diff, calculated_track);

    return calculated_roll - angle;
}

double Axle::_find_roll_jounce(double angle, double LR_ratio, double initial_guess) {
    // Simple bisection method to find root
    double a = -0.5;  // Lower bound (reasonable jounce range)
    double b = 0.5;   // Upper bound
    
    // Try initial guess first
    double fa = _roll_resid_func(a, angle, LR_ratio);
    double fb = _roll_resid_func(b, angle, LR_ratio);
    
    // If initial guess is within bounds, use it as starting point
    if (initial_guess >= a && initial_guess <= b) {
        double f_guess = _roll_resid_func(initial_guess, angle, LR_ratio);
        if (std::abs(f_guess) < 1e-6) {
            return initial_guess;
        }
        
        // Adjust bounds based on sign
        if (fa * f_guess < 0) {
            b = initial_guess;
            fb = f_guess;
        } else {
            a = initial_guess;
            fa = f_guess;
        }
    }
    
    // Ensure function has opposite signs at bounds
    if (fa * fb > 0) {
        // Expand bounds if needed
        if (std::abs(fa) < std::abs(fb)) {
            a = initial_guess - 0.1;
            fa = _roll_resid_func(a, angle, LR_ratio);
        } else {
            b = initial_guess + 0.1;
            fb = _roll_resid_func(b, angle, LR_ratio);
        }
    }
    
    // Bisection method
    double c;
    for (int i = 0; i < 100; ++i) {  // Max 100 iterations
        c = (a + b) / 2.0;
        double fc = _roll_resid_func(c, angle, LR_ratio);
        
        if (std::abs(fc) < 1e-6 || (b - a) / 2.0 < 1e-6) {
            return c;
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

void Axle::reset_roll() {
    // Reset both sides to zero jounce
    left->jounce(0, 0, 0, 0);
    right->jounce(0, 0, 0, 0);
}

void Axle::steer(double rack_displacement) {
    // Steer both wheels with same rack displacement
    left->steer(rack_displacement);
    right->steer(rack_displacement);
}

void Axle::axle_heave(double heave) {
    // Apply same heave to both sides
    left->jounce(0.0, heave, 0.0, 0.0);
    right->jounce(0.0, heave, 0.0, 0.0);
}

void Axle::axle_pitch(double pitch) {
    // Apply same pitch to both sides
    left->jounce(0.0, 0.0, 0.0, pitch);
    right->jounce(0.0, 0.0, 0.0, pitch);
}

double Axle::track_width() const {
    Node* left_cp = left->getContactPatch();
    Node* right_cp = right->getContactPatch();
    double track = std::fabs(left_cp->position[1] - right_cp->position[1]);
    return track;
}

double Axle::roll_stiffness() const {
    double left_wheelrate = left->wheelrate();
    double right_wheelrate = right->wheelrate();
    StaticVector<double, 3UL> left_position = left->getContactPatch()->position;
    StaticVector<double, 3UL> right_position = right->getContactPatch()->position;
    StaticVector<double, 3UL> cg_position = cg->getPosition();

    // Roll stiffness = 0.25 * (left_arm^2 * left_wheelrate + right_arm^2 * right_wheelrate)
    double left_arm = left_position[1] - cg_position[1];
    double right_arm = right_position[1] - cg_position[1];
    
    double roll_stiffness = 0.25 * (std::pow(left_arm, 2) * left_wheelrate + 
                                     std::pow(right_arm, 2) * right_wheelrate);

    return roll_stiffness;
}

void Axle::translate(const StaticVector<double, 3UL> &translation) {
    // Translate both DoubleWishbone assemblies and CG
    StaticVector<double, 3UL> trans = translation;  // Create non-const copy
    left->translate(trans);
    right->translate(trans);
    // CG translation is handled through its node
    cg->getNode()->translate(translation);
}

void Axle::flatten_rotate(const StaticVector<double, 3UL> &angle) {
    // Rotate both DoubleWishbone assemblies and CG
    StaticVector<double, 3UL> ang = angle;  // Create non-const copy
    left->flatten_rotate(ang);
    right->flatten_rotate(ang);
    // CG rotation is handled through its node
    cg->getNode()->flatten_rotate(angle);
}




