#include "push_pull_rod.h"
#include "../assets/misc_linalg.h"
#include <cmath>

using namespace blaze;

PushPullRod::PushPullRod (Node *inboard, Node *outboard, bool upper, bool bellcrank, Node *bc_pivot, StaticVector<double, 3UL> &bc_direction, Node *shock_outboard, Node *shock_inboard) {
    rod = new Beam(inboard, outboard);
    initial_rod_length = rod_length ();
    bellcrank_angle = 0.0;
    wishbone_angle = 0.0;
    bc_exists = bellcrank;

    if (bc_exists) {
        bellcrank_pivot = bc_pivot;
        bellcrank_direction = bc_direction;
        // Create a simple linear damping curve for the damper
        // Default: linear damping with 1:1 ratio (velocity -> force)
        std::vector<std::pair<double, double>> damping_curve = {
            {0.0, 0.0},
            {1.0, 1.0},
            {2.0, 2.0}
        };
        damper = new Damper (shock_inboard, shock_outboard, damping_curve);
        initial_spring_damper_length = spring_damper_length ();
    } else {
        damper = nullptr;
        bellcrank_pivot = nullptr;
    }

    if (bc_exists) {
        elements[0] = all_elements[0] = (Node *) rod;
        elements[1] = all_elements[1] = bellcrank_pivot;
        elements[2] = all_elements[2] = (Node *) damper;
    } else {
        elements[0] = all_elements[0] = (Node *) rod;
    }
}

void PushPullRod::rotate_rod (StaticVector<double, 3UL> &axis, Node *origin, double angle) {
    _set_initial_position ();
    StaticMatrix<double, 3UL, 3UL> rot_mat = rotation_matrix (axis, angle);
    StaticVector<double, 3UL> translated_point = rod->getOutboardNode ()->position - origin->position;
    rod->getOutboardNode ()->position = (rot_mat * translated_point) + origin->position;
}
void PushPullRod::rotate_bellcrank (double angle) {
    if (!bc_exists || !damper) {
        return;
    }
    bellcrank_angle = angle;  // Update the bellcrank angle
    StaticMatrix<double, 3UL, 3UL> bellcrank_rot_mat = rotation_matrix (bellcrank_direction, angle);
    /* Rotate rod about bellcrank */
    StaticVector<double, 3UL> rod_rotated_point = rod->getInboardNode ()->position - bellcrank_pivot->position;
    rod->getInboardNode ()->position = (bellcrank_rot_mat * rod_rotated_point) + bellcrank_pivot->position;

    /* Rotate spring / damper about bellcrank */
    StaticVector<double, 3UL> spring_damper_rotated_point = damper->getBeam ()->getOutboardNode ()->position - bellcrank_pivot->position;
    damper->getBeam ()->getOutboardNode ()->position = bellcrank_rot_mat * spring_damper_rotated_point + bellcrank_pivot->position;
}

double PushPullRod::rod_length () const {
    StaticVector<double, 3UL> diff = rod->getInboardNode ()->position - rod->getOutboardNode ()->position;
    return std::sqrt(diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2]);
}

double PushPullRod::spring_damper_length () const {
    if (!bc_exists || !damper) {
        return 0.0;
    }
    StaticVector<double, 3UL> diff = damper->getBeam ()->getInboardNode ()->position - damper->getBeam ()->getOutboardNode ()->position;
    return std::sqrt(diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2]);
}

void PushPullRod::update () {
    /* NEED FSOLVE FOR FOLLOWING : 
       bellcrank_angle = fsolve(self._bellcrank_resid_func, [0])
        
        self.rotate_bellcrank(angle=bellcrank_angle[0]) */
}

double PushPullRod::_bellcrank_resid_func (double bellcrank_rotation) {
    bellcrank_angle = bellcrank_rotation;
    rotate_bellcrank (bellcrank_rotation);
    
   double residual = initial_rod_length - rod_length ();
    _reset_bellcrank_position ();
    return residual;
}

void PushPullRod::_reset_bellcrank_position () {
    if (bc_exists && damper) {
        rod->getInboardNode ()->reset ();
        damper->getBeam ()->getOutboardNode ()->reset ();
    }
}

void PushPullRod::_set_initial_position () {
    rod->getOutboardNode ()->reset ();
    _reset_bellcrank_position ();
}

void PushPullRod::translate (StaticVector<double, 3UL> &translation) {
    rod->translate (translation);

    if (bc_exists) {
        bellcrank_pivot->translate (translation);
        damper->getBeam ()->translate (translation);
    }
}
void PushPullRod::flatten_rotate(StaticVector<double, 3UL> &angle) {
    rod->flatten_rotate (angle);

    if (bc_exists) {
        bellcrank_pivot->flatten_rotate (angle);
        damper->getBeam ()->flatten_rotate (angle);
    }
}

Beam* PushPullRod::getRod() const {
    return rod;
}

Damper* PushPullRod::getDamper() const {
    return damper;
}

double PushPullRod::getBellcrankAngle() const {
    return bellcrank_angle;
}

double PushPullRod::getInitialRodLength() const {
    return initial_rod_length;
}

double PushPullRod::getInitialSpringDamperLength() const {
    return initial_spring_damper_length;
}

bool PushPullRod::hasBellcrank() const {
    return bc_exists;
}

