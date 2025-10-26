#include "axle.h"
using namespace blaze;

Axle::Axle (DoubleWishbone *left_assy, DoubleWishbone *right_assy, CG *cg) {
    this->left = left_assy;
    this->right = right_assy;
    this->cg = cg;

    elements[0] = all_elements[0] = left;
    elements[1] = all_elements[1] = right;
    elements[2] = all_elements[2] = (DoubleWishbone *) cg;
}

void Axle::roll(double angle){
    Node* left_cp = left->getContactPatch ();
    Node* right_cp = right->getContactPatch ();

    double cg_lateral_pos = cg->getPosition ()->position[1] - right_cp->position[1];
    double left_cp_pos = left_cp->position[1] - right_cp->position[1];

    double left_arm = std::fabs(left_cp_pos - cg_lateral_pos);
    double right_arm = std::fabs(left_cp_pos - left_arm);
    double LR_ratio = left_arm/right_arm;
    double left_jounce_guess = left_arm * std::tan(angle);

    //TODO NEED FSOLVE FOR 

    // double jounce_soln = fsolve(left_jounce_guess, angle, LR_ratio, cg);
    // left->jounce(0.0, 0.0, -1 * jounce_soln, 0.0);
    // right->jounce(0.0, 0.0, jounce_soln / LR_ratio, 0.0);

}

// Jounce = (_jounce, _heave_jounce, _roll_jounce, _pitch_jounce)
double Axle::_roll_resid_func(StaticVector<double, 2UL> &args, double x) {
    double angle = args[0];
    double LR_ratio = args[1];

    double left_jounce_guess = x;
    double right_jounce_guess = x / LR_ratio;

    Node* left_cp = left->getContactPatch ();
    Node* right_cp = right->getContactPatch ();

    // left.jounce(roll_jounce = -1*left_jounce_guess)
    // right.jounce(roll_jounce = right_jounce_guess)
    left->jounce(0.0, 0.0, -1 * left_jounce_guess, 0.0);
    right->jounce(0.0, 0.0, right_jounce_guess, 0.0);

    double calculated_track = fabs(left_cp->position[1] - right_cp->position[1]);
    double calculated_roll = atan((left_jounce_guess + right_jounce_guess) / calculated_track);

    return calculated_roll - angle;
}

void Axle::reset_roll(){
    // jounce = 0
    left->jounce (0, 0, 0, 0);
    right->jounce (0, 0, 0, 0);
}

void Axle::steer(double rack_displacement){
    left->steer(rack_displacement);
    right->steer(rack_displacement);
}

void Axle::axle_heave (double heave){
    // heave_jounce = jounce
    left->jounce(0.0, heave, 0.0, 0.0);
    right->jounce(0.0, heave, 0.0, 0.0);
}

void Axle::axle_pitch (double heave){
    //pitch_jounce = jounce
    left->jounce(0.0, 0.0, 0.0, heave);
    right->jounce(0.0, 0.0, 0.0, heave);
}

double Axle::track_width () const{
    double track = fabs(left->getContactPatch ()->position[1] - right->getContactPatch ()->position[1]);
    return track;
}

double Axle::roll_stiffness() const{
    double left_wheelrate = left->wheelrate ();
    double right_wheelrate = right->wheelrate ();
    StaticVector<double, 3UL> left_position = left->getContactPatch ()->position;
    StaticVector<double, 3UL> right_position = right->getContactPatch ()->position;
    StaticVector<double, 3UL> cg_position = cg->getPosition ()->position;

    double roll_stiffness = 0.25 * (pow((left_position[1] - cg_position[1]), 2) * left_wheelrate + pow((right_position[1] - cg_position[1]), 2) * right_wheelrate);

    return roll_stiffness;
}

void Axle::translate (StaticVector<double, 3UL> &translation){
    int array_size = sizeof(all_elements) / sizeof(all_elements[0]);
    for (int i = 0; i < array_size; i++) {
        all_elements[i]->translate(translation);
    }
}

void Axle::flatten_rotate (StaticVector<double, 3UL> &angle) {
    int array_size = sizeof(all_elements) / sizeof(all_elements[0]);
    for (int i = 0; i < array_size; i++) {
        all_elements[i]->flatten_rotate(angle);
    }
}




