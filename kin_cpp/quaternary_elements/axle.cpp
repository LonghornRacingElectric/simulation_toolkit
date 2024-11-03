#include "axle.h"
using namespace blaze;

Axle::Axle (DoubleWishbone *left_assy, DoubleWishbone *right_assy, CG *cg) {
    left = left_assy;
    right = right_assy;
    this->cg = cg;

    elements[0] = all_elements[0] = left;
    elements[1] = all_elements[1] = right;
}

void Axle::roll(double angle){
    Node* left_cp = left->contact_patch;
    Node* right_cp = right->contact_patch;

    double cg_lateral_pos = cg->position[1] - right_cp->position[1];
    double left_cp_pos = left_cp->position[1] - right_cp->position[1];

    double left_arm = std::abs(left_cp_pos - cg_lateral_pos);
    double right_arm = std::abs(left_cp_pos - left_arm);
    double LR_ration = left_arm/right_arm;

    double left_jounce_guess = left_arm * std::tan(angle);

    //TODO: implement solve function for solve_root

    // double jounce_soln = solve_root(left_jounce_guess, angle, LR_ratio, cg);
    // left->jounce(0.0, 0.0, -1 * jounce_soln, 0.0);
    // right->jounce(0.0, 0.0, jounce_soln / LR_ratio, 0.0);

    kin_rc->update();

}

// Jounce = (_jounce, _heave_jounce, _roll_jounce, _pitch_jounce)
double Axle::_roll_resid_func(StaticVector<double, 2UL> args, StaticVector<double, 2UL> x){
    double angle = args[0];
    double LR_ration = args[1];

    double left_jounce_guess = x[0];
    double right_jounce_guess = x[0] / LR_ratio;

    Node* left_cp = left->contact_patch;
    Node* right_cp = right->contact_patch;

    // left.jounce(roll_jounce = -1*left_jounce_guess)
    // right.jounce(roll_jounce = right_jounce_guess)
    left->jounce(0.0, 0.0, -1 * left_jounce_guess, 0.0);
    right->jounce(0.0, 0.0, right_jounce_guess, 0.0);

    double calculated_track = abs(left.cp->posision[1] - right_cp->position[1]);
    double calculated_roll = atan((left_jounce_guess + right_jounce_guess) / calculated_track);

    return StaticVector<double, 1UL>{calculated_roll - angle};
}

void Axle::reset_roll(){
    // jounce = 0
    axle.jounce(0.0, 0.0, 0.0, 0.0);
}

void Axle::steer(double rack_displacement){
    left->steer(rack_displacement);
    right->steer(rack_displacement);

    kin_RC->update();
}

void axle_heave (double heave){
    // heave_jounce = jounce
    left->jounce(0.0, heave, 0.0, 0.0);
    right->jounce(0.0, heave, 0.0, 0.0);

    kin_RC->update();
}

void axle_pitch (double heave){
    //pitch_jounce = jounce
    left->jounce(0.0, 0.0, 0.0, jounce);
    right->jounce(0.0, 0.0, 0.0, jounce);

    rin_RC->update();
}

double track_width () const{
    double track = abs(left->contact_patch->position[1] - right->contact_patch->position[1]);
    return track;
}

double roll_stiffness() const{
    double left_wheelrate = left->wheelrate;
    double right_wheelrate = right->wheelrate;
    double left_position = left->contact_patch->position;
    double right_position = right->contact_patch->position;
    double cg_position = cg->position;

    double roll_stiffness = 0.25 * ((left_position[1] - cg_position[1])**2 * left_wheelrate + (right_position[1] - cg_position[1])**2 * right_wheelrate);

    return roll_stiffness;
}

void translate (StaticVector<double, 3UL> translation){
    //TODO
}





