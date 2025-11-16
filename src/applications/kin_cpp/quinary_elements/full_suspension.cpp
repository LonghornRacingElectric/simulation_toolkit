#include "full_suspension.h"
#include "../assets/misc_linalg.h"
using namespace blaze;

FullSuspension::FullSuspension(Axle *front_axle, Axle *rear_axle, CG *cg) {
    this->Fr_axle = front_axle;
    this->Rr_axle = rear_axle;
    this->cg = cg;

    bool transform_origin = true;

    current_average_cp = 
        (Fr_axle->getLeft ()->getContactPatch ()->position + 
        Fr_axle->getRight ()->getContactPatch ()->position + 
        Rr_axle->getLeft ()->getContactPatch ()->position + 
        Rr_axle->getRight ()->getContactPatch ()->position) / 4;
    prac_average_cp = current_average_cp;
    ang_x = ang_y = 0;

    elements[0] = all_elements[0] = Fr_axle;
    elements[1] = all_elements[1] = Rr_axle;
    elements[2] = (Axle *) cg;
}

void FullSuspension::steer (double rack_displacement){
    reset_position();
    Fr_axle->steer(rack_displacement);
    
    if (transform_origin) {
        flatten();
    }
}

void FullSuspension::heave (double heave){
    reset_position();
    Fr_axle->axle_heave(heave);
    Rr_axle->axle_heave(heave);
    

    if (transform_origin) {
        flatten();
    }
}

void FullSuspension::pitch (double angle){
    Node *FL_cp = Fr_axle->getLeft ()->getContactPatch ();
    Node *RL_cp = Rr_axle->getLeft ()->getContactPatch ();

    double cg_long_pos = cg->getPosition ()->position[0] - RL_cp->position[0];
    double front_cp_pos = FL_cp->position[0] - RL_cp->position[0];

    double front_arm = fabs(front_cp_pos - cg_long_pos);
    double rear_arm = fabs(front_cp_pos - front_arm);

    double FR_ratio = front_arm/rear_arm;

    double front_heave_guess = front_arm * sin(angle);
    //TODO Implement fsolve() below
    double heave_soln = fsolve(_pitch_resid_func, front_heave_guess, {angle, FR_ratio});

    Fr_axle->axle_pitch(heave_soln);
    Rr_axle->axle_pitch(-1 * heave_soln / FR_ratio);

    if (transform_origin) {
        flatten ();
    }
}

void FullSuspension::roll (double angle){
    reset_position();
    Fr_axle->roll(angle);
    Rr_axle->roll(angle);
    
    if (transform_origin){
        flatten();
    }
}

double FullSuspension::left_wheelbase () const {
    double left_wheelbase = abs (Fr_axle->getLeft ()->getContactPatch ()->position[0] -
                                 Rr_axle->getLeft ()->getContactPatch ()->position[0]);
    return left_wheelbase;
}

double FullSuspension::right_wheelbase () const {
    double right_wheelbase = abs (Fr_axle->getRight ()->getContactPatch ()->position[0] -
                                 Rr_axle->getRight ()->getContactPatch ()->position[0]);
    return right_wheelbase;
}

double FullSuspension::_pitch_resid_func (double x, StaticVector<double, 2UL> args) {
    double angle = args[0];
    double FR_ratio = args[1];

    double front_heave_guess = x;
    double rear_heave_guess = x / FR_ratio;

    Node *FL_cp = Fr_axle->getLeft ()->getContactPatch ();
    Node *RL_cp = Rr_axle->getLeft ()->getContactPatch ();

    Fr_axle->axle_pitch (front_heave_guess);
    Rr_axle->axle_pitch (-1 * front_heave_guess / FR_ratio);

    double calculated_wheelbase = abs (FL_cp->position[0] - RL_cp->position[0]);
    double calculated_pitch = atan ((front_heave_guess + rear_heave_guess) / calculated_wheelbase);

    return calculated_pitch - angle;
}

void FullSuspension::_update_FAP () {
    DoubleWishbone *Fr_left = Fr_axle->getLeft ();
    DoubleWishbone *Fr_right = Fr_axle->getRight ();
    DoubleWishbone *Rr_left = Rr_axle->getLeft ();
    DoubleWishbone *Rr_right = Rr_axle->getRight ();

    Fr_left->getFVFAP ()->position = Fr_left->FV_FAP_position ();
    Fr_left->getSVFAP ()->position = Fr_left->SV_FAP_position ();

    Fr_right->getFVFAP ()->position = Fr_right->FV_FAP_position ();
    Fr_right->getSVFAP ()->position = Fr_right->SV_FAP_position ();

    Rr_left->getFVFAP ()->position = Rr_left->FV_FAP_position ();
    Rr_left->getSVFAP ()->position = Rr_left->SV_FAP_position ();

    Rr_right->getFVFAP ()->position = Rr_right->FV_FAP_position ();
    Rr_right->getSVFAP ()->position = Rr_right->SV_FAP_position ();
}

double FullSuspension::heave_stiffness () const {
    double FL_wheelrate = Fr_axle->getLeft ()->wheelrate ();
    double FR_wheelrate = Fr_axle->getRight ()->wheelrate ();
    double RL_wheelrate = Rr_axle->getLeft ()->wheelrate ();
    double RR_wheelrate = Rr_axle->getRight ()->wheelrate ();

    return FL_wheelrate + FR_wheelrate + RL_wheelrate + RR_wheelrate;
}

double FullSuspension::roll_stiffness () const {
    double Fr_axle_rate = Fr_axle->roll_stiffness ();
    double Rr_axle_rate = Rr_axle->roll_stiffness ();

    return Fr_axle_rate + Rr_axle_rate;
}

double FullSuspension::pitch_stiffness () const {
    double Fr_axle_rate = Fr_axle->getLeft()->wheelrate () + Fr_axle->getRight()->wheelrate ();
    double Rr_axle_rate = Rr_axle->getLeft()->wheelrate () + Rr_axle->getRight()->wheelrate ();
    double Fr_position = Fr_axle->getLeft()->getContactPatch ()->position[0];
    double Rr_position = Rr_axle->getLeft()->getContactPatch ()->position[0];
    double cg_position = cg->getPosition ()->position[0];

    double pitch_stiffness = 1/4 * (pow((Fr_position - cg_position), 2) * Fr_axle_rate + pow((Rr_position - cg_position), 2) * Rr_axle_rate);
    return pitch_stiffness;
}

void FullSuspension::reset_position () {
    translate(-1 * prac_average_cp);
    flatten_rotate({ang_x, -1 * ang_y, 0});
    translate(current_average_cp);
}

void FullSuspension::hard_reset () {
    steer(0.000001);
    heave(0.000001);
    pitch(0.000001);
    roll(0.000001);

    *this = FullSuspension (this->Fr_axle, this->Rr_axle, this->cg);
}

void FullSuspension::flatten () {
    Node * FL_cp = Fr_axle->getLeft()->getContactPatch ();
    Node * FR_cp = Fr_axle->getRight ()->getContactPatch ();
    Node * RL_cp = Rr_axle->getLeft()->getContactPatch ();
    Node * RR_cp = Rr_axle->getRight ()->getContactPatch ();

    StaticVector<double, 3UL> average_cp = (FL_cp->position + FR_cp->position + RL_cp->position + RR_cp->position) / 4;

    current_average_cp = average_cp;
    translate(-1 * average_cp);

    /* Parameter for plane */
    StaticMatrix<double, 3UL, 3UL> cp_plane_params;
    column (cp_plane_params, 0) = FL_cp->position;
    column (cp_plane_params, 1) = FR_cp->position;
    column (cp_plane_params, 2) = RL_cp->position;

    StaticVector<double, 6UL> cp_plane = plane (cp_plane_params);
    double a = cp_plane[0], b = cp_plane[1], c = cp_plane[2];
    double x_0 = cp_plane[3], y_0 = cp_plane[4], z_0 = cp_plane[5];
    auto plane_eqn = [a, b, c, x_0, y_0, z_0](int a1, int a2) -> double {
        ((a * x_0 + b * y_0 + c * z_0) - a * a1 - b * a2) / c; }; 

    double ang_x = atan(plane_eqn(0,1));
    double ang_y = atan(plane_eqn(1,0));

    if (transform_origin) {
        this->ang_x = ang_x;
        this->ang_y = ang_y;
        flatten_rotate({-ang_x, ang_y, 0.0});
    }

    prac_average_cp = {current_average_cp[0], current_average_cp[1], 0.0};
    translate(prac_average_cp);
}

void FullSuspension::translate(StaticVector<double, 3UL> translation) {
    Fr_axle->translate (translation);
    Rr_axle->translate (translation);
    cg->getPosition ()->translate (translation);
}

void FullSuspension::flatten_rotate (StaticVector<double, 3UL> angle) {
    Fr_axle->flatten_rotate (angle);
    Rr_axle->flatten_rotate (angle);
    cg->getPosition ()->flatten_rotate (angle);
}
  

