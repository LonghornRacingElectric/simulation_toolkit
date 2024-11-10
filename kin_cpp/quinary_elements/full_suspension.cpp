#include "full_suspension.h"
using namespace blaze;

FullSuspension::FullSuspension(Axle *front_axle, Axle *rear_axle, CG *cg) {
    this->Fr_axle = front_axle;
    this->Rr_axle = rear_axle;
    this->cg = cg;

    bool transform_origin = true;

    left_kin_pc = new KinPC (Fr_axle->getLeft ()->getSVICLink (), Rr_axle->getLeft ()->getSVICLink (), cg);
    right_kin_pc = new KinPC (Fr_axle->getRight ()->getSVICLink (), Rr_axle->getRight ()->getSVICLink (), cg);
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
    
    left_kin_pc->update ();
    right_kin_pc->update ();

    if (transform_origin) {
        flatten();
    }
}

void FullSuspension::heave (double heave){
    reset_position();
    Fr_axle->axle_heave(heave);
    Rr_axle->axle_heave(heave);
    
    left_kin_PC->update ();
    right_kin_PC->update ();

    if (transform_origin) {
        flatten();
    }
}

void FullSuspension::pitch (double angle){
    Node *FL_cp = Fr_axle->left->contact_patch;
    Node *RL_cp = Rr_axle->left->contact_patch;

    double cg_long_pos = cg->position[0] - RL_cp->position[0];
    double front_cp_pos = FL_cp.position[0] - RL_cp.position[0];

    double front_arm = fabs(front_cp_pos - cg_long_pos);
    double rear_arm = fabs(front_cp_pos - front_arm);

    double FR_ratio = front_arm/rear_arm;

    double front_heave_guess = front_arm * sin(angle);
    //TODO Implement fsolve() below
    double heave_soln = fsolve(_pitch_resid_func, front_heave_guess, args={angle, FR_ratio});

    Fr_axle->axle_pitch(heave_soln);
    Rr_axle->axle_pitch(-1 * heave_soln / FR_ratio);

    left_kin_PC->update();
    right_kin_PC->update();

    if (transform_origin) {
        flatten ();
    }
}

void FullSuspension::roll (double angle){
    reset_position();
    Fr_axle->roll(angle);
    Rr_axle->roll(angle);
    
    left_kin_PC->update();
    right_kin_PC->update();

    if (transform_origin()){
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

    double front_heave_guess = x[0];
    double rear_heave_guess = x[0] / FR_ratio;

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
