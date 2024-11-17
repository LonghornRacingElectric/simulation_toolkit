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
    
    left_kin_pc->update ();
    right_kin_pc->update ();

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

    left_kin_pc->update();
    right_kin_pc->update();

    if (transform_origin) {
        flatten ();
    }
}

void FullSuspension::roll (double angle){
    reset_position();
    Fr_axle->roll(angle);
    Rr_axle->roll(angle);
    
    left_kin_pc->update();
    right_kin_pc->update();

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

double FullSuspension::roll_stiffness () const {
    double Fr_axle_rate = Fr_axle->roll_stiffness;
    double Rr_axle_rate = Rr_axle->roll_stiffness;

    return Fr_axle_rate + Rr_axle_rate;
}

double FullSuspension::pitch_stiffness () const {
    double Fr_axle_rate = Fr_axle->left->wheelrate + Fr_axle->right->wheelrate;
    double Rr_axle_rate = Rr_axle->left->wheelrate + Rr_axle->right->wheelrate;
    double Fr_position = Fr_axle->left->contact_patch.->position[0];
    double Rr_position = Rr_axle->left->contact_patch.->position[0];
    double cg_position = cg->position[0];

    double pitch_stiffness = 1/4 * ((Fr_position - cg_position)**2 * Fr_axle_rate + (Rr_position - cg_position)**2 * Rr_axle_rate);
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
    vector<double> FL_cp = Fr_axle->left->position;
    vector<double> FR_cp = Fr_axle->right->position;
    vector<double> RL_cp = Rr_axle->left->position;
    vector<double> RR_cp = Rr_axle->right->position;

    vector<double> average_cp = {(FL_cp[0] + FR_cp[0] + RL_cp[0] + RR_cp[0]) / 4,
                                (FL_cp[1] + FR_cp[1] + RL_cp[1] + RR_cp[1]) / 4,
                                (FL_cp[2] + FR_cp[2] + RL_cp[2] + RR_cp[2]) / 4};

    this-> current_average_cp = average_cp;
    translate(-1*average_cp);
    auto plane_enq = 

    auto plane_coeffs = plane({FL_cp, FR_cp, RL_cp});
        double a = plane_coeffs[0], b = plane_coeffs[1], c = plane_coeffs[2];
        double x_0 = plane_coeffs[3], y_0 = plane_coeffs[4], z_0 = plane_coeffs[5];

    auto plane_eqn = [&](double x, double y) -> double {
            return ((a * x_0 + b * y_0 + c * z_0) - a * x - b * y) / c;
        };

    ang_x = atan(plane_eqn(0,1));
    ang_y = atan(plane_eqn(1,0));

    if (transform_origin) {
        flatten_rotate({-ang_x, ang_y, 0.0});
    }

    prac_average_cp = {current_average_cp[0], current_average_cp[1], 0.0};
    translate(prac_average_cp);
}

vector<double, 6> FullSuspension::plane (vector<vector<double>> points) {
    assert(points.size() == 3 && "Plane generator only accepts 3 points");
        vector<double> PQ = {points[1][0] - points[0][0], points[1][1] - points[0][1], points[1][2] - points[0][2]};
        vector<double> PR = {points[2][0] - points[0][0], points[2][1] - points[0][1], points[2][2] - points[0][2]};
        
        // Calculates cross product or PQ and PR
        vector<double, 3> normal;
        normal[0] = PQ[1] * PR[2] - PQ[2] * PR[1]; // i-component
        normal[1] = PQ[2] * PR[0] - PQ[0] * PR[2]; // j-component
        normal[2] = PQ[0] * PR[1] - PQ[1] * PR[0]; // k-component

        double a = normal[0];
        double b = normal[1];
        double c = normal[2];

        double x_0 = points[0][0];
        double y_0 = points[0][1];
        double z_0 = points[0][2];

        return {a, b, c, x_0, y_0, z_0};
}

void FullSuspension::translate(vector<double> translation) {
    for (int i=0; i<all_elements.size(); i++) {
        all_elements[i].translate(translation);
    }
}

void FullSuspension::flatten_rotate (vector<double> angle) {
    for (int i=0; i<all_elements.size(); i++) {
        all_elements[i].flatten_rotate(angle);
    }
}

//TODO
void FullSuspension::plot_elements() {

}
       

