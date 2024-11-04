#include "full_suspension.h"
using namespace blaze;

FullSuspension::FullSuspension(Axle *front_axle, Axle *rear_axle, CG *cg) {
    this->Fr_axle = front_axle;
    this->Rr_axle = rear_axle;
    this->cg = cg;

    bool transform_origin = true;

    //TODO: Kin_RC
    //TOTO current_avg, prac_avg
    StaticVector<double, 3UL> current_average_cp, prac_average_cp;

    double ang_x, double ang_y = 0;
    Node *elements[3];
    Node *all_elements[2];
}

void steer (double rack_displacement){
    reset_position();
    Fr_axle->steer(rack_displacement);
    //TODO update right and left kin_PC
    if (transform_origin()){
        flatten();
    }
}

void heave (double heave){
    reset_position();
    Fr_axle->axle_heave(heave);
    Rr_axle->axle_heave(heave);
    //TODO update right and left kin_PC
    if (transform_origin()){
        flatten();
    }
}

void pitch (double angle){
    FL_cp = Fr_axle->left->contact_patch;
    RL_cp = Rr_axle->left->contact_patch;

    cg_long_pos = cg->position[0] - RL_cp->position[0];
    front_cp_pos = FL_cp.position[0] - RL_cp.position[0];

    front_arm = fabs(front_cp_pos - cg_long_pos);
    rear-arm = fabs(front_cp_pos - front_arm);

    FR_ratio = front_arm/rear_arm;

    front_heave_guess = front_arm * sin(angle);
    //TODO Implement fsolve() below
    heave_soln = fsolve(_pitch_resid_func, front_heave_guess, args={angle, FR_ratio});

    Fr_axle->axle_pitch(heave_soln[0]);
    Rr_axle->axle_pitch(-1 * heave_soln[0] / FR_ratio);

    // TODO update left and right kin_PC
    left_kin_PC->update();
    right_kin_PC->update();
}

void roll (double angle){
    reset_position();
    Fr_axle->roll(angle);
    Rr_axle->roll(angle);
    
    // TODO left and right kin_PC
    left_kin_PC->update();
    right_kin_PC->update();

    if (transform_origin()){
        flatten();
    }
}