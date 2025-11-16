#include "kin_pc.h"

KinPC::KinPC(Beam *_front_swing_arm, Beam *_rear_swing_arm, CG *_cg) {
    front_swing_arm = _front_swing_arm;
    rear_swing_arm = _rear_swing_arm;
    cg = _cg;

    /* Actual kinematic roll center */
    true_KinPC = new Node (true_KinPC_pos ());
    long_position = true_KinPC->position[0];
    vertical_position = true_KinPC->position[2];

    cg_axis_KinPC = new Node (cg_axis_KinPC_pos ());

    elements[0] = all_elements[0] = true_KinPC;
}

void KinPC::update() {
    true_KinPC->position = true_KinPC_pos ();
    long_position = true_KinPC->position[0];
    vertical_position = true_KinPC->position[2];
    cg_axis_KinPC->position = cg_axis_KinPC_pos ();
}

StaticVector<double, 3UL> KinPC::cg_axis_KinPC_pos () const {
    StaticVector<double, 3UL> &kpc_pos = true_KinPC->position;
    StaticVector<double, 3UL> &cg_pos = cg->getPosition ()->position;
    const StaticVector<double, 3UL> &cg_dir = cg->getDirection ();
    
    double x = (kpc_pos[2] - cg_pos[2]) * cg_dir[0] / cg_dir[2] + cg_pos[0];
    return {x, kpc_pos[1], kpc_pos[2]};
}

StaticVector<double, 3UL> KinPC::true_KinPC_pos () const {
    return front_swing_arm->xz_intersection (rear_swing_arm);
}

void KinPC::translate(const StaticVector<double, 3UL>& translation) {
    true_KinPC->translate (translation);
}

void KinPC::flatten_rotate(const StaticVector<double, 3UL>& angle) {
    true_KinPC->flatten_rotate (angle);
}