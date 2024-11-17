#include "kin_rc.h"

KinRC::KinRC (Beam *_left_swing_arm, Beam *_right_swing_arm, CG *_cg) {
    left_swing_arm = _left_swing_arm;
    right_swing_arm = _right_swing_arm;
    cg = _cg;

    /* Actual Kinematic Roll Center */
    true_KinRC = new Node (true_kinRC_pos ());
    lateral_position = true_KinRC->position[1];
    vertical_position = true_KinRC->position[2];

    cg_axis_KinRC = new Node (cg_axis_KinRC_pos ());

    elements[0] = all_elements[0] = true_KinRC;
}

void KinRC::update () {
    /* Reassign KinRC position to agree with rest of system */
    true_KinRC->position = true_kinRC_pos ();
    lateral_position = true_KinRC->position[1];
    vertical_position = true_KinRC->position[2];
    cg_axis_KinRC->position = cg_axis_KinRC_pos ();
}

StaticVector<double, 3UL> KinRC::cg_axis_KinRC_pos () const {
    StaticVector<double, 3UL> &krc_pos = true_KinRC->position;
    StaticVector<double, 3UL> &cg_pos = cg->getPosition ()->position;
    const StaticVector<double, 3UL> &cg_dir = cg->getDirection ();
    double y = (krc_pos[2] - cg_pos[2]) * cg_dir[1] / cg_dir[2] + cg_pos[1];
    return {krc_pos[0], y, krc_pos[2]};
}

StaticVector<double, 3UL> KinRC::true_kinRC_pos () const {
    return left_swing_arm->yz_intersection (right_swing_arm);
}

void KinRC::translate (StaticVector<double, 3UL> &translation) {
    true_KinRC->translate (translation);
}

void KinRC::flatten_rotate (StaticVector<double, 3UL> &angle) {
    true_KinRC->flatten_rotate (angle);
}