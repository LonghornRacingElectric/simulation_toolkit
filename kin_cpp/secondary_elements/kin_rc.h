#ifndef KIN_RC_H
#define KIN_RC_H

#include "../primary_elements/beam.h"
#include "cg.h"

class KinRC {
public:
    KinRC (Beam *_left_swing_arm, Beam *_right_swing_arm, CG *_cg);
    void update ();
    StaticVector<double, 3UL> cg_axis_KinRC_pos () const;
    StaticVector<double, 3UL> true_kinRC_pos () const;
    void translate (StaticVector<double, 3UL> &translation);
    void flatten_rotate (StaticVector<double, 3UL> &angle);
private:
    Beam *left_swing_arm;
    Beam *right_swing_arm;
    CG *cg;
    Node *true_KinRC;
    Node *cg_axis_KinRC;
    double lateral_position;
    double vertical_position;

    Node *elements[1];
    Node *all_elements[1];
};

#endif //KIN_RC_H