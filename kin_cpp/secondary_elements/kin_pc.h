#ifndef KIN_PC_H
#define KIN_PC_H

#include "../primary_elements/beam.h"
#include "cg.h"

class KinPC {
public:
    KinPC(Beam *_front_swing_arm, Beam *_rear_swing_arm, CG* _cg);

    void update();
    StaticVector<double, 3UL> cg_axis_KinPC_pos() const;
    StaticVector<double, 3UL> true_KinPC_pos() const;
    void translate(const StaticVector<double, 3UL>& translation);
    void flatten_rotate(const StaticVector<double, 3UL>& angle);
private:
    Beam *front_swing_arm;
    Beam *rear_swing_arm;
    CG* cg;

    Node* true_KinPC;
    Node* cg_axis_KinPC;

    double long_position;
    double vertical_position;

    Node *elements[1];
    Node *all_elements[1];
};

#endif //KIN_PC_H