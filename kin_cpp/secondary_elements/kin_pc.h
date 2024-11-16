#ifndef KIN_PC_H
#define KIN_PC_H

#include "../primary_elements/beam.h"
#include "cg.h"

class KinPC {
public:
    KinPC(Link* front_swing_arm, Link* rear_swing_arm, CG* cg);

    void update();
    StaticVector<double, 2> getCGAxisKinPCPos() const;
    StaticVector<double, 2> getTrueKinPCPos() const;
    void translate(const StaticVector<double, 3UL>& translate);
    void flatten_rotate(const StaticVector<double, 3UL>& angle);
private:
    Beam* front_swing_arm;
    Beam* rear_swing_arm;
    CG* cg;

    Node* true_KinPC;
    Node* cg_axis_KinPC;

    double long_position;
    double vertical_position;

    Beam *elements[2];
    Node *all_elements[3];
    StaticVector<double, 3> true_KinPC_pos;
    StaticVector<double, 3> cg_axis_KinPC_pos;
};

#endif //KIN_PC_H