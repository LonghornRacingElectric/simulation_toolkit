#ifndef AXLE_H
#define AXLE_H

#include "../tertiary_elements/double_wishbone.h"

class Axle {
public:
    Axle (DoubleWishbone *left, DoubleWishbone *right, CG *cg);
    void roll (double angle);
    double _roll_resid_func (StaticVector<double, 2UL> args);
    void reset_roll ();
    void steer (double rack_displacement);
    void axle_heave (double heave);
    void axle_pitch (double heave);
    double track_width () const;
    double roll_stiffness () const;
    void translate (StaticVector<double, 3UL> translation);
    void flatten_rotate (StaticVector<double, 3UL> angle);
private:
    DoubleWishbone *left;
    DoubleWIshbone *right
    CG *cg;

    Node *elements[2];
    Node *all_elements[2];
};

#endif //AXLE_H