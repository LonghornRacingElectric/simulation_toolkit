#ifndef WISHBONE_H
#define WISHBONE_H

#include <array>
#include "../primary_elements/beam.h"

using namespace blaze;

class Wishbone {
public:
    Wishbone (Beam *fore, Beam *aft);
    void rotate (double angle);
    void flatten_rotate (const StaticVector<double, 3> &); //angle : [x_rot, y_rot, z_rot]
    void set_initial_position ();
    void translate (const StaticVector<double, 3> &); //translation : [x_shift, y_shift, z_shift]
    StaticVector<double, 6> plane () const;
    StaticVector<double, 3> direction_vec () const;

    //getters
    Beam *getForeBeam ();
    Beam *getAftBeam ();
    double getAngle ();
    StaticVector<double, 3> &getDirection ();
private:
    Beam *fore;
    Beam *aft;
    Beam *elements[2];
    Node *all_elements[4];  // fore inboard, fore outboard, aft inboard, aft outboard

    //direction vector
    StaticVector<double, 3> direction;

    //angle between beams
    double angle;
};

#endif