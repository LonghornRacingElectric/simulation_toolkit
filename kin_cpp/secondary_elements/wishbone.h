#ifndef WISHBONE_H
#define WISHBONE_H

#include <array>
#include "../primary_elements/beam.h"

using namespace std;

class Wishbone {
public:
    Wishbone (Beam *fore, Beam *aft);
    void rotate (double angle);
    void flatten_rotate (array<double, 3>); //angle : [x_rot, y_rot, z_rot]
    void _set_initial_position (void);
    void translate (array<double, 3>); //translation : [x_shift, y_shift, z_shift]
    array<double, 6> plane () const;
    array<double, 3> direction_vec () const;

    //getters
    Beam *getForeBeam ();
    Beam *getAftBeam ();
private:
    Beam *fore;
    Beam *aft;
    Beam *elements[2];
    Node *all_elements[3];

    //direction vector
    double direction[3];

    //angle between beams
    double angle;
};

#endif