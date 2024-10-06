#ifndef AFRAME_H
#define AFRAME_H

#include <array>
#include "../primary_elements/beam.h"

using namespace std;

class Aframe {
public:
    Aframe (Beam *fore, Beam *aft);
    void rotate (double angle);
    void flatten_rotate (array<double, 3>); //angle : [x_rot, y_rot, z_rot]
    void _set_initial_position ();
    void translate (array<double, 3>); //translation : [x_shift, y_shift, z_shift]
    array<double, 6> plane ();
    array<double, 3> direction_vec ();
private:
    Beam *fore;
    Beam *aft;
    Beam *elements[2];
    Beam *all_elements[2];

    //direction vector
    double direction[3];

    //angle between beams
    double angle;
};

#endif