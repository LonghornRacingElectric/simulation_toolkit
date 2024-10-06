#ifndef BEAM_H
#define BEAM_H

#include <array>
#include "node.h"

class Beam {
public:
    Beam(Node *inboard, Node *outboard);

    Node *getInboardNode ();
    Node *getOutboardNode ();

    array<double, 2> normalized_transform ();

    //calculate intersection points
    array<double, 3> yz_intersection ();
    array<double, 3> xz_intersection ();

    void translate (array<double, 3>); //parameter : [x_shift, y_shift, z_shift]
    void flatten_rotate (array<double, 3>); //parameter : [x_rot, y_rot, z_rot]
    
    //coordinates relating to the beam
    array<double, 3> direction ();
    array<double, 3> center ();
    array<double, 3> radius ();
    
    double height ();

private:
    Node *inboard_node;
    Node *outboard_node;
    Node *elements[2];
    Node *all_elements[2];
    bool plotted;
};

#endif