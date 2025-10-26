#ifndef BEAM_H
#define BEAM_H

#include <array>
#include "node.h"

using namespace blaze;
class Beam {
public:
    Beam(Node *inboard, Node *outboard);

    Node *getInboardNode ();
    Node *getOutboardNode ();
    
    StaticVector<double, 2UL> normalized_transform ();

    //calculate intersection points
    StaticVector<double, 3UL> yz_intersection (Beam *);
    StaticVector<double, 3UL> xz_intersection (Beam *);

    void translate (const StaticVector<double, 3> &); //parameter : [x_shift, y_shift, z_shift]
    void flatten_rotate (const StaticVector<double, 3> &); //parameter : [x_rot, y_rot, z_rot]
    
    //coordinates relating to the beam
    StaticVector<double, 3UL> direction () const;
    StaticVector<double, 3UL> center () const;
    double radius () const;

    double height () const;

private:
    Node *inboard_node;
    Node *outboard_node;
    Node *elements[2];
    Node *all_elements[2];
    bool plotted;
};

#endif