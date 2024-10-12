#ifndef TIE_H
#define TIE_H

#include <iostream>
#include "../primary_elements/beam.h"

class Tie {
public: 
    Tie(Beam *beam);

    // Functions for getting inboard and outboard
    Node *getInboardNode();
    Node *getOutboardNode();

    // Functions for getting length and angle
    double getLength() const;
    double getAngle() const;
    

private:
    Node *inboard_node;
    Node *outboard_node;
    Node *elements[2];
    Node *all_elements[2];
    double length;
    double angle;

    double calculateLength() const;
};

#endif