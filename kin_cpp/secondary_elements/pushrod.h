#ifndef PUSHROD.H
#define PUSHROD.H

#include <iostream>
#include "../primary_elements/beam.h"

class Pushrod {
public:
    Pushrod(Beam *beam);

    // Functions for inboard and outboard nodes
    Node *getInboardNode();
    Node *getOutboardNode();

    // Functions for accessing length and angle
    double getAngle() const;
    double getLength() const;

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