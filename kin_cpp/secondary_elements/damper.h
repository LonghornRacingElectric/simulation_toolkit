#ifndef DAMPER.H
#define DAMPER.H

#include <iostream>
#include "../primary_elements/beam.h"

class Damper {
public:
    Damper(Beam *beam);

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