#ifndef PUSHROD.H
#define PUSHROD.H

#include <iostream>
#include "../primary_elements/beam.h"

class Pushrod {
public:
    Pushrod(Beam *beam);

    // Functions for accessing length and angle
    double getAngle() const;
    double getLength() const;
    double calculateLength();
private:
    Beam *pushrod_beam;
    Node *elements[2];
    Node *all_elements[2];
    double length;
    double angle;
};

#endif