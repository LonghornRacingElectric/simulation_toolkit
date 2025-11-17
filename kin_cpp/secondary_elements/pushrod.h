#ifndef PUSHROD_H
#define PUSHROD_H

#include <iostream>
#include "../primary_elements/beam.h"

class Pushrod {
public:
    Pushrod(Beam *beam);

    Beam *getBeam();

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