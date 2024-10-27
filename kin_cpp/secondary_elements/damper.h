#ifndef DAMPER.H
#define DAMPER.H

#include <iostream>
#include "../primary_elements/beam.h"

class Damper {
public:
    Damper(Beam *beam);

    Beam *getBeam ();
    // Functions for accessing length and angle
    double getAngle() const;
    double getLength() const;
private:
    Beam *damper_beam;
    Node *elements[2];
    Node *all_elements[2];
    double length;
    double angle;

    double calculateLength();
};

#endif