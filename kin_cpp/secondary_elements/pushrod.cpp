// Notes: Pushrod is made from a beam and a bellcrank
#include <iostream>
#include <cmath>
#include "pushrod.h"
using namespace std;

Pushrod::Pushrod(Beam *beam) {
    pushrod_beam = beam;
    length = calculateLength();
    angle = 0.0;
}

// Getter functions for length and angle
double Pushrod::getLength() const {
    return length;
}

double Pushrod::getAngle() const {
    return angle;
}

//TODO : Implement pushrod rotation

double Pushrod::calculateLength() {
    length = pushrod_beam->height();
    return length;
}

