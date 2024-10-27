#include <iostream>
#include <cmath>
#include "damper.h"
using namespace std;

Damper::Damper(Beam *beam) {
    damper_beam = beam;
    length = calculateLength();
    angle = 0.0;
}

/* Getter function for length */
double Damper::getLength() const {
    return length;
}

/* Getter function for angle */
double Damper::getAngle() const {
    return angle;
}

/* Calculates the length of this damper */
double Damper::calculateLength() {
    length = damper_beam->height();
    return length;
}