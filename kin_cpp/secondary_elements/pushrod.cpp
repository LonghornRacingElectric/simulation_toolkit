// Notes: Pushrod is made from a beam
#include <iostream>
#include <cmath>
#include "pushrod.h"
#include "../primary_elements/beam.h"
using namespace std;

Pushrod::Pushrod(Beam *beam) {
    pushrod_beam = beam;
    
    // Store node pointers for backward compatibility
    elements[0] = all_elements[0] = beam->getInboardNode();
    elements[1] = all_elements[1] = beam->getOutboardNode();
    
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

Beam *Pushrod::getBeam() {
    return pushrod_beam;
}
double Pushrod::calculateLength() {
    length = pushrod_beam->length();
    return length;
}

