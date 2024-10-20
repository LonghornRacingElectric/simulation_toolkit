#include "kingpin.h"

Kingpin::Kingpin(Beam *kp_beam) {
    kingpin_beam = kp_beam;
    initial_length = length();
}

Beam *Kingpin::getBeam() const {
    return kingpin_beam;
}

double Kingpin::length() const {
    return kingpin_beam->height();
}