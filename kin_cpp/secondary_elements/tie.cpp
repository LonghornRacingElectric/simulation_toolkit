#include <iostream>
#include <cmath>
#include "tie.h"
#include "../assets/misc_linalg.h"
using namespace blaze;

Tie::Tie(Beam *beam, Kingpin *kp) {
    tie_beam = beam;
    kingpin = kp;
    length = calculateLength();
    initial_length = length;
    angle = 0.0;
    steering_pickup_to_kingpin = _steering_pickup_to_kingpin();
}

StaticVector<double, 3UL> Tie::_steering_pickup_to_kingpin() const {
    StaticVector<double, 2UL> angles = kingpin->getBeam()->normalized_transform();
    StaticVector<double, 3UL> steering_pickup_pos_shifted = tie_beam->getOutboardNode()->position - kingpin->getBeam()->getInboardNode()->position;
    StaticMatrix<double, 3UL, 3UL> x_rot = rotation_matrix({1, 0, 0}, angles[0]);
    StaticMatrix<double, 3UL, 3UL> y_rot = rotation_matrix({0, 1, 0}, -1 * angles[1]);
    return y_rot * (x_rot * steering_pickup_pos_shifted);
}

// Return functions for length and angle
double Tie::getLength() const {
    return length;
}
double Tie::getAngle() const {
    return angle;
}

double Tie::getInitialLength () const {
    return initial_length;
}

Beam *Tie::getTieBeam () const {
    return tie_beam;
}
void Tie::update() {
    StaticVector<double, 3UL> angles = kingpin->getBeam()->normalized_transform();
    StaticMatrix<double, 3UL, 3UL> x_rot = rotation_matrix({1, 0, 0}, -1 * angles[0]);
    StaticMatrix<double, 3UL, 3UL> y_rot = rotation_matrix({0, 1, 0}, angles[1]);
    StaticMatrix<double, 3UL, 3UL> z_rot = rotation_matrix({0, 0, 1}, angle);

    StaticVector<double, 3UL> steering_pickup_position = x_rot * (y_rot * (z_rot * steering_pickup_to_kingpin)) + kingpin->getBeam()->getInboardNode()->position;
    tie_beam->getOutboardNode()->position = steering_pickup_position;
}

void Tie::set_initial_position() {
    StaticMatrix<double, 3UL, 3UL> kingpin_rot = rotation_matrix(kingpin->getBeam()->direction(), -1 * angle);
    StaticVector<double, 3UL> steer_about_origin = tie_beam->getOutboardNode()->position - kingpin->getBeam()->getInboardNode()->position;
    tie_beam->getOutboardNode()->position = kingpin_rot * steer_about_origin + kingpin->getBeam()->getInboardNode()->position;
}

void Tie::rotate (double new_angle) {
    set_initial_position();
    angle = new_angle;
    update();
}
double Tie::calculateLength() {
    length = tie_beam->height();
    return length;
}