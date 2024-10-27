#include "steering_link.h"
#include "../assets/misc_linalg.h"

SteeringLink::SteeringLink (Beam *sb, Kingpin *kp) {
    steering_beam = sb;
    kingpin = kp;
    initial_length = length ();
    angle = 0.0;
}

double SteeringLink::length () const {
    return norm (steering_beam->getInboardNode ()->position - steering_beam->getOutboardNode ()->position);
}

StaticVector<double, 3UL> SteeringLink::_steering_pickup_to_kingpin () const {
    StaticVector<double, 2UL> xy_angles = kingpin->getBeam ()->normalized_transform ();
    StaticVector<double, 3UL> steering_pickup_pos_shifted = steering_beam->getOutboardNode ()->position - steering_beam->getInboardNode ()->position;
    StaticMatrix<double, 3UL, 3UL> x_rot = rotation_matrix ({1, 0, 0}, xy_angles[0]);
    StaticMatrix<double, 3UL, 3UL> y_rot = rotation_matrix ({0, 1, 0}, -1 * xy_angles[1]);

    return y_rot * (x_rot * steering_pickup_pos_shifted);
}
void SteeringLink::update () {
    StaticVector<double, 2UL> xy_angles = kingpin->getBeam ()->normalized_transform ();
    StaticMatrix<double, 3UL, 3UL> x_rot = rotation_matrix ({1, 0, 0}, -1 * xy_angles[0]);
    StaticMatrix<double, 3UL, 3UL> y_rot = rotation_matrix ({0, 1, 0}, xy_angles[1]);
    StaticMatrix<double, 3UL, 3UL> z_rot = rotation_matrix ({0, 0, 1}, angle);
    StaticVector<double, 3UL> steering_pickup_position = x_rot * (y_rot * (z_rot * _steering_pickup_to_kingpin ())) + kingpin->getBeam ()->getInboardNode ()->position;

    steering_beam->getOutboardNode ()->position = steering_pickup_position;
}

void SteeringLink::rotate (double new_angle) {
    _set_initial_position ();
    angle = new_angle;
    update ();
}
void SteeringLink::_set_initial_position () {
    StaticMatrix<double, 3UL, 3UL> kingpin_rot = rotation_matrix (kingpin->getBeam ()->direction (), -1 * angle);
    StaticVector<double, 3UL> steer_about_origin = steering_beam->getOutboardNode ()->position - kingpin->getBeam ()->getInboardNode ()->position;
    steering_beam->getOutboardNode ()->position = kingpin_rot * steer_about_origin + kingpin->getBeam ()->getInboardNode ()->position;
}