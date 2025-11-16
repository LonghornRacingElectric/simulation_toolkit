#include "tire.h"
#include "../assets/misc_linalg.h"

Tire::Tire(Node *contact_patch, Kingpin *kp, double _static_gamma, double _static_toe, double _radius, double _width) {
    cp = contact_patch;
    kingpin = kp;
    gamma = _static_gamma;
    static_toe = _static_toe;
    radius = _radius;
    width = _width;
    _induced_steer = _static_toe;

    /* Calculate initial center */
    StaticMatrix<double, 3UL, 3UL> rotation = rotation_matrix({1, 0, 0}, gamma);
    StaticVector<double, 3UL> z_axis({0, 0, 1});
    initial_center = (rotation * z_axis) * radius + cp->position;

    /* Calculate center relative to kingpin */
    StaticVector<double, 2UL> xy_angles = kingpin->getBeam()->normalized_transform();
    StaticVector<double, 3UL> center_shifted = initial_center - kingpin->getBeam()->getInboardNode()->position;
    StaticMatrix<double, 3UL, 3UL> x_rot = rotation_matrix({1, 0, 0}, xy_angles[0]);
    StaticMatrix<double, 3UL, 3UL> y_rot = rotation_matrix({0, 1, 0}, -1 * xy_angles[1]);
    center_to_kingpin = y_rot * (x_rot * center_shifted);
    
    /* Calculate contact patch relative to kingpin */
    StaticVector<double, 3UL> cp_shifted = cp->initial_position - kingpin->getBeam()->getInboardNode()->position;
    cp_to_kingpin = y_rot * (x_rot * cp_shifted);

    /* Calculate static direction */
    initial_kpi = xy_angles[0];
    initial_caster = xy_angles[1];
    StaticVector<double, 3UL> y_axis({0, 1, 0});
    initial_direction = rotation * y_axis;

    /* Calculate direction relative to kingpin */
    tire_direction = y_rot * (x_rot * initial_direction);

    elements[0] = all_elements[0] = cp;
}

void Tire::translate (StaticVector<double, 3UL> &translation) {
    for (Node *element : all_elements) {
        element->translate(translation);
    }
}

void Tire::flatten_rotate(StaticVector<double, 3UL> &angle) {
    for (Node *element : all_elements) {
        element->flatten_rotate (angle);
    }
}

StaticVector<double, 3UL> Tire::direction() const {
    StaticVector<double, 2UL> xy_angles = kingpin->getBeam()->normalized_transform();
    StaticMatrix<double, 3UL, 3UL> x_rot = rotation_matrix({1, 0, 0}, -1 * xy_angles[0]);
    StaticMatrix<double, 3UL, 3UL> y_rot = rotation_matrix({0, 1, 0}, xy_angles[1]);
    StaticMatrix<double, 3UL, 3UL> z_rot = rotation_matrix({0, 0, 1}, _induced_steer);

    StaticVector<double, 3UL> direction = x_rot * (y_rot * (z_rot * tire_direction));
    return direction;
}

StaticVector<double, 3UL> Tire::center() const {
    StaticVector<double, 2UL> xy_angles = kingpin->getBeam()->normalized_transform();
    StaticMatrix<double, 3UL, 3UL> x_rot = rotation_matrix({1, 0, 0}, -1 * xy_angles[0]);
    StaticMatrix<double, 3UL, 3UL> y_rot = rotation_matrix({0, 1, 0}, xy_angles[1]);
    StaticMatrix<double, 3UL, 3UL> z_rot = rotation_matrix({0, 0, 1}, _induced_steer);

    StaticVector<double, 3UL> center_position = y_rot * (x_rot * (z_rot * center_to_kingpin)) + kingpin->getBeam()->getInboardNode()->position;

    return center_position;
}

double Tire::induced_steer() const {
    return _induced_steer;
}

void Tire::set_induced_steer(double value) {
    _induced_steer = static_toe;
    _induced_steer += value;

    StaticVector<double, 2UL> xy_angles = kingpin->getBeam()->normalized_transform();
    StaticMatrix<double, 3UL, 3UL> x_rot = rotation_matrix({1, 0, 0}, -1 * xy_angles[0]);
    StaticMatrix<double, 3UL, 3UL> y_rot = rotation_matrix({0, 1, 0}, xy_angles[1]);
    StaticMatrix<double, 3UL, 3UL> z_rot = rotation_matrix({0, 0, 1}, _induced_steer);

    cp->position = x_rot * (y_rot * (z_rot * cp_to_kingpin)) + kingpin->getBeam()->getInboardNode()->position;
}