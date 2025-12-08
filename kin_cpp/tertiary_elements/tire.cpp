#include "tire.h"
#include "../assets/misc_linalg.h"
#include <cmath>  // For M_PI

Tire::Tire(Node *contact_patch, Kingpin *kp, double _static_gamma, double _static_toe, double _radius, double _width) {
    cp = contact_patch;
    kingpin = kp;
    gamma = _static_gamma;
    static_toe = _static_toe;
    radius = _radius;
    width = _width;
    _induced_steer = _static_toe;

    /* Calculate initial center relative to contact patch */
    // Note: gamma and static_toe are in radians
    // Python: rotation_matrix(unit_vec=[1, 0, 0], theta=static_gamma * np.pi / 180)
    // The Python code converts degrees to radians, but in C++ we receive radians directly
    // For camber (gamma), positive rotation about x-axis should tilt the tire outward (positive y)
    // So we use -gamma to match the expected sign convention
    StaticMatrix<double, 3UL, 3UL> x_rot_static = rotation_matrix({1, 0, 0}, -gamma);
    StaticMatrix<double, 3UL, 3UL> z_rot_static = rotation_matrix({0, 0, 1}, static_toe);
    StaticVector<double, 3UL> center_vec_local = {0, 0, radius};
    initial_center_relative_to_cp = z_rot_static * (x_rot_static * center_vec_local);
    initial_center = cp->position + initial_center_relative_to_cp;

    /* Calculate center relative to kingpin */
    StaticVector<double, 2UL> xy_angles = kingpin->getBeam()->rotation_angles();
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
    StaticMatrix<double, 3UL, 3UL> rotation = rotation_matrix({1, 0, 0}, -gamma);
    initial_direction = z_rot_static * (rotation * y_axis);

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
    StaticVector<double, 2UL> xy_angles = kingpin->getBeam()->rotation_angles();
    StaticMatrix<double, 3UL, 3UL> x_rot = rotation_matrix({1, 0, 0}, -1 * xy_angles[0]);
    StaticMatrix<double, 3UL, 3UL> y_rot = rotation_matrix({0, 1, 0}, xy_angles[1]);
    StaticMatrix<double, 3UL, 3UL> z_rot = rotation_matrix({0, 0, 1}, _induced_steer);

    StaticVector<double, 3UL> direction = x_rot * (y_rot * (z_rot * tire_direction));
    return direction;
}

StaticVector<double, 3UL> Tire::center() const {
    // Center is calculated relative to contact patch, then transformed by kingpin and steer
    // For simple case (vertical kingpin, no additional steer), center is just: cp->position + initial_center_relative_to_cp
    
    StaticVector<double, 2UL> xy_angles = kingpin->getBeam()->rotation_angles();
    double additional_steer = _induced_steer - static_toe;
    
    // If kingpin is vertical and no additional steer, use simple calculation
    if (std::abs(xy_angles[0]) < 1e-6 && std::abs(xy_angles[1]) < 1e-6 && std::abs(additional_steer) < 1e-6) {
        return cp->position + initial_center_relative_to_cp;
    }
    
    // Otherwise, apply kingpin rotations and additional steer
    StaticMatrix<double, 3UL, 3UL> x_rot = rotation_matrix({1, 0, 0}, -1 * xy_angles[0]);
    StaticMatrix<double, 3UL, 3UL> y_rot = rotation_matrix({0, 1, 0}, xy_angles[1]);
    StaticMatrix<double, 3UL, 3UL> z_rot = rotation_matrix({0, 0, 1}, additional_steer);

    // Transform center relative to contact patch, then add contact patch position
    StaticVector<double, 3UL> center_position = cp->position + y_rot * (x_rot * (z_rot * initial_center_relative_to_cp));

    return center_position;
}

double Tire::induced_steer() const {
    return _induced_steer;
}

void Tire::set_induced_steer(double value) {
    _induced_steer = static_toe;
    _induced_steer += value;

    StaticVector<double, 2UL> xy_angles = kingpin->getBeam()->rotation_angles();
    StaticMatrix<double, 3UL, 3UL> x_rot = rotation_matrix({1, 0, 0}, -1 * xy_angles[0]);
    StaticMatrix<double, 3UL, 3UL> y_rot = rotation_matrix({0, 1, 0}, xy_angles[1]);
    StaticMatrix<double, 3UL, 3UL> z_rot = rotation_matrix({0, 0, 1}, _induced_steer);

    cp->position = x_rot * (y_rot * (z_rot * cp_to_kingpin)) + kingpin->getBeam()->getInboardNode()->position;
}

double Tire::height() const {
    // Height is the outer diameter (radius * 2)
    return radius * 2.0;
}

double Tire::getRadius() const {
    return radius;
}

double Tire::getWidth() const {
    return width;
}

double Tire::getGamma() const {
    return gamma;
}

double Tire::getStaticToe() const {
    return static_toe;
}

Node* Tire::getContactPatch() const {
    return cp;
}