#include "beam.h"
#include "../assets/misc_linalg.h"
#include <cmath>
#include <stdexcept>
#include <iostream>
#include <limits>
using namespace blaze;

/* Beam constructor */
Beam::Beam(Node *in, Node *out) {
    inboard_node = in;
    outboard_node = out;
    plotted = false;
    
    elements[0] = all_elements[0] = in;
    elements[1] = all_elements[1] = out;
    
    // Calculate initial length using Node subtraction operator
    Node diff = *out - *in;
    initial_length_value = norm(diff.position);
}

/* GETTER : gets inboard Node */
Node* Beam::getInboardNode() const {
    return inboard_node;
}

/* GETTER : gets outboard Node */
Node* Beam::getOutboardNode() const {
    return outboard_node;
}

/* Initial length getter */
double Beam::initial_length() const {
    return initial_length_value;
}

/* Calculates intersection point between two links in the y-z plane */
Node Beam::yz_intersection(const Beam &link) const {
    // Use Node objects and operators, matching Python: (l_1o - l_1i)[2] / (l_1o - l_1i)[1]
    Node l_1i = *inboard_node;
    Node l_1o = *outboard_node;
    Node diff_1 = l_1o - l_1i;
    double y_diff_1 = diff_1[1];
    double m_1 = (std::abs(y_diff_1) < 1e-10) ? 0.0 : diff_1[2] / y_diff_1;
    double y_1 = l_1o[1], z_1 = l_1o[2];
    
    Node l_2i = *link.inboard_node;
    Node l_2o = *link.outboard_node;
    Node diff_2 = l_2o - l_2i;
    double y_diff_2 = diff_2[1];
    double m_2 = (std::abs(y_diff_2) < 1e-10) ? 0.0 : diff_2[2] / y_diff_2;
    double y_2 = l_2o[1], z_2 = l_2o[2];
    
    StaticMatrix<double, 2UL, 2UL> a{
        {-1 * m_1, 1},
        {-1 * m_2, 1}
    };
    
    StaticVector<double, 2UL> b{
        -1 * m_1 * y_1 + z_1,
        -1 * m_2 * y_2 + z_2
    };
    
    double y, z;
    try {
        StaticVector<double, 2UL> yz;
        solve(a, yz, b);
        y = yz[0];
        z = yz[1];
    } catch (...) {
        // Singular matrix - intersection at infinity
        std::cerr << "\nSingular Matrix Encountered | yz intersection assumed at infinity. This is not a critical error, but check results carefully." << std::endl;
        y = std::numeric_limits<double>::infinity();
        z = (z_2 + z_1) / 2.0;
    }
    
    // Calculate x-value - average between the two links (using Node indexing)
    double x = (l_1o[0] + l_2o[0]) / 2.0;
    
    return Node(StaticVector<double, 3UL>{x, y, z});
}

/* Calculates intersection point between two links in the x-z plane */
Node Beam::xz_intersection(const Beam &link) const {
    // Use Node objects and operators, matching Python: (l_1o - l_1i)[2] / (l_1o - l_1i)[0]
    Node l_1i = *inboard_node;
    Node l_1o = *outboard_node;
    Node diff_1 = l_1o - l_1i;
    double x_diff_1 = diff_1[0];
    double m_1 = (std::abs(x_diff_1) < 1e-10) ? 0.0 : diff_1[2] / x_diff_1;
    double x_1 = l_1o[0], z_1 = l_1o[2];
    
    Node l_2i = *link.inboard_node;
    Node l_2o = *link.outboard_node;
    Node diff_2 = l_2o - l_2i;
    double x_diff_2 = diff_2[0];
    double m_2 = (std::abs(x_diff_2) < 1e-10) ? 0.0 : diff_2[2] / x_diff_2;
    double x_2 = l_2o[0], z_2 = l_2o[2];
    
    StaticMatrix<double, 2UL, 2UL> a{
        {-1 * m_1, 1},
        {-1 * m_2, 1}
    };
    
    StaticVector<double, 2UL> b{
        -1 * m_1 * x_1 + z_1,
        -1 * m_2 * x_2 + z_2
    };
    
    // Calculate y-value - average between front and rear halves (using Node indexing)
    double y = (l_1o[1] + l_2o[1]) / 2.0;
    
    double x, z;
    try {
        StaticVector<double, 2UL> xz;
        solve(a, xz, b);
        x = xz[0];
        z = xz[1];
    } catch (...) {
        // Singular matrix - intersection at infinity
        std::cerr << "\nSingular Matrix Encountered | xz intersection assumed at infinity. This is not a critical error, but check results carefully." << std::endl;
        x = std::numeric_limits<double>::infinity();
        z = (z_2 + z_1) / 2.0;
    }
    
    return Node(StaticVector<double, 3UL>{x, y, z});
}

/* Calculates Node coordinates with Link treated as z-axis */
StaticVector<double, 3UL> Beam::link_centered_coords(const Node &node) const {
    StaticVector<double, 2UL> rot_angles = rotation_angles();
    double ang_x = rot_angles[0];
    double ang_y = rot_angles[1];
    
    // Translate node relative to inboard using Node subtraction
    Node node_translated = node - *inboard_node;
    StaticVector<double, 3UL> node_translated_pos = node_translated.position;
    
    // Rotate about x-axis
    StaticMatrix<double, 3UL, 3UL> x_rot = rotation_matrix(StaticVector<double, 3UL>{1, 0, 0}, ang_x);
    // Rotate about y-axis (negative)
    StaticMatrix<double, 3UL, 3UL> y_rot = rotation_matrix(StaticVector<double, 3UL>{0, 1, 0}, -ang_y);
    
    // Apply rotations: y_rot * x_rot * node_translated
    StaticVector<double, 3UL> node_coords_rotated = y_rot * (x_rot * node_translated_pos);
    
    return node_coords_rotated;
}

/* Calculates the smallest angles between the ground plane and the projection of Link on the principal planes */
StaticVector<double, 2UL> Beam::component_angles() const {
    Node origin_transform = *outboard_node - *inboard_node;
    StaticVector<double, 3UL> transform_pos = origin_transform.position;
    double ang_x = (std::abs(transform_pos[1]) < 1e-10) ? 0.0 : std::atan(transform_pos[2] / transform_pos[1]);
    double ang_y = (std::abs(transform_pos[0]) < 1e-10) ? 0.0 : std::atan(transform_pos[2] / transform_pos[0]);
    
    return StaticVector<double, 2UL>{ang_x, ang_y};
}

/* Calculates the rotations about x and y which result in a vector pointing strictly in z */
StaticVector<double, 2UL> Beam::rotation_angles() const {
    Node origin_transform = *outboard_node - *inboard_node;
    StaticVector<double, 3UL> transform_pos = origin_transform.position;
    double len = length();
    
    if (len < 1e-10) {
        return StaticVector<double, 2UL>{0.0, 0.0};
    }
    
    double ang_x = (std::abs(transform_pos[2]) < 1e-10) ? 0.0 : std::atan(transform_pos[1] / transform_pos[2]);
    double sign_z = (transform_pos[2] >= 0) ? 1.0 : -1.0;
    double ang_y = sign_z * std::asin(transform_pos[0] / len);
    
    return StaticVector<double, 2UL>{ang_x, ang_y};
}

/* Direction attribute of Link */
StaticVector<double, 3UL> Beam::direction() const {
    // Use Node position access through Node objects
    return unit_vec(inboard_node->position, outboard_node->position);
}

/* Center attribute of link */
StaticVector<double, 3UL> Beam::center() const {
    // Use Node addition and division operators
    Node center_node = (*inboard_node + *outboard_node) / 2.0;
    return center_node.position;
}

/* Radius attribute of Link */
double Beam::radius() const {
    const double DIAMETER = 0.015875;
    return DIAMETER / 2.0;
}

/* Length of link */
double Beam::length() const {
    Node diff = *outboard_node - *inboard_node;
    return norm(diff.position);
}

/* Translates all children (inboard and outboard nodes) */
void Beam::translate(const StaticVector<double, 3UL> &translation) {
    for (Node *curr : all_elements) {
        curr->translate(translation);
    }
}

/* Rotates the beam by rotating both nodes about a common origin */
void Beam::rotate(const Node &origin,
                  bool persistent,
                  const std::optional<StaticVector<double, 3UL>> &direction,
                  const std::optional<double> &angle,
                  const std::optional<double> &ang_x,
                  const std::optional<double> &ang_y,
                  const std::optional<double> &ang_z) {
    // Rotate both nodes about the same origin to maintain beam geometry
    inboard_node->rotate(origin, persistent, direction, angle, ang_x, ang_y, ang_z);
    outboard_node->rotate(origin, persistent, direction, angle, ang_x, ang_y, ang_z);
}

/* Legacy method: Rotates all children (inboard and outboard nodes) about their own positions */
void Beam::flatten_rotate(const StaticVector<double, 3UL> &rotation) {
    for (Node *curr : all_elements) {
        curr->flatten_rotate(rotation);
    }
}
