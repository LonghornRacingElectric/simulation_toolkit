#include <array>
#include "../primary_elements/beam.h"
#include "wishbone.h"
#include "../assets/misc_linalg.h"
using namespace blaze;

// Wishbone constructor
Wishbone::Wishbone (Beam *fore_beam, Beam *aft_beam) {
    fore = fore_beam;
    aft = aft_beam;

    elements[0] = fore;
    elements[1] = aft;

    // Store all nodes: fore inboard, fore outboard, aft inboard, aft outboard
    all_elements[0] = fore->getInboardNode();
    all_elements[1] = fore->getOutboardNode();
    all_elements[2] = aft->getInboardNode();
    all_elements[3] = aft->getOutboardNode();
    
    angle = 0.0;
    direction = direction_vec();
}

/* Rotates wishbone around axis connecting inboard nodes
   Parameter : new_angle : float - Angle of rotation in radians */
void Wishbone::rotate (double new_angle) {
    // Reset both outboard nodes to initial positions (matching Python)
    fore->getOutboardNode()->reset();
    aft->getOutboardNode()->reset();
    
    // Set the new angle
    angle = new_angle;
    
    // Rotate the fore outboard node about the fore inboard node using direction and angle
    // Match Python: self.fore_link.outboard_node.rotate(origin=self.fore_link.inboard_node, direction=self.direction, angle=angle)
    fore->getOutboardNode()->rotate(
        *fore->getInboardNode(),  // origin
        false,                     // persistent
        std::make_optional(direction),  // direction vector (wrapped in optional)
        std::make_optional(angle),     // angle (wrapped in optional)
        std::nullopt, std::nullopt, std::nullopt  // no xyz angles
    );
}

/* Rotates all children - used to reorient vehicle such that contact patches intersect with x-y plane 
    Parameter : angle : (3-element array) -- angle of rotation in radians [x_rot, y_rot, z_rot] */
void Wishbone::flatten_rotate (const StaticVector<double, 3> &angle) {
    for (Node *curr : all_elements) {
        curr->flatten_rotate (angle);
    }
}

/* Resets position of wishbone to initial position */
void Wishbone::set_initial_position () {
    // Reset both outboard nodes (matching Python behavior)
    fore->getOutboardNode()->reset();
    aft->getOutboardNode()->reset();
    angle = 0.0;
}

/* Translates all children (inboard and outboard Nodes) 
    Parameter : translation : (3-element array) -- translation [x_shift, y_shift, z_shift] */
void Wishbone::translate (const StaticVector<double, 3> &translation) {
    for (Node *curr : all_elements) {
        curr->translate (translation);
    }
}

/* Calculates plane coincident with wishbone
   General equation: a(x - x_{0}) + b(y - y_{0}) + c(z - z_{0}) = 0
   Returns parameters defining plane [a, b, c, x_0, y_0, z_0] */
StaticVector<double, 6> Wishbone::plane () const {
    // Match Python implementation: use cross product of PQ and PR
    // PQ = fore outboard - fore inboard
    // PR = aft outboard - aft inboard
    Node PQ_node = *fore->getOutboardNode() - *fore->getInboardNode();
    Node PR_node = *aft->getOutboardNode() - *aft->getInboardNode();
    
    StaticVector<double, 3UL> PQ = PQ_node.position;
    StaticVector<double, 3UL> PR = PR_node.position;
    
    // Calculate cross product: a, b, c = cross(PQ, PR)
    StaticVector<double, 3UL> normal = cross(PQ, PR);
    double a = normal[0];
    double b = normal[1];
    double c = normal[2];
    
    // x_0, y_0, z_0 = fore outboard position
    StaticVector<double, 3UL> x0_vec = fore->getOutboardNode()->position;
    double x_0 = x0_vec[0];
    double y_0 = x0_vec[1];
    double z_0 = x0_vec[2];
    
    return StaticVector<double, 6>{a, b, c, x_0, y_0, z_0};
}

/* Calculates unit vector from inboard aft node to inboard fore node 
   Returns unit vector (inboard aft to inboard fore)*/
StaticVector<double, 3> Wishbone::direction_vec () const {
    // Use unit_vec function: from aft inboard to fore inboard
    return unit_vec(aft->getInboardNode()->position, fore->getInboardNode()->position);
}

/* GETTER : Aft beam beam*/
Beam *Wishbone::getForeBeam () {
    return fore;
}

/* GETTER : Aft beam*/
Beam *Wishbone::getAftBeam () {
    return aft;
}

/*GETTER : Angle */
double Wishbone::getAngle () {
    return angle;
}

/* GETTER : Direction vector */
StaticVector<double, 3> &Wishbone::getDirection () {
    return direction;
}
