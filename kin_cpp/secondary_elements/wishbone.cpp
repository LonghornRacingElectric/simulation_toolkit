#include <array>
#include "../primary_elements/beam.h"
#include "wishbone.h"
#include "../assets/misc_linalg.h"
using namespace blaze;

void rotate (double angle);
void flatten_rotate (const StaticVector<double, 3> &); 
void set_initial_position ();
void translate (const StaticVector<double, 3> &); 
StaticVector<double, 6> plane ();
StaticVector<double, 3> direction_vec ();

Beam *getForeBeam ();
Beam *getAftBeam ();
double getAngle ();
StaticVector<double, 3> &getDirection ();

//Aframe constructor
Wishbone::Wishbone (Beam *fore_beam, Beam *aft_beam) {
    fore = fore_beam;
    aft = aft_beam;

    elements[0] = fore;
    elements[1] = aft;

    all_elements[0] = fore->getInboardNode();
    all_elements[1] = fore->getOutboardNode ();
    all_elements[2] = aft->getInboardNode ();
    angle = 0;
    direction = direction_vec ();
    
}

/* Rotates wishbone around axis connecting inboard nodes
   Parameter : new_angle : float - Angle of rotation in radians */
void Wishbone::rotate (double new_angle) {
    set_initial_position ();
    StaticMatrix<double, 3UL, 3UL> rot_mat = rotation_matrix(direction, angle);
    StaticVector<double, 3UL> outboard_point = all_elements[1]->position - all_elements[0]->position;
    all_elements[1]->position = (rot_mat * outboard_point) + all_elements[0]->position;
    angle = new_angle; 
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
    StaticMatrix<double, 3UL, 3UL> rot_mat = rotation_matrix(direction, -1 * angle);
    StaticVector<double, 3> outboard_point = all_elements[1]->position - all_elements[0]->position;
    all_elements[1]->position = (rot_mat * outbpard_point) + all_elements[0]->position;
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
    StaticMatrix<double, 3, 3> point_vecs;
    column(point_vecs, 0) = all_elements[0]->position;
    column(point_vecs, 1) = all_elements[1]->position;
    column(point_vecs, 2) = all_elements[2]->position;
    return ::plane(point_vecs);
}

/* Calculates unit vector from inboard aft node to inboard fore node 
   Returns unit vector (inboard aft to inboard fore)*/
StaticVector<double, 3> Wishbone::direction_vec () const {
    //need to implement unit_vec for : return unit_vec(self.fore_link.inboard_node, self.aft_link.inboard_node)
    StaticVector<double, 3UL> difference = all_elements[0]->position - all_elements[2]->position
    double norm = norm(diff);
    return difference / norm;
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
