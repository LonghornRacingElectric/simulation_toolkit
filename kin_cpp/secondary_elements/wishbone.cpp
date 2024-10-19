#include <array>
#include "../primary_elements/beam.h"
#include "wishbone.h"

using namespace std;

void rotate (double angle);
void flatten_rotate (array<double, 3>); 
void _set_initial_position ();
void translate (array<double, 3>); 
array<double, 6> plane ();
array<double, 3> direction_vec ();

Beam *getForeBeam ();
Beam *getAftBeam ();
double getAngle ();
array<double, 3> getDirection ();

array<double, 3> position_diff (array<double, 3>, array<double, 3>);

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
    //need linalg library for : rot_mat = rotation_matrix (direction, angle)
    array<double, 3> outboard_point = position_diff (all_elements[1]->position, all_elements[0]->position);
    //need linalg library for all_elements[0]->position = np.matmul (rot_mat, outboard_point) + inpos
    angle = new_angle; 
}

/* Rotates all children - used to reorient vehicle such that contact patches intersect with x-y plane 
    Parameter : angle : (3-element array) -- angle of rotation in radians [x_rot, y_rot, z_rot] */
void Wishbone::flatten_rotate (array<double, 3> angle) {
    for (Node *curr : all_elements) {
        curr->flatten_rotate (angle);
    }
}

/* Resets position of wishbone to initial position */
void Wishbone::set_initial_position () {
    //need linalg library for : rot_mat = rotation_matrix (direction, -1 * angle)
    array<double, 3> outboard_point = position_diff (all_elements[1]->position, all_elements[0]->position);
    //need linalg library for : fore_link.outboard_node.position = np.matmul(self.rot_mat, outboard_point) + self.fore_link.inboard_node.position
}

/* Translates all children (inboard and outboard Nodes) 
    Parameter : translation : (3-element array) -- translation [x_shift, y_shift, z_shift] */
void Wishbone::translate (array<double, 3> translation) {
    for (Node *curr : all_elements) {
        curr->translate (translation);
    }
}

/* Calculates plane coincident with wishbone
   General equation: a(x - x_{0}) + b(y - y_{0}) + c(z - z_{0}) = 0
   Returns parameters defining plane [a, b, c, x_0, y_0, z_0] */
array<double, 6> Wishbone::plane () const {
    array<double, 3> PQ = position_diff (all_elements[1]->position, all_elements[0]->position);
    array<double, 3> PR = position_diff (all_elements[1]->position, all_elements[2]->position);
    //need linalg library for : a, b, c = np.cross(PQ, PR);
    int x_0 = all_elements[1]->position[0], x_1 = all_elements[1]->position[1], x_2 = all_elements[1]->position[2];
    return {a, b, c, x_0, x_1, x_2};
}

/* Calculates unit vector from inboard aft node to inboard fore node 
   Returns unit vector (inboard aft to inboard fore)*/
array<double, 3> Wishbone::direction_vec () const {
    //need to implement unit_vec for : return unit_vec(self.fore_link.inboard_node, self.aft_link.inboard_node)
    return nullptr;
}

/* GETTER : Aft beam beam*/
Beam *Wishbone::getForeBeam () {
    return fore;
}

/* GETTER : Aft beam*/
Beam *Wishbone::getAftBeam () {
    return aft;
}

/*GETTER : angle */
double Wishbone::getAngle () {
    return angle;
}

/* GETTER : direction vector */
array<double, 3> Wishbone::getDirection () {
    return direction;
}
//Helper function to vector between two points because I don't wanna write this shit multiple times
array<double, 3> Wishbone::position_diff (array<double, 3> pos_out, array<double, 3> pos_in) {
    return {pos_out[0] - pos_in[0], pos_out[1] - pos_in[1], pos_out[2] - pos_in[2]};
}