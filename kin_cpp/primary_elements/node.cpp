#include <array>
#include "node.h"
#include "../assets/misc_linalg.h"
using namespace blaze;

void reset ();
void translate(const StaticVector<double, 3UL>);
void flatten_rotate(const StaticVector<double, 3UL>);

/* constructor for Node object 
   Parameters : pos (3-element array) -- x, y, z*/
Node::Node (const StaticVector<double, 3UL> &pos) {
    //copy coordinates into position - blaze does deep copy on assign
    position = pos;
}

// Function Reset: resets the current posiiton array to the initial position
void Node::reset() {
    position = initial_position;
}

void Node::translate(const StaticVector<double, 3UL> &translation) {
    //TODO: requires plotting library
}

void Node::flatten_rotate(const StaticVector<double, 3UL> &rotation) {
    StaticMatrix<double, 3UL, 3UL> x_rot = rotation_matrix(StaticVector<double, 3UL>{1, 0, 0}, rotation[0]);
    StaticMatrix<double, 3UL, 3UL> y_rot = rotation_matrix(StaticVector<double, 3UL>{0, 1, 0}, rotation[1]);
    StaticMatrix<double, 3UL, 3UL> z_rot = rotation_matrix(StaticVector<double, 3UL>{0, 0, 1}, rotation[2]);

    // Apply the rotation: z_rot * y_rot * x_rot * position
    // NOTE: The order of multiplication switches because matrix multiplication in blaze applies transformations in reverse order.
    // This is due to the nature of matrix multiplication: transformations are applied from right to left, meaning
    // z_rot * y_rot * x_rot * position will rotate the position vector first by the x-axis, then y-axis, then z-axis.
    position = z_rot * (y_rot * (x_rot * position));
}
