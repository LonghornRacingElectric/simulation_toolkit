#include <array>
#include "node.h"
using namespace blaze;

void reset ();
void translate(StaticVector<double, 3UL>);
void flatten_rotate(StaticVector<double, 3UL>);

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
    //TODO: requires linear alg library
}