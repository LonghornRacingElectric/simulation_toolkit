#include <array>
#include "node.h"
using namespace std;

void reset ();
void translate(array<double, 3>);
void flatten_rotate(array<double, 3>);

/* constructor for Node object 
   Parameters : pos (3-element array) -- x, y, z*/
Node::Node (array<double, 3> pos) {
    //copy coordinates into position
    for (int i = 0; i < 3; i++)
    {
        position[i] = initial_position[i] = pos[i];
    } 

}

/* Function Reset: resets the current posiiton array to the initial position */ 
void Node::reset() {
    for (int i=0; i<3; i++) {
        position[i] = initial_position[i];
    }
}

/* Translates Node
   Parameters : translation (3-element array) -- [x_shift, y_shift, z_shift]*/
void Node::translate(array<double, 3> translation) {
    //TODO: requires plotting library
}

/* Rotates node
   - Used to re-orient vehicle such that contact patches intersect with x-y plane 
   Parameters : angle (3-element array) -- [x_rot, y_rot, z_rot] */
void Node::flatten_rotate(array<double, 3> angle) {
    //TODO: requires linear alg library
}