#include <array>
#include "node.h"
using namespace std;

Node (array<double, 3>);
void reset ();
void translate(array<double, 3>);
void flatten_rotate(array<double, 3>);

/* constructor for Node object 
   Parameters : pos (3-element array) -- x, y, z*/
Node::Node (array<double, 3> pos) {
    //copy coordinates into position
    for (int i = 0; i < 3; i++)
    {
        position[i] = pos[i];
    } 
}

/*  */