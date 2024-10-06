#include <iostream>
#include "node.h"
using namespace std;

// Node constructor
Node::Node (double pos[]) {
    //copy coordinates into position
    for (int i = 0; i < 3; i++)
    {
        position[i] = pos[i];
    } 
}