#include <iostream>
#include "beam.h"
using namespace std;

Node *inboard_node;
Node *outboard_node;
Node *elements[2];
Node *all_elements[2];
bool plotted;

//Beam constructor
Beam::Beam (Node *in, Node *out) {
    inboard_node = in;
    outboard_node = out;
    plotted = false;

    elements[0] = in;
    elements[1] = out;
    all_elements[0] = in;
    all_elements[1] = out;
}

Node *Beam::getInboardNode () {
    return inboard_node;
}

Node *Beam::getOutboardNode() {
    return outboard_node;
}
