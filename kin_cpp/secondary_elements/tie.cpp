#include <iostream>
#include <cmath>
#include "tie.h"
#include "../primary_elements/node.h"
#include "../primary_elements/beam.h"
using namespace std;

Tie::Tie(Beam *beam) {
    inboard_node = beam->getInboardNode();
    outboard_node = beam->getOutboardNode();
    length = calculateLength();
    angle = 0.0;

}

// Return functions for inboard and outboard nodes
Node *Tie::getInboardNode() {
    return inboard_node;
}
Node *Tie::getOutboardNode() {
    return outboard_node;
}

// Return functions for length and angle
double Tie::getLength() const {
    return length;
}
double Tie::getAngle() const {
    return angle;
}

double Tie::calculateLength() const {
    // Gather xyz coordinates for inboard and outboard nodes
    double x1 = inboard_node->position[0];
    double y1 = inboard_node->position[1];
    double z1 = inboard_node->position[2];

    double x2 = outboard_node->position[0];
    double y2 = outboard_node->position[1];
    double z2 = outboard_node->position[2];

    // Calculate the distance 
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) + pow(z2 - z1, 2));
}