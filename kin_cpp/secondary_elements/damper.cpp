#include <iostream>
#include <cmath>
#include "pushrod.h"
#include "../primary_elements/node.h"
#include "../primary_elements/beam.h"
using namespace std;

Damper::Damper(Beam *beam) {
    inboard_node = beam->getInboardNode();
    outboard_node = beam->getOutboardNode();
    length = calculateLength();
    angle = 0.0;
}

// Getter functions for inboard and outboard nodes
Node *Beam::getInboardNode() {
    return inboard_node;
}

Node *Beam::getOutboardNode() {
    return outboard_node;
}

// Getter functions for length and angle
double Beam::getLength() const {
    return length;
}

double Beam::getAngle() const {
    return angle;
}

// Change length of damper
double Beam::changeLength() {
    // TODO : Implement change of outboard node positions to change length
}

//TODO : Implement spring rates

double Beam::calculateLength() const {
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