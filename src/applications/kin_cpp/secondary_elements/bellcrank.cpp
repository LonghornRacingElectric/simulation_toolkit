#include "bellcrank.h"
#include <iostream>
#include "../primary_elements/node.h"
#include "../primary_elements/beam.h"
using namespace std;

Bellcrank::Bellcrank (Beam *out, Beam *in, Beam *connecting, Node *node, double *BCdirection, double *BCangle) {
    bellcrank_pivot = node;
    inboard = in;
    outboard = out;
    connecting_beam = connecting;
    direction = BCdirection;
    angle = BCangle;
    elements[0] = all_elements[0] = (Node *) outboard;
    elements[1] = all_elements[1] = (Node *) inboard;
    elements[2] = all_elements[2] = (Node *) connecting;
    elements[3] = all_elements[3] = node;

}

Beam *Bellcrank:: getInboardBeam() {
    return inboard;
}

Beam *Bellcrank:: getOutboardBeam() {
    return outboard;
}

Beam *Bellcrank:: getConnectingBeam() {
    return connecting_beam;
}

double *Bellcrank:: getBellcrankDirection() {
    return direction;
}

double *Bellcrank:: getBellcrankAngle() {
    return angle;
}