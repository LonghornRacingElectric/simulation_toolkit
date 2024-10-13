#ifndef BELLCRANK_H
#define BELLCRANK_H

#include <iostream>
#include "../primary_elements/node.h"
#include "../primary_elements/beam.h"

class Bellcrank {
public:
    Bellcrank(Beam *inboard_beam, Beam *outboard_beam, Beam *connecting_beam, Node *node, double *bellcrank_direction, double *bellcrank_angle);

    // Functions for getting inboard, outboard, and connecting beams
    Beam *getInboardBeam();
    Beam *getOutboardBeam();
    Beam *getConnectingBeam();

    double *getBellcrankDirection();
    double *getBellcrankAngle();

private:
    Node *bellcrank_pivot;
    Node *elements[4];
    Node *all_elements[4];
    double *bellcrank_angle;
    double *bellcrank_direction;

};