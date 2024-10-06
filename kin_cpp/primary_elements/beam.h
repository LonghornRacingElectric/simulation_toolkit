#ifndef BEAM_H
#define BEAM_H

#include <iostream>
#include "node.h"

class Beam {
public:
    Beam(Node *inboard, Node *outboard);

    Node *getInboardNode();
    Node *getOutboardNode();

private:
    Node *inboard_node;
    Node *outboard_node;
    Node *elements[2];
    Node *all_elements[2];
    bool plotted;
};

#endif