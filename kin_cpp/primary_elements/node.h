#ifndef NODE_H
#define NODE_H

class Node {
public:
    // x, y, z coordinates
    double position[3];

    // Constructor that takes an array of positions
    Node(double pos[]);
};

#endif // NODE_H