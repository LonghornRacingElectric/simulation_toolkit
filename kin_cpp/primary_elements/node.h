#ifndef NODE_H
#define NODE_H

#include <array>

using namespace std;

class Node {
public:
    // x, y, z coordinates
    array<double, 3> position;
    array<double, 3> initial_position;

    // Constructor that takes an array of positions
    Node (array<double, 3>);
    void reset ();
    void translate(array<double, 3>); //parameter : [x_shift, y_shift, z_shift]
    void flatten_rotate(array<double, 3>); //parameter : [x_rot, y_rot, z_rot]
    void plot_elements();
};

#endif // NODE_H