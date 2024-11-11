#ifndef NODE_H
#define NODE_H

#include <blaze/Math.h>
using namespace blaze;

class Node {
public:
    // x, y, z coordinates
    StaticVector<double, 3UL> position;
    StaticVector<double, 3UL> initial_position;

    // Constructor that takes an array of positions
    Node (const StaticVector<double, 3UL> &);
    void reset ();
    void translate(const StaticVector<double, 3UL> &); //parameter : [x_shift, y_shift, z_shift]
    void flatten_rotate(const StaticVector<double, 3UL> &); //parameter : [x_rot, y_rot, z_rot]
    void plot_elements();
};

#endif // NODE_H