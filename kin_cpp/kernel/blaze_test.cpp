#include <blaze/Math.h>
#include <iostream>
#include <cstdio>
#include "../primary_elements/beam.h"
int main() {
    Node a({0, 1, 2});
    Node b({3, 4, 5});
    Beam beam(&a, &b);

    printf("a : %.2f, b : %.2f", beam.getInboardNode()->position[0], beam.getOutboardNode()->position[0]);
    return 0;
}
