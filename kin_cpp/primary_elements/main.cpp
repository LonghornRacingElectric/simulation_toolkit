#include <iostream>
#include "beam.h"

using namespace std;

int main(int argc, char const *argv[])
{
    double pos[3] = {1, 2, 3};
    Node n1(pos);
    double pos2[3] = {4, 5, 6};
    Node n2(pos2);

    Beam b1(&n1, &n2);

    cout << b1.getInboardNode() << " " << b1.getOutboardNode();
    return 0;
}
