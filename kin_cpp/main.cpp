#include <iostream>
#include "./primary_elements/node.h"
#include "./primary_elements/beam.h"
#include "./secondary_elements/tie.h"

using namespace std;

int main() {
    array<double, 3> position1 = {0.0, 0.0, 0.0};
    array<double, 3> position2 = {1.0, 1.0, 1.0};

    Node node1(position1);
    Node node2(position2);
    Beam beam1(&node1, &node2);

    Tie tie1(&beam1);

    cout << tie1.getAngle() << endl;
    cout << tie1.getInboardNode() << endl;
    cout << tie1.getOutboardNode() << endl;
    cout << tie1.getLength() << endl;

    return 0;
}