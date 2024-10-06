#include <iostream>
using namespace std;

// Defines Node as basic unit of position
class Node {
private:
    //x, y, z coordinates
    double position[];

public:
    Node (double pos[]) {
        //ensure pos passes in x, y z
        assert (sizeof (pos) / sizeof (double) == 3);
        position = pos;
    }
}