#include <iostream>
using namespace std;

// Defines Node as basic unit of position
class Node {
private:
    //x, y, z coordinates
    double position[3];

public:
    Node (double pos[]) {
        //copy coordinates into position
        for (int i = 0; i < 3; i++)
        {
            position[i] = pos[i];
        } 
    }
};