#ifndef BELLCRANK_H
#define BELLCRANK_H

#include "../primary_elements/node.h"
#include <vector>
#include <blaze/Math.h>

using namespace blaze;

class Bellcrank {
public:
    /**
     * Bellcrank constructor
     * 
     * @param nodes All distinct pickup Nodes on bellcrank
     *              - First entry should connect to push/pull rod
     *              - Final entry should connect to inboard rod/shock
     * @param pivot Bellcrank pivot Node
     * @param pivot_direction Unit vector representing Bellcrank pivot axis
     */
    Bellcrank(const std::vector<Node*>& nodes, Node* pivot, const StaticVector<double, 3UL>& pivot_direction);

    /**
     * Rotates bellcrank
     * 
     * @param angle Angle of rotation in radians
     */
    void rotate(double angle);

    // Getters
    const std::vector<Node*>& getNodes() const;
    Node* getPivot() const;
    const StaticVector<double, 3UL>& getPivotDirection() const;
    double getAngle() const;

private:
    std::vector<Node*> nodes;  // All distinct pickup Nodes on bellcrank
    Node* pivot;                // Bellcrank pivot Node
    StaticVector<double, 3UL> pivot_direction;  // Unit vector for pivot axis
    double angle;               // Current rotation angle
};
#endif //BELLCRANK_H