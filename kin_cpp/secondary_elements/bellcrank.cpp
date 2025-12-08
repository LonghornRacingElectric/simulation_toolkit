#include "bellcrank.h"
#include "../primary_elements/node.h"
#include <optional>

Bellcrank::Bellcrank(const std::vector<Node*>& nodes, Node* pivot, const StaticVector<double, 3UL>& pivot_direction)
    : nodes(nodes), pivot(pivot), pivot_direction(pivot_direction), angle(0.0) {
}

void Bellcrank::rotate(double rotation_angle) {
    // Rotate each node about the pivot with the pivot direction
    for (Node* node : nodes) {
        node->reset();
        node->rotate(*pivot, false, std::make_optional(pivot_direction), std::make_optional(rotation_angle));
    }
    
    angle = rotation_angle;
}

const std::vector<Node*>& Bellcrank::getNodes() const {
    return nodes;
}

Node* Bellcrank::getPivot() const {
    return pivot;
}

const StaticVector<double, 3UL>& Bellcrank::getPivotDirection() const {
    return pivot_direction;
}

double Bellcrank::getAngle() const {
    return angle;
}