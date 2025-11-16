#include "cg.h"

using namespace blaze;

CG::CG (Node *pos) {
    position = pos;
    direction = StaticVector<double, 3UL> ({0, 0, 1});
}
Node *CG::getPosition() const {
    return position;
}

const StaticVector<double, 3UL> &CG::getDirection() const {
    return direction;
}