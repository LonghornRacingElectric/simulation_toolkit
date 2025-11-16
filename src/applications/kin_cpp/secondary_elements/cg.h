#ifndef CG_H
#define CG_H

#include "../primary_elements/node.h"

class CG {
public:
    CG (Node *);
    Node *getPosition() const;
    const StaticVector<double, 3UL> &getDirection() const;
private:
    Node *position;
    StaticVector<double, 3UL> direction;
};
#endif //CG_H