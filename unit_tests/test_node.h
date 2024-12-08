#ifndef TEST_NODE_H
#define TEST_NODE_H

#include <blaze/Math.h>
#include <kin_cpp/primary_elements/node.h>
#include "../assets/misc_linalg.h"

using namespace blaze;

class testNode {
public:
    void testNodePosition();
    void testNodeType();
    void testNodeTranslatePosition();
    void testNodeTranslateType();
    void testNodeTranslateInitialPosition();
    void testNodeRotateBase();
    void testNodeResetPosition();
    void testNodeResetType();
};
#endif