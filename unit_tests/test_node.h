#define TEST_NODE_H
#include <blaze/Math.h>
#include <kin_cpp/primary_elements/node.h>
#include "../assets/misc_linalg.h"

using namespace blaze;

class testNode {
public:
    // Constructor
    testNode()
        : test_node(StaticVector<double, 3UL>({0, 0, 0})) {}

    // reset test node back to position {0,0,0} 
    void resetTestPosition() {
        test_node.position = StaticVector<double, 3UL>({0, 0, 0});
    }

    void testNodePosition();
    void testNodeType();
    void testNodeTranslatePosition();
    void testNodeTranslateType();
    void testNodeTranslateInitialPosition();
    void testNodeRotateBase();
    void testNodeRotateEdge();
    void testNodeResetPosition();
    void testNodeResetType();
private:
    // Member Variables
    Node test_node;
};
#endif