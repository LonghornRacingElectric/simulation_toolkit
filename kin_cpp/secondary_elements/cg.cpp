#include "cg.h"
#include "../primary_elements/node.h"

using namespace blaze;

/**
 * Constructor
 * 
 * Initializes the CG with a Node pointer representing the center of gravity position.
 * This matches the Python implementation where CG_node is a Node object.
 * 
 * @param cg_node Pointer to the Node representing the center of gravity position
 */
CG::CG(Node *cg_node) {
    this->cg_node = cg_node;
}

/**
 * Get the CG node (equivalent to Python's CG_node)
 * 
 * @return Pointer to the CG Node
 */
Node* CG::getNode() const {
    return cg_node; 
}

/**
 * Get the CG position (convenience method)
 * 
 * @return Reference to the position vector of the CG node
 */
const StaticVector<double, 3UL>& CG::getPosition() const {
    return cg_node->position;
}

/**
 * Get pointer to the CG node (alias for getNode for consistency)
 * 
 * @return Pointer to the CG Node
 */
Node* CG::getCGNode() const {
    return cg_node;
}
