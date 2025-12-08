#ifndef CG_H
#define CG_H

#include "../primary_elements/node.h"
#include <blaze/Math.h>

using namespace blaze;

/**
 * CG class represents the center of gravity of a vehicle.
 * 
 * In the Python implementation, CG is stored as a Node object (CG_node).
 * This C++ class wraps a Node pointer to match that functionality.
 * 
 * The CG position is calculated from tire contact patches and mass properties:
 * - cg_x: Interpolated between front-left and rear-left tire positions using CGBX
 * - cg_y: Interpolated between front-left and front-right tire positions using CGBY  
 * - cg_z: Direct value from mass properties (CGZ)
 */
class CG {
public:
    /**
     * Constructor
     * 
     * @param cg_node Pointer to the Node representing the center of gravity position
     */
    CG(Node *cg_node);
    
    /**
     * Get the CG node (equivalent to Python's CG_node)
     * 
     * @return Pointer to the CG Node
     */
    Node* getNode() const;
    
    /**
     * Get the CG position (convenience method)
     * 
     * @return Reference to the position vector of the CG node
     */
    const StaticVector<double, 3UL>& getPosition() const;
    
    /**
     * Get pointer to the CG node (alias for getNode for consistency)
     * 
     * @return Pointer to the CG Node
     */
    Node* getCGNode() const;

private:
    Node *cg_node;  // Pointer to the Node representing center of gravity
};

#endif // CG_H
