#ifndef SPRING_H
#define SPRING_H

#include "../primary_elements/beam.h"

using namespace blaze;

class Spring {
public:
    /**
     * Spring constructor
     * 
     * @param inboard_node Node representing inboard end of spring
     * @param outboard_node Node representing outboard end of spring
     * @param free_length Free length of spring (unloaded length)
     * @param rate Spring rate (force per unit compression, N/m)
     */
    Spring(Node* inboard_node, Node* outboard_node, 
           double free_length, double rate);
    
    /**
     * Get the spring compression
     * Compression = free_length - current_length
     * 
     * @return Spring compression (positive when compressed, negative when in tension)
     */
    double compression() const;
    
    /**
     * Get the spring force
     * Force = rate * compression
     * 
     * @return Spring force (positive when compressed)
     */
    double force() const;
    
    // Getters
    Beam* getBeam() const;
    Node* getInboardNode() const;
    Node* getOutboardNode() const;
    double getFreeLength() const;
    double getRate() const;
    double getCompliance() const;  // Alias for rate (matching Python terminology)
    
    // Setters
    void setFreeLength(double free_length);
    void setRate(double rate);
    
    /**
     * Check if spring is in tension (compression < 0)
     * 
     * @return true if spring is in tension, false otherwise
     */
    bool isInTension() const;
    
private:
    Beam* spring_beam;
    double free_length_value;  // Free length of spring
    double rate_value;         // Spring rate (force per unit compression)
};

#endif // SPRING_H

