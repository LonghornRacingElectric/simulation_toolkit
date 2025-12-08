#include "spring.h"
#include "../primary_elements/node.h"
#include <stdexcept>
#include <iostream>

Spring::Spring(Node* inboard_node, Node* outboard_node,
               double free_length, double rate)
    : free_length_value(free_length), rate_value(rate) {
    
    // Validate inputs
    if (free_length <= 0.0) {
        throw std::runtime_error("Spring free_length must be positive");
    }
    if (rate <= 0.0) {
        throw std::runtime_error("Spring rate must be positive");
    }
    if (inboard_node == nullptr || outboard_node == nullptr) {
        throw std::runtime_error("Spring nodes cannot be null");
    }
    
    // Create the beam from the nodes
    spring_beam = new Beam(inboard_node, outboard_node);
}

Beam* Spring::getBeam() const {
    return spring_beam;
}

Node* Spring::getInboardNode() const {
    return spring_beam->getInboardNode();
}

Node* Spring::getOutboardNode() const {
    return spring_beam->getOutboardNode();
}

double Spring::getFreeLength() const {
    return free_length_value;
}

double Spring::getRate() const {
    return rate_value;
}

double Spring::getCompliance() const {
    // In Python, compliance is the same as rate
    // This method provides compatibility with Python terminology
    return rate_value;
}

void Spring::setFreeLength(double free_length) {
    if (free_length <= 0.0) {
        throw std::runtime_error("Spring free_length must be positive");
    }
    free_length_value = free_length;
}

void Spring::setRate(double rate) {
    if (rate <= 0.0) {
        throw std::runtime_error("Spring rate must be positive");
    }
    rate_value = rate;
}

double Spring::compression() const {
    // Python: comp = self.free_length - self.length
    double current_length = spring_beam->length();
    double comp = free_length_value - current_length;
    
    // Python: if comp < 0: warnings.warn("Requested coil spring is in tension")
    // In C++, we'll just return the value (negative means tension)
    // The isInTension() method can be used to check this
    if (comp < 0.0) {
        // Optionally output a warning (matching Python behavior)
        // std::cerr << "Warning: Spring is in tension (compression < 0)" << std::endl;
    }
    
    return comp;
}

double Spring::force() const {
    // Python: return self.compliance * self.compression
    // Note: In Python, compliance is the same as rate
    return rate_value * compression();
}

bool Spring::isInTension() const {
    return compression() < 0.0;
}

