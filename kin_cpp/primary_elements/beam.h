#ifndef BEAM_H
#define BEAM_H

#include "node.h"
#include <optional>
#include <string>

using namespace blaze;

class Beam {
public:
    // Constructor
    Beam(Node *inboard_node, Node *outboard_node);
    
    // Getters
    Node* getInboardNode() const;
    Node* getOutboardNode() const;
    
    // Intersection methods
    Node yz_intersection(const Beam &link) const;
    Node xz_intersection(const Beam &link) const;
    
    // Link-centered coordinates
    StaticVector<double, 3UL> link_centered_coords(const Node &node) const;
    
    // Properties (matching Python properties)
    StaticVector<double, 3UL> direction() const;
    StaticVector<double, 3UL> center() const;
    double radius() const;
    double length() const;
    
    // Component angles and rotation angles
    StaticVector<double, 2UL> component_angles() const;
    StaticVector<double, 2UL> rotation_angles() const;
    
    // Initial length (stored at construction)
    double initial_length() const;
    
    // Compliance (optional)
    std::optional<double> compliance;
    std::optional<std::string> compliance_unit;
    
    // Translation and rotation
    void translate(const StaticVector<double, 3UL> &translation);
    void rotate(const Node &origin,
                bool persistent = false,
                const std::optional<StaticVector<double, 3UL>> &direction = std::nullopt,
                const std::optional<double> &angle = std::nullopt,
                const std::optional<double> &ang_x = std::nullopt,
                const std::optional<double> &ang_y = std::nullopt,
                const std::optional<double> &ang_z = std::nullopt);
    // Legacy method for backward compatibility
    void flatten_rotate(const StaticVector<double, 3UL> &rotation);

private:
    Node *inboard_node;
    Node *outboard_node;
    double initial_length_value;
    
    // For backward compatibility with existing code
    Node *elements[2];
    Node *all_elements[2];
    bool plotted;
};

#endif // BEAM_H
