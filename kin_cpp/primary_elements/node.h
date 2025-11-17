#ifndef NODE_H
#define NODE_H

#include <blaze/Math.h>
#include <vector>
#include <functional>
#include <memory>
#include <optional>
using namespace blaze;

// Forward declaration
class Node;

// Updateable interface for listeners
class Updateable {
public:
    virtual ~Updateable() = default;
    virtual void update() = 0;
};

class Node {
public:
    // x, y, z coordinates
    StaticVector<double, 3UL> position;
    StaticVector<double, 3UL> initial_position;

    // Constructor that takes an array of positions
    Node(const StaticVector<double, 3UL> &pos);
    
    // Core methods
    void reset();
    void translate(const StaticVector<double, 3UL> &translation);
    
    // Rotation methods - two modes
    void rotate(const Node &origin, 
                bool persistent = false,
                const std::optional<StaticVector<double, 3UL>> &direction = std::nullopt,
                const std::optional<double> &angle = std::nullopt,
                const std::optional<double> &ang_x = std::nullopt,
                const std::optional<double> &ang_y = std::nullopt,
                const std::optional<double> &ang_z = std::nullopt);
    
    // Legacy method for backward compatibility
    void flatten_rotate(const StaticVector<double, 3UL> &rotation);
    
    // Child node management
    void add_child(Node *node);
    
    // Listener management
    void add_listener(Updateable *listener);
    
    // Operator overloads
    Node operator+(const Node &other) const;
    Node operator-(const Node &other) const;
    Node operator*(double num) const;
    Node operator/(double num) const;
    
    // Indexing operators
    double operator[](size_t index) const;
    double& operator[](size_t index);
    
    // Mirroring methods (properties in Python)
    Node mirrored_xy() const;
    Node mirrored_xz() const;
    Node mirrored_yz() const;
    
    // String representation
    std::string to_string() const;

private:
    // Child nodes
    std::vector<Node*> child_nodes;
    
    // Listeners
    std::vector<Updateable*> listeners;
    
    // Track previous translation
    std::optional<StaticVector<double, 3UL>> translation;
    
    // Track previous rotation
    std::optional<double> rotation_angle;
    std::optional<const Node*> rotation_origin;
    std::optional<StaticVector<double, 3UL>> rotation_direction;
    
    // Helper method to update listeners
    void update_listeners();
};

#endif // NODE_H