#include "node.h"
#include "../assets/misc_linalg.h"
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
using namespace blaze;

/* Constructor for Node object 
   Parameters: pos (3-element StaticVector) -- x, y, z */
Node::Node(const StaticVector<double, 3UL> &pos) {
    // Copy coordinates into position - blaze does deep copy on assign
    position = pos;
    initial_position = pos;
}

// Function Reset: resets the current position array to the initial position
void Node::reset() {
    position = initial_position;
    
    translation = std::nullopt;
    
    rotation_angle = std::nullopt;
    rotation_origin = std::nullopt;
    rotation_direction = std::nullopt;
    
    // Reset all child nodes
    for (Node* child : child_nodes) {
        child->reset();
    }
    
    // Note: update_listeners() is commented out in Python version
    // update_listeners();
}

void Node::translate(const StaticVector<double, 3UL> &translation_vec) {
    reset();
    
    position += translation_vec;
    
    // Translate all child nodes
    for (Node* child : child_nodes) {
        child->position += translation_vec;
        child->translation = translation_vec;
    }
    
    translation = translation_vec;
    
    update_listeners();
}

void Node::rotate(const Node &origin, 
                  bool persistent,
                  const std::optional<StaticVector<double, 3UL>> &direction,
                  const std::optional<double> &angle,
                  const std::optional<double> &ang_x,
                  const std::optional<double> &ang_y,
                  const std::optional<double> &ang_z) {
    
    if (!persistent) {
        reset();
    }
    
    // Check which rotation mode to use
    bool use_direction_angle = direction.has_value() && angle.has_value();
    bool use_xyz_angles = ang_x.has_value() && ang_y.has_value() && ang_z.has_value();
    
    if (use_direction_angle && (ang_x.has_value() || ang_y.has_value() || ang_z.has_value())) {
        throw std::runtime_error("You cannot provide ang_x, ang_y, or ang_z to Node.rotate() if direction and angle are also provided.");
    }
    
    if (use_direction_angle) {
        // Mode 1: Rotate about axis defined by direction and angle
        StaticMatrix<double, 3UL, 3UL> rot = rotation_matrix(direction.value(), angle.value());
        
        // Calculate relative position from origin
        StaticVector<double, 3UL> relative_pos = position - origin.position;
        StaticVector<double, 3UL> self_rotated = rot * relative_pos;
        position = self_rotated + origin.position;
        
        // Rotate all child nodes
        for (Node* child : child_nodes) {
            StaticVector<double, 3UL> child_relative = child->position - origin.position;
            StaticVector<double, 3UL> child_rotated = rot * child_relative;
            child->position = child_rotated + origin.position;
            
            child->rotation_angle = angle.value();
            child->rotation_origin = &origin;
            child->rotation_direction = direction.value();
            child->update_listeners();
        }
        
        rotation_angle = angle.value();
        rotation_origin = &origin;
        rotation_direction = direction.value();
        
    } else if (use_xyz_angles) {
        // Mode 2: Rotate by Euler angles (ang_x, ang_y, ang_z)
        StaticMatrix<double, 3UL, 3UL> x_rot = rotation_matrix(StaticVector<double, 3UL>{1, 0, 0}, ang_x.value());
        StaticMatrix<double, 3UL, 3UL> y_rot = rotation_matrix(StaticVector<double, 3UL>{0, 1, 0}, ang_y.value());
        StaticMatrix<double, 3UL, 3UL> z_rot = rotation_matrix(StaticVector<double, 3UL>{0, 0, 1}, ang_z.value());
        
        // Calculate relative position from origin
        StaticVector<double, 3UL> relative_pos = position - origin.position;
        StaticVector<double, 3UL> self_rotated = z_rot * (y_rot * (x_rot * relative_pos));
        position = self_rotated + origin.position;
        
        // Rotate all child nodes
        for (Node* child : child_nodes) {
            StaticVector<double, 3UL> child_relative = child->position - origin.position;
            StaticVector<double, 3UL> child_rotated = z_rot * (y_rot * (x_rot * child_relative));
            child->position = child_rotated + origin.position;
            
            // Note: Python version raises NotImplementedError for listeners with xyz rotations
            // We'll skip updating listeners for child nodes in this mode
        }
        
    } else {
        throw std::runtime_error("You must provide either: (direction and angle) OR (ang_x and ang_y and ang_z) to Node.rotate().");
    }
    
    update_listeners();
}

// Legacy method for backward compatibility
void Node::flatten_rotate(const StaticVector<double, 3UL> &rotation) {
    StaticMatrix<double, 3UL, 3UL> x_rot = rotation_matrix(StaticVector<double, 3UL>{1, 0, 0}, rotation[0]);
    StaticMatrix<double, 3UL, 3UL> y_rot = rotation_matrix(StaticVector<double, 3UL>{0, 1, 0}, rotation[1]);
    StaticMatrix<double, 3UL, 3UL> z_rot = rotation_matrix(StaticVector<double, 3UL>{0, 0, 1}, rotation[2]);

    // Apply the rotation: z_rot * y_rot * x_rot * position
    position = z_rot * (y_rot * (x_rot * position));
}

void Node::add_child(Node *node) {
    child_nodes.push_back(node);
}

void Node::add_listener(Updateable *listener) {
    listeners.push_back(listener);
}

Node Node::operator+(const Node &other) const {
    StaticVector<double, 3UL> node_sum = position + other.position;
    return Node(node_sum);
}

Node Node::operator-(const Node &other) const {
    StaticVector<double, 3UL> node_sub = position - other.position;
    return Node(node_sub);
}

Node Node::operator*(double num) const {
    StaticVector<double, 3UL> node_mul = position * num;
    return Node(node_mul);
}

Node Node::operator/(double num) const {
    if (std::abs(num) < 1e-10) {
        throw std::runtime_error("Division by zero");
    }
    StaticVector<double, 3UL> node_div = position / num;
    return Node(node_div);
}

double Node::operator[](size_t index) const {
    if (index >= 3) {
        throw std::out_of_range("Node index out of range");
    }
    return position[index];
}

double& Node::operator[](size_t index) {
    if (index >= 3) {
        throw std::out_of_range("Node index out of range");
    }
    update_listeners();  // Update listeners when position is modified
    return position[index];
}

Node Node::mirrored_xy() const {
    StaticVector<double, 3UL> mirrored_pos{
        initial_position[0],
        initial_position[1],
        -initial_position[2]
    };
    return Node(mirrored_pos);
}

Node Node::mirrored_xz() const {
    StaticVector<double, 3UL> mirrored_pos{
        initial_position[0],
        -initial_position[1],
        initial_position[2]
    };
    return Node(mirrored_pos);
}

Node Node::mirrored_yz() const {
    StaticVector<double, 3UL> mirrored_pos{
        -initial_position[0],
        initial_position[1],
        initial_position[2]
    };
    return Node(mirrored_pos);
}

std::string Node::to_string() const {
    std::ostringstream oss;
    oss << "Current position: [" << position[0] << ", " << position[1] << ", " << position[2] << "] | "
        << "Initial position: [" << initial_position[0] << ", " << initial_position[1] << ", " << initial_position[2] << "]";
    return oss.str();
}

void Node::update_listeners() {
    for (Updateable* listener : listeners) {
        listener->update();
    }
}
