#include "test_bellcrank.h"
#include <cmath>
#include <vector>

using namespace blaze;

// Test: Bellcrank initialization
TEST_F(TestBellcrank, test_bellcrank_init) {
    Node node_one(StaticVector<double, 3UL>{0, 1, 0});
    Node node_two(StaticVector<double, 3UL>{0, 1, 1});
    Node node_three(StaticVector<double, 3UL>{0, 0, 1});
    
    std::vector<Node*> pickup_nodes = {&node_one, &node_two, &node_three};
    
    Node pivot_node(StaticVector<double, 3UL>{0, 0, 0});
    
    StaticVector<double, 3UL> pivot_direction{1, 0, 0};
    
    Bellcrank bellcrank(pickup_nodes, &pivot_node, pivot_direction);
    
    const std::vector<Node*>& bellcrank_nodes = bellcrank.getNodes();
    for (size_t index = 0; index < pickup_nodes.size(); ++index) {
        EXPECT_EQ(bellcrank_nodes[index], pickup_nodes[index]);
    }
}

// Test: Bellcrank rotation by π/2
TEST_F(TestBellcrank, test_bellcrank_rotate_one) {
    Node node_one(StaticVector<double, 3UL>{0, 1, 0});
    Node node_two(StaticVector<double, 3UL>{0, 1, 1});
    Node node_three(StaticVector<double, 3UL>{0, 0, 1});
    
    Node pivot_node(StaticVector<double, 3UL>{0, 0, 0});
    
    StaticVector<double, 3UL> pivot_direction{1, 0, 0};
    
    std::vector<Node*> pickup_nodes = {&node_one, &node_two, &node_three};
    Bellcrank bellcrank(pickup_nodes, &pivot_node, pivot_direction);
    
    bellcrank.rotate(M_PI / 2.0);
    
    std::vector<StaticVector<double, 3UL>> new_positions = {
        StaticVector<double, 3UL>{0, 0, 1},
        StaticVector<double, 3UL>{0, -1, 1},
        StaticVector<double, 3UL>{0, -1, 0}
    };
    
    const std::vector<Node*>& bellcrank_nodes = bellcrank.getNodes();
    for (size_t index = 0; index < bellcrank_nodes.size(); ++index) {
        StaticVector<double, 3UL> rounded_pos = roundVector(bellcrank_nodes[index]->position, 3);
        expectVectorNear(rounded_pos, new_positions[index], 1e-3);
    }
}

// Test: Bellcrank rotation by -π/2
TEST_F(TestBellcrank, test_bellcrank_rotate_two) {
    Node node_one(StaticVector<double, 3UL>{0, 1, 0});
    Node node_two(StaticVector<double, 3UL>{0, 1, 1});
    Node node_three(StaticVector<double, 3UL>{0, 0, 1});
    
    Node pivot_node(StaticVector<double, 3UL>{0, 0, 0});
    
    StaticVector<double, 3UL> pivot_direction{1, 0, 0};
    
    std::vector<Node*> pickup_nodes = {&node_one, &node_two, &node_three};
    Bellcrank bellcrank(pickup_nodes, &pivot_node, pivot_direction);
    
    bellcrank.rotate(-M_PI / 2.0);
    
    std::vector<StaticVector<double, 3UL>> new_positions = {
        StaticVector<double, 3UL>{0, 0, -1},
        StaticVector<double, 3UL>{0, 1, -1},
        StaticVector<double, 3UL>{0, 1, 0}
    };
    
    const std::vector<Node*>& bellcrank_nodes = bellcrank.getNodes();
    for (size_t index = 0; index < bellcrank_nodes.size(); ++index) {
        StaticVector<double, 3UL> rounded_pos = roundVector(bellcrank_nodes[index]->position, 3);
        expectVectorNear(rounded_pos, new_positions[index], 1e-3);
    }
}

// Test: Bellcrank repeated rotations
TEST_F(TestBellcrank, test_bellcrank_rotate_repeated) {
    Node node_one(StaticVector<double, 3UL>{0, 1, 0});
    Node node_two(StaticVector<double, 3UL>{0, 1, 1});
    Node node_three(StaticVector<double, 3UL>{0, 0, 1});
    
    Node pivot_node(StaticVector<double, 3UL>{0, 0, 0});
    
    StaticVector<double, 3UL> pivot_direction{1, 0, 0};
    
    std::vector<Node*> pickup_nodes = {&node_one, &node_two, &node_three};
    Bellcrank bellcrank(pickup_nodes, &pivot_node, pivot_direction);
    
    bellcrank.rotate(-M_PI / 2.0);
    bellcrank.rotate(M_PI / 2.0);
    
    std::vector<StaticVector<double, 3UL>> new_positions = {
        StaticVector<double, 3UL>{0, 0, 1},
        StaticVector<double, 3UL>{0, -1, 1},
        StaticVector<double, 3UL>{0, -1, 0}
    };
    
    const std::vector<Node*>& bellcrank_nodes = bellcrank.getNodes();
    for (size_t index = 0; index < bellcrank_nodes.size(); ++index) {
        StaticVector<double, 3UL> rounded_pos = roundVector(bellcrank_nodes[index]->position, 3);
        expectVectorNear(rounded_pos, new_positions[index], 1e-3);
    }
}

// Test: Bellcrank rotation with child nodes
TEST_F(TestBellcrank, test_bellcrank_rotate_child) {
    Node node_one(StaticVector<double, 3UL>{0, 1, 0});
    Node node_two(StaticVector<double, 3UL>{0, 1, 1});
    Node node_three(StaticVector<double, 3UL>{0, 0, 1});
    
    Node pivot_node(StaticVector<double, 3UL>{0, 0, 0});
    
    StaticVector<double, 3UL> pivot_direction{1, 0, 0};
    
    Node child_node(StaticVector<double, 3UL>{0, 0.5, 0});
    node_one.add_child(&child_node);
    
    std::vector<Node*> pickup_nodes = {&node_one, &node_two, &node_three};
    Bellcrank bellcrank(pickup_nodes, &pivot_node, pivot_direction);
    
    bellcrank.rotate(M_PI / 2.0);
    
    StaticVector<double, 3UL> expected_child_pos{0, 0, 0.5};
    StaticVector<double, 3UL> rounded_child_pos = roundVector(child_node.position, 7);
    expectVectorNear(rounded_child_pos, expected_child_pos, 1e-7);
}

