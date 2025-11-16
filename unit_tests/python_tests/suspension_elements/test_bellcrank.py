from src.vehicle_model.suspension_model.suspension_elements._2_elements.bellcrank import Bellcrank
from src.vehicle_model.suspension_model.suspension_elements._1_elements.node import Node

from unittest import TestCase
import numpy as np


class TestBellcrank(TestCase):
    def test_bellcrank_init(self):
        node_one = Node(position=[0, 1, 0])
        node_two = Node(position=[0, 1, 1])
        node_three = Node(position=[0, 0, 1])
        pickup_nodes = [node_one, node_two, node_three]

        pivot_node = Node(position=[0, 0, 0])

        pivot_direction = [1, 0, 0]

        bellcrank = Bellcrank(node_one, node_two, node_three, pivot=pivot_node, pivot_direction=pivot_direction)

        for index, node in enumerate(bellcrank.nodes):
            self.assertIs(node, pickup_nodes[index])
        
    def test_bellcrank_rotate_one(self):
        node_one = Node(position=[0, 1, 0])
        node_two = Node(position=[0, 1, 1])
        node_three = Node(position=[0, 0, 1])

        pivot_node = Node(position=[0, 0, 0])

        pivot_direction = [1, 0, 0]

        bellcrank = Bellcrank(node_one, node_two, node_three, pivot=pivot_node, pivot_direction=pivot_direction)
        bellcrank.rotate(angle=np.pi.__float__() / 2)

        new_positions = [[0, 0, 1], [0, -1, 1], [0, -1, 0]]

        for index, node in enumerate(bellcrank.nodes):
            self.assertListEqual([round(x, 3) for x in node.position], new_positions[index])
    
    def test_bellcrank_rotate_two(self):
        node_one = Node(position=[0, 1, 0])
        node_two = Node(position=[0, 1, 1])
        node_three = Node(position=[0, 0, 1])

        pivot_node = Node(position=[0, 0, 0])

        pivot_direction = [1, 0, 0]

        bellcrank = Bellcrank(node_one, node_two, node_three, pivot=pivot_node, pivot_direction=pivot_direction)
        bellcrank.rotate(angle=-np.pi.__float__() / 2)

        new_positions = [[0, 0, -1], [0, 1, -1], [0, 1, 0]]

        for index, node in enumerate(bellcrank.nodes):
            self.assertListEqual([round(x, 3) for x in node.position], new_positions[index])
    
    def test_bellcrank_rotate_repeated(self):
        node_one = Node(position=[0, 1, 0])
        node_two = Node(position=[0, 1, 1])
        node_three = Node(position=[0, 0, 1])

        pivot_node = Node(position=[0, 0, 0])

        pivot_direction = [1, 0, 0]

        bellcrank = Bellcrank(node_one, node_two, node_three, pivot=pivot_node, pivot_direction=pivot_direction)
        bellcrank.rotate(angle=-np.pi.__float__() / 2)
        bellcrank.rotate(angle=np.pi.__float__() / 2)

        new_positions = [[0, 0, 1], [0, -1, 1], [0, -1, 0]]

        for index, node in enumerate(bellcrank.nodes):
            self.assertListEqual([round(x, 3) for x in node.position], new_positions[index])
    
    def test_bellcrank_rotate_child(self):
        node_one = Node(position=[0, 1, 0])
        node_two = Node(position=[0, 1, 1])
        node_three = Node(position=[0, 0, 1])

        pivot_node = Node(position=[0, 0, 0])

        pivot_direction = [1, 0, 0]

        child_node = Node(position=[0, 0.5, 0])
        node_one.add_child(node=child_node)

        bellcrank = Bellcrank(node_one, node_two, node_three, pivot=pivot_node, pivot_direction=pivot_direction)
        bellcrank.rotate(angle=np.pi.__float__() / 2)

        self.assertListEqual([round(x, 7) for x in child_node.position], [0, 0, 0.5])