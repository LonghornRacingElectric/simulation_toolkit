from vehicle_model.suspension_model.suspension_elements.primary_elements.link import Link
from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
import numpy as np

from unittest import main, TestCase


class LinkTest(TestCase):
    def test_link_init_direction_value(self):
        inboard = Node(position=[-1, 0, 0])
        outboard = Node(position=[1, 0, 0])
        test_link = Link(inboard_node=inboard, outboard_node=outboard)

        self.assertListEqual(test_link.direction, [1, 0, 0])
    
    def test_link_init_center_value(self):
        inboard = Node(position=[-3, 0, 0])
        outboard = Node(position=[1, 0, 0])
        test_link = Link(inboard_node=inboard, outboard_node=outboard)

        self.assertListEqual(test_link.center, [-1, 0, 0])

    def test_link_init_length_value(self):
        inboard = Node(position=[-3, 0, 0])
        outboard = Node(position=[1, 0, 0])
        test_link = Link(inboard_node=inboard, outboard_node=outboard)

        self.assertEqual(test_link.length, 4)

    def test_link_yz_intersection_base(self):
        inboard_1 = Node(position=[0, 1, 1])
        outboard_1 = Node(position=[0, 2, 2])
        test_link_1 = Link(inboard_node=inboard_1, outboard_node=outboard_1)

        inboard_2 = Node(position=[0, 0, 2])
        outboard_2 = Node(position=[0, -1, 3])
        test_link_2 = Link(inboard_node=inboard_2, outboard_node=outboard_2)

        intersection = test_link_1.yz_intersection(test_link_2)

        self.assertListEqual(intersection.position, [0, 1, 1])
    
    def test_link_yz_intersection_edge_one(self):
        inboard_1 = Node(position=[0, 1, 1])
        outboard_1 = Node(position=[0, 2, 1])
        test_link_1 = Link(inboard_node=inboard_1, outboard_node=outboard_1)

        inboard_2 = Node(position=[0, 1, 0])
        outboard_2 = Node(position=[0, 2, 0])
        test_link_2 = Link(inboard_node=inboard_2, outboard_node=outboard_2)

        intersection = test_link_1.yz_intersection(test_link_2)

        self.assertListEqual(intersection.position, [0, np.inf, 0.5])
    
    def test_link_yz_intersection_edge_two(self):
        inboard_1 = Node(position=[0, 1, 1])
        outboard_1 = Node(position=[0, 2, 1])
        test_link_1 = Link(inboard_node=inboard_1, outboard_node=outboard_1)

        inboard_2 = Node(position=[0, 1, 1])
        outboard_2 = Node(position=[0, 2, 1])
        test_link_2 = Link(inboard_node=inboard_2, outboard_node=outboard_2)

        intersection = test_link_1.yz_intersection(test_link_2)
        
        self.assertListEqual(intersection.position, [0, np.inf, 1])

    def test_link_xz_intersection(self):
        inboard_1 = Node(position=[1, 0, 1])
        outboard_1 = Node(position=[2, 0, 2])
        test_link_1 = Link(inboard_node=inboard_1, outboard_node=outboard_1)

        inboard_2 = Node(position=[0, 0, 2])
        outboard_2 = Node(position=[-1, 0, 3])
        test_link_2 = Link(inboard_node=inboard_2, outboard_node=outboard_2)

        intersection = test_link_1.xz_intersection(test_link_2)

        self.assertListEqual(intersection.position, [1, 0, 1])
    
    def test_link_xz_intersection_edge_one(self):
        inboard_1 = Node(position=[1, 0, 1])
        outboard_1 = Node(position=[2, 0, 1])
        test_link_1 = Link(inboard_node=inboard_1, outboard_node=outboard_1)

        inboard_2 = Node(position=[1, 0, 0])
        outboard_2 = Node(position=[2, 0, 0])
        test_link_2 = Link(inboard_node=inboard_2, outboard_node=outboard_2)

        intersection = test_link_1.xz_intersection(test_link_2)

        self.assertListEqual(intersection.position, [np.inf, 0, 0.5])
    
    def test_link_xz_intersection_edge_two(self):
        inboard_1 = Node(position=[1, 0, 1])
        outboard_1 = Node(position=[2, 0, 1])
        test_link_1 = Link(inboard_node=inboard_1, outboard_node=outboard_1)

        inboard_2 = Node(position=[1, 0, 1])
        outboard_2 = Node(position=[2, 0, 1])
        test_link_2 = Link(inboard_node=inboard_2, outboard_node=outboard_2)

        intersection = test_link_1.xz_intersection(test_link_2)
        
        self.assertListEqual(intersection.position, [np.inf, 0, 1])
    
    def test_link_centered_coords(self):
        lower_link_node = Node(position=[1, 1, 0])
        upper_link_node = Node(position=[2, 2, 1])

        ref_node = Node(position=[2, 2, 1])

        test_link = Link(inboard_node=lower_link_node, outboard_node=upper_link_node)
        test_coord_output = test_link.link_centered_coords(node=ref_node)

        self.assertListEqual([round(x, 3) for x in test_coord_output], [0, 0, round(np.sqrt(3),3)])