from vehicle_model.suspension_model.suspension_elements._2_elements.wishbone import Wishbone
from vehicle_model.suspension_model.suspension_elements._1_elements.link import Link
from vehicle_model.suspension_model.suspension_elements._1_elements.node import Node
import numpy as np

from unittest import TestCase


class TestWishbone(TestCase):
    def test_wishbone_base_direction(self):
        inboard_fore = Node(position=[2, 0, 0])
        inboard_aft = Node(position=[0, 0, 0])
        outboard = Node(position=[1, 2, 0])

        fore_link = Link(inboard_node=inboard_fore, outboard_node=outboard)
        aft_link = Link(inboard_node=inboard_aft, outboard_node=outboard)

        wishbone = Wishbone(fore_link=fore_link, aft_link=aft_link)

        self.assertListEqual(wishbone.direction, [1, 0, 0])

    def test_wishbone_base_angle(self):
        inboard_fore = Node(position=[2, 0, 0])
        inboard_aft = Node(position=[0, 0, 0])
        outboard = Node(position=[1, 2, 0])

        fore_link = Link(inboard_node=inboard_fore, outboard_node=outboard)
        aft_link = Link(inboard_node=inboard_aft, outboard_node=outboard)

        wishbone = Wishbone(fore_link=fore_link, aft_link=aft_link)

        self.assertEqual(wishbone.angle, 0)
    
    def test_rotate(self):
        inboard_fore = Node(position=[2, 0, 0])
        inboard_aft = Node(position=[0, 0, 0])
        outboard = Node(position=[1, 2, 0])

        fore_link = Link(inboard_node=inboard_fore, outboard_node=outboard)
        aft_link = Link(inboard_node=inboard_aft, outboard_node=outboard)

        wishbone = Wishbone(fore_link=fore_link, aft_link=aft_link)
        wishbone.rotate(angle=np.pi/2)

        self.assertListEqual([round(x, 7) for x in wishbone.fore_link.outboard_node.position], [1, 0, 2])
    
    def test_child_rotate(self):
        inboard_fore = Node(position=[2, 0, 0])
        inboard_aft = Node(position=[0, 0, 0])
        outboard = Node(position=[1, 2, 0])

        fore_link = Link(inboard_node=inboard_fore, outboard_node=outboard)
        aft_link = Link(inboard_node=inboard_aft, outboard_node=outboard)

        wishbone = Wishbone(fore_link=fore_link, aft_link=aft_link)

        child_node = Node(position=[1, 1, 0])
        outboard.add_child(node=child_node)
        
        wishbone.rotate(angle=np.pi/2)

        self.assertListEqual([round(x, 7) for x in child_node.position], [1, 0, 1])

    def test_plane_xy(self):
        inboard_fore = Node(position=[2, 0, 0])
        inboard_aft = Node(position=[0, 0, 0])
        outboard = Node(position=[1, 2, 0])

        fore_link = Link(inboard_node=inboard_fore, outboard_node=outboard)
        aft_link = Link(inboard_node=inboard_aft, outboard_node=outboard)

        wishbone = Wishbone(fore_link=fore_link, aft_link=aft_link)
        a, b, c, x_0, y_0, z_0 = wishbone.plane

        # Check if planes are equivalent type shit
        point_1 = a * (1 - x_0) + b * (0 - y_0) + c * (0 - z_0)
        point_2 = a * (0 - x_0) + b * (1 - y_0) + c * (0 - z_0)
        point_3 = a * (0 - x_0) + b * (0 - y_0) + c * (0 - z_0)

        self.assertListEqual([point_1, point_2, point_3], [0, 0, 0])

    def test_direction_vec(self):
        inboard_fore = Node(position=[2, 0, 0])
        inboard_aft = Node(position=[0, 0, 0])
        outboard = Node(position=[1, 2, 0])

        fore_link = Link(inboard_node=inboard_fore, outboard_node=outboard)
        aft_link = Link(inboard_node=inboard_aft, outboard_node=outboard)

        wishbone = Wishbone(fore_link=fore_link, aft_link=aft_link)

        self.assertListEqual(wishbone.direction_vec, [1, 0, 0])