from vehicle_model.suspension_model.suspension_elements.primary_elements.link import Link
from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
from vehicle_model.assets.plotter import Plotter
import numpy as np

from unittest import main, TestCase


class LinkTest(TestCase):
    def test_link_init_direction_value(self):
        inboard = Node(position=[-1, 0, 0])
        outboard = Node(position=[1, 0, 0])
        test_link = Link(inboard=inboard, outboard=outboard)

        self.assertListEqual(list(test_link.direction), [1, 0, 0])
    
    def test_link_init_direction_type(self):
        inboard = Node(position=[-1, 0, 0])
        outboard = Node(position=[1, 0, 0])
        test_link = Link(inboard=inboard, outboard=outboard)

        self.assertIsInstance(test_link.direction, np.ndarray)
    
    def test_link_init_center_value(self):
        inboard = Node(position=[-3, 0, 0])
        outboard = Node(position=[1, 0, 0])
        test_link = Link(inboard=inboard, outboard=outboard)

        self.assertListEqual(list(test_link.center), [-1, 0, 0])

    def test_link_init_center_type(self):
        inboard = Node(position=[-3, 0, 0])
        outboard = Node(position=[1, 0, 0])
        test_link = Link(inboard=inboard, outboard=outboard)

        self.assertIsInstance(test_link.center, np.ndarray)

    def test_link_init_height_value(self):
        inboard = Node(position=[-3, 0, 0])
        outboard = Node(position=[1, 0, 0])
        test_link = Link(inboard=inboard, outboard=outboard)

        self.assertEqual(test_link.height, 4)

    def test_link_init_height_type(self):
        inboard = Node(position=[-3, 0, 0])
        outboard = Node(position=[1, 0, 0])
        test_link = Link(inboard=inboard, outboard=outboard)

        self.assertIsInstance(test_link.height, float)

    def test_link_yz_intersection_base(self):
        inboard_1 = Node(position=[0, 1, 1])
        outboard_1 = Node(position=[0, 2, 2])
        test_link_1 = Link(inboard=inboard_1, outboard=outboard_1)

        inboard_2 = Node(position=[0, 0, 2])
        outboard_2 = Node(position=[0, -1, 3])
        test_link_2 = Link(inboard=inboard_2, outboard=outboard_2)

        intersection = test_link_1.yz_intersection(test_link_2)

        self.assertListEqual(list(intersection.position), [0, 1, 1])
    
    def test_link_yz_intersection_edge_1(self):
        inboard_1 = Node(position=[0, 1, 1])
        outboard_1 = Node(position=[0, 2, 1])
        test_link_1 = Link(inboard=inboard_1, outboard=outboard_1)

        inboard_2 = Node(position=[0, 1, 0])
        outboard_2 = Node(position=[0, 2, 0])
        test_link_2 = Link(inboard=inboard_2, outboard=outboard_2)

        intersection = test_link_1.yz_intersection(test_link_2)

        self.assertListEqual(list(intersection.position), [0, np.inf, 0.5])
    
    def test_link_yz_intersection_edge_2(self):
        inboard_1 = Node(position=[0, 1, 1])
        outboard_1 = Node(position=[0, 2, 1])
        test_link_1 = Link(inboard=inboard_1, outboard=outboard_1)

        inboard_2 = Node(position=[0, 1, 1])
        outboard_2 = Node(position=[0, 2, 1])
        test_link_2 = Link(inboard=inboard_2, outboard=outboard_2)

        intersection = test_link_1.yz_intersection(test_link_2)
        
        self.assertListEqual(list(intersection.position), [0, np.inf, 1])

    def test_link_xz_intersection(self):
        inboard_1 = Node(position=[1, 0, 1])
        outboard_1 = Node(position=[2, 0, 2])
        test_link_1 = Link(inboard=inboard_1, outboard=outboard_1)

        inboard_2 = Node(position=[0, 0, 2])
        outboard_2 = Node(position=[-1, 0, 3])
        test_link_2 = Link(inboard=inboard_2, outboard=outboard_2)

        intersection = test_link_1.xz_intersection(test_link_2)

        self.assertListEqual(list(intersection.position), [1, 0, 1])
    
    def test_link_xz_intersection_edge_1(self):
        inboard_1 = Node(position=[1, 0, 1])
        outboard_1 = Node(position=[2, 0, 1])
        test_link_1 = Link(inboard=inboard_1, outboard=outboard_1)

        inboard_2 = Node(position=[1, 0, 0])
        outboard_2 = Node(position=[2, 0, 0])
        test_link_2 = Link(inboard=inboard_2, outboard=outboard_2)

        intersection = test_link_1.xz_intersection(test_link_2)

        self.assertListEqual(list(intersection.position), [np.inf, 0, 0.5])
    
    def test_link_xz_intersection_edge_2(self):
        inboard_1 = Node(position=[1, 0, 1])
        outboard_1 = Node(position=[2, 0, 1])
        test_link_1 = Link(inboard=inboard_1, outboard=outboard_1)

        inboard_2 = Node(position=[1, 0, 1])
        outboard_2 = Node(position=[2, 0, 1])
        test_link_2 = Link(inboard=inboard_2, outboard=outboard_2)

        intersection = test_link_1.xz_intersection(test_link_2)
        
        self.assertListEqual(list(intersection.position), [np.inf, 0, 1])


    # def test_node_translate_type(self):
    #     test_node = Node(position=[1, 1, 1])
    #     test_node.translate(translation=[-1, -1, -1])

    #     self.assertIsInstance(test_node.position, np.ndarray)

    # def test_node_translate_initial_position(self):
    #     test_node = Node(position=[1, 1, 1])
    #     test_node.translate(translation=[-1, -1, -1])
        
    #     self.assertListEqual(list(test_node.initial_position), [1, 1, 1])

    # def test_node_rotate_base(self):
    #     origin_node = Node(position=[0, 0, 0])
    #     update_node = Node(position=[1, 1, 0])

    #     update_node.rotate(origin=origin_node, angle_z=np.pi)
    #     self.assertListEqual(list([round(float(x)) for x in update_node.position]), [-1, -1, 0])

    # def test_node_rotate_edge(self):
    #     origin_node = Node(position=[0, 0, 0])
    #     update_node = Node(position=[0, 0, 0])

    #     update_node.rotate(origin=origin_node, angle_z=np.pi)
    #     self.assertListEqual(list([round(float(x)) for x in update_node.position]), [0, 0, 0])
    
    # def test_node_rotate_ND(self):
    #     origin_node = Node(position=[1, 1, 1])
    #     update_node = Node(position=[0, 0, 0])

    #     update_node.rotate(origin=origin_node, angle_x=np.pi, angle_y=np.pi/2)
    #     self.assertListEqual(list([round(float(x)) for x in update_node.position]), [2, 2, 2])
    
    # def test_node_reset_position(self):
    #     origin_node = Node(position=[1, 1, 1])
    #     update_node = Node(position=[0, 0, 0])

    #     update_node.translate(translation=[-1, -1, -1])
    #     update_node.rotate(origin=origin_node, angle_x=np.pi, angle_y=np.pi/2)
    #     update_node.reset()

    #     self.assertListEqual(list(update_node.position), [0, 0, 0])
    
    # def test_node_reset_type(self):
    #     origin_node = Node(position=[1, 1, 1])
    #     update_node = Node(position=[0, 0, 0])

    #     update_node.translate(translation=[-1, -1, -1])
    #     update_node.rotate(origin=origin_node, angle_x=np.pi, angle_y=np.pi/2)
    #     update_node.reset()

    #     self.assertIsInstance(update_node.position, np.ndarray)
    
    # def test_plotter_base(self):
    #     test_plotter = Plotter()

    #     test_node_1 = Node(position=[1, 1, 1])
    #     test_node_2 = Node(position=[1, -1, 1])
    #     test_node_3 = Node(position=[-1, 1, 1])
    #     test_node_4 = Node(position=[-1, -1, 1])
    #     test_node_5 = Node(position=[1, 1, -1])
    #     test_node_6 = Node(position=[1, -1, -1])
    #     test_node_7 = Node(position=[-1, 1, -1])
    #     test_node_8 = Node(position=[-1, -1, -1])

    #     nodes = [test_node_1, test_node_2, test_node_3, test_node_4, test_node_5, test_node_6, test_node_7, test_node_8]

    #     for node in nodes:
    #         node.plot(plotter=test_plotter, radius=0.25)
    #     # test_plotter.show()
    
    # def test_plotter_transform(self):
    #     test_plotter = Plotter()

    #     test_node_1 = Node(position=[1, 1, 1])
    #     test_node_2 = Node(position=[1, -1, 1])
    #     test_node_3 = Node(position=[-1, 1, 1])
    #     test_node_4 = Node(position=[-1, -1, 1])
    #     test_node_5 = Node(position=[1, 1, -1])
    #     test_node_6 = Node(position=[1, -1, -1])
    #     test_node_7 = Node(position=[-1, 1, -1])
    #     test_node_8 = Node(position=[-1, -1, -1])

    #     nodes = [test_node_1, test_node_2, test_node_3, test_node_4, test_node_5, test_node_6, test_node_7, test_node_8]

    #     for node in nodes:
    #         node.plot(plotter=test_plotter, radius=0.25)

    #     for node in nodes:
    #         node.rotate(origin=Node(position=[0, 0, 0]), angle_x=np.pi/4, angle_y=np.pi/4)

    #     for node in nodes:
    #         node.plot(plotter=test_plotter, radius=0.25)
    #     # test_plotter.show()