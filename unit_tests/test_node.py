from vehicle_model.new_suspension_model.suspension_elements.primary_elements.node import Node
from vehicle_model.new_suspension_model.assets.plotter import Plotter
import numpy as np

from unittest import main, TestCase


class NodeTest(TestCase):
    def test_node_init_position(self):
        test_node = Node(position=[1, 1, 1])

        self.assertListEqual(list(test_node.position), [1, 1, 1])
        self.assertIsInstance(test_node.position, np.ndarray)
    
    def test_node_init_type(self):
        test_node = Node(position=[1, 1, 1])

        self.assertIsInstance(test_node.position, np.ndarray)

    def test_node_translate_position(self):
        test_node = Node(position=[1, 1, 1])
        test_node.translate(translation=[-1, -1, -1])
        
        self.assertListEqual(list(test_node.position), [0, 0, 0])

    def test_node_translate_type(self):
        test_node = Node(position=[1, 1, 1])
        test_node.translate(translation=[-1, -1, -1])

        self.assertIsInstance(test_node.position, np.ndarray)

    def test_node_translate_initial_position(self):
        test_node = Node(position=[1, 1, 1])
        test_node.translate(translation=[-1, -1, -1])
        
        self.assertListEqual(list(test_node.initial_position), [1, 1, 1])

    def test_node_rotate_base(self):
        origin_node = Node(position=[0, 0, 0])
        update_node = Node(position=[1, 1, 0])

        update_node.rotate(origin=origin_node, angle_z=np.pi)
        self.assertListEqual(list([round(float(x)) for x in update_node.position]), [-1, -1, 0])

    def test_node_rotate_edge(self):
        origin_node = Node(position=[0, 0, 0])
        update_node = Node(position=[0, 0, 0])

        update_node.rotate(origin=origin_node, angle_z=np.pi)
        self.assertListEqual(list([round(float(x)) for x in update_node.position]), [0, 0, 0])
    
    def test_node_rotate_ND(self):
        origin_node = Node(position=[1, 1, 1])
        update_node = Node(position=[0, 0, 0])

        update_node.rotate(origin=origin_node, angle_x=np.pi, angle_y=np.pi/2)
        self.assertListEqual(list([round(float(x)) for x in update_node.position]), [2, 2, 2])
    
    def test_node_reset_position(self):
        origin_node = Node(position=[1, 1, 1])
        update_node = Node(position=[0, 0, 0])

        update_node.translate(translation=[-1, -1, -1])
        update_node.rotate(origin=origin_node, angle_x=np.pi, angle_y=np.pi/2)
        update_node.reset()

        self.assertListEqual(list(update_node.position), [0, 0, 0])
    
    def test_node_reset_type(self):
        origin_node = Node(position=[1, 1, 1])
        update_node = Node(position=[0, 0, 0])

        update_node.translate(translation=[-1, -1, -1])
        update_node.rotate(origin=origin_node, angle_x=np.pi, angle_y=np.pi/2)
        update_node.reset()

        self.assertIsInstance(update_node.position, np.ndarray)
    
    def test_plotter_base(self):
        test_plotter = Plotter()

        test_node_1 = Node(position=[1, 1, 1])
        test_node_2 = Node(position=[1, -1, 1])
        test_node_3 = Node(position=[-1, 1, 1])
        test_node_4 = Node(position=[-1, -1, 1])
        test_node_5 = Node(position=[1, 1, -1])
        test_node_6 = Node(position=[1, -1, -1])
        test_node_7 = Node(position=[-1, 1, -1])
        test_node_8 = Node(position=[-1, -1, -1])

        nodes = [test_node_1, test_node_2, test_node_3, test_node_4, test_node_5, test_node_6, test_node_7, test_node_8]

        for node in nodes:
            node.plot(plotter=test_plotter, radius=0.25)
        # test_plotter.show()
    
    def test_plotter_transform(self):
        test_plotter = Plotter()

        test_node_1 = Node(position=[1, 1, 1])
        test_node_2 = Node(position=[1, -1, 1])
        test_node_3 = Node(position=[-1, 1, 1])
        test_node_4 = Node(position=[-1, -1, 1])
        test_node_5 = Node(position=[1, 1, -1])
        test_node_6 = Node(position=[1, -1, -1])
        test_node_7 = Node(position=[-1, 1, -1])
        test_node_8 = Node(position=[-1, -1, -1])

        nodes = [test_node_1, test_node_2, test_node_3, test_node_4, test_node_5, test_node_6, test_node_7, test_node_8]

        for node in nodes:
            node.plot(plotter=test_plotter, radius=0.25)

        for node in nodes:
            node.rotate(origin=Node(position=[0, 0, 0]), angle_x=np.pi/4, angle_y=np.pi/4)

        for node in nodes:
            node.plot(plotter=test_plotter, radius=0.25)
        # test_plotter.show()