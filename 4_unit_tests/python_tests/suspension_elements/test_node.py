from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node
import numpy as np

from unittest import main, TestCase


class TestNode(TestCase):
    def test_node_init_position(self):
        test_node = Node(position=[1, 1, 1])

        self.assertListEqual(test_node.position, [1, 1, 1])
    
    def test_node_translate_position(self):
        test_node = Node(position=[1, 1, 1])
        test_node.translate(translation=[-1, -1, -1])
        
        self.assertListEqual(test_node.position, [0, 0, 0])

    def test_node_translate_initial_position(self):
        test_node = Node(position=[1, 1, 1])
        test_node.translate(translation=[-1, -1, -1])
        
        self.assertListEqual(test_node.initial_position, [1, 1, 1])

    def test_node_rotate_base_angles(self):
        origin_node = Node(position=[0, 0, 0])
        update_node = Node(position=[1, 1, 0])

        update_node.rotate(origin=origin_node, ang_x=0, ang_y=0, ang_z=np.pi)
        self.assertListEqual([round(float(x)) for x in update_node.position], [-1, -1, 0])

    def test_node_rotate_edge_angles(self):
        origin_node = Node(position=[0, 0, 0])
        update_node = Node(position=[0, 0, 0])

        update_node.rotate(origin=origin_node, ang_x=0, ang_y=0, ang_z=np.pi)
        self.assertListEqual([round(float(x)) for x in update_node.position], [0, 0, 0])
    
    def test_node_rotate_ND_angles(self):
        origin_node = Node(position=[1, 1, 1])
        update_node = Node(position=[0, 0, 0])

        update_node.rotate(origin=origin_node, ang_x=np.pi, ang_y=np.pi/2, ang_z=0)
        self.assertListEqual([round(float(x)) for x in update_node.position], [2, 2, 2])
    
    def test_node_rotate_base_vector(self):
        origin_node = Node(position=[0, 0, 0])
        update_node = Node(position=[1, 1, 0])

        update_node.rotate(origin=origin_node, direction=[0, 0, 1], angle=np.pi)
        self.assertListEqual([round(float(x)) for x in update_node.position], [-1, -1, 0])

    def test_node_rotate_edge_vector(self):
        origin_node = Node(position=[0, 0, 0])
        update_node = Node(position=[0, 0, 0])

        update_node.rotate(origin=origin_node, direction=[0, 0, 1], angle=0)
        self.assertListEqual([round(float(x)) for x in update_node.position], [0, 0, 0])
    
    def test_node_rotate_repeated_one(self):
        origin_node = Node(position=[0, 0, 0])
        update_node = Node(position=[0, 1, 0])

        update_node.rotate(origin=origin_node, direction=[0, 0, 1], angle=np.pi/2)
        update_node.rotate(origin=origin_node, direction=[0, 0, 1], angle=np.pi/2)
        self.assertListEqual([round(float(x)) for x in update_node.position], [-1, 0, 0])
    
    def test_node_rotate_repeated_two(self):
        origin_node = Node(position=[0, 0, 0])
        update_node = Node(position=[0, 1, 0])

        update_node.rotate(origin=origin_node, direction=[0, 0, 1], angle=np.pi/2)
        update_node.rotate(origin=origin_node, direction=[0, 0, 1], angle=-np.pi/2)
        self.assertListEqual([round(float(x)) for x in update_node.position], [1, 0, 0])

    def test_node_rotate_repeated_three(self):
        origin_node = Node(position=[0, 0, 0])
        update_node = Node(position=[0, 1, 0])

        update_node.rotate(origin=origin_node, direction=[0, 0, 1], angle=np.pi/2)
        update_node.rotate(origin=origin_node, direction=[0, 0, 1], angle=0)
        self.assertListEqual([round(float(x)) for x in update_node.position], [0, 1, 0])

    def test_node_reset_position(self):
        origin_node = Node(position=[1, 1, 1])
        update_node = Node(position=[0, 0, 0])

        update_node.translate(translation=[-1, -1, -1])
        update_node.rotate(origin=origin_node, ang_x=np.pi, ang_y=np.pi/2, ang_z=0)
        update_node.reset()

        self.assertListEqual(update_node.position, [0, 0, 0])
    
    def test_node_addition(self):
        first_node = Node(position=[1, 1, 1])
        second_node = Node(position=[1, 1, 1])

        node_sum = first_node + second_node

        self.assertListEqual(node_sum.position, [2, 2, 2])

    def test_node_subtraction(self):
        first_node = Node(position=[3, 3, 3])
        second_node = Node(position=[2, 2, 2])

        node_diff = first_node - second_node

        self.assertListEqual(node_diff.position, [1, 1, 1])
    
    def test_node_multiplication(self):
        node = Node(position=[1, 1, 1])

        node_product = node * 2

        self.assertListEqual(node_product.position, [2, 2, 2])
    
    def test_node_division(self):
        node = Node(position=[2, 2, 2])

        node_quotient = node / 2

        self.assertListEqual(node_quotient.position, [1, 1, 1])
    
    def test_node_getitem(self):
        node = Node(position=[1, 2, 3])

        self.assertEqual(node[2], 3)
    
    def test_node_setitem(self):
        node = Node(position=[1, 2, 3])
        node[2] = 1

        self.assertEqual(node[2], 1)
    
    def test_node_child(self):
        node = Node(position=[0, 0, 0])
        node_child = Node(position=[0, 0, 1])

        node.add_child(node=node_child)

        self.assertIn(node_child, node.child_nodes)
        
    def test_node_child_translate(self):
        node = Node(position=[0, 0, 0])
        node_child = Node(position=[0, 0, 1])

        node.add_child(node=node_child)

        node.translate(translation=[0, 0, 1])

        self.assertEqual(node_child[2], 2)
    
    def test_node_child_reset(self):
        node = Node(position=[0, 0, 0])
        node_child = Node(position=[0, 0, 1])

        node.add_child(node=node_child)

        node.translate(translation=[0, 0, 1])
        node.reset()

        self.assertEqual(node_child[2], 1)

    def test_node_child_rotate_angles(self):
        origin_node = Node(position=[0, 0, 0])
        update_node = Node(position=[1, 1, 0])

        node_child = Node(position=[-1, -1, 0])

        update_node.add_child(node=node_child)

        update_node.rotate(origin=origin_node, direction=[0, 0, 1], angle=np.pi)
        self.assertListEqual([round(float(x)) for x in node_child.position], [1, 1, 0])

    def test_node_child_rotate_vector(self):
        origin_node = Node(position=[0, 0, 0])
        update_node = Node(position=[1, 1, 0])

        node_child = Node(position=[-1, -1, 0])

        update_node.add_child(node=node_child)

        update_node.rotate(origin=origin_node, ang_x=0, ang_y=0, ang_z=np.pi)
        self.assertListEqual([round(float(x)) for x in node_child.position], [1, 1, 0])
    
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
    #         test_plotter.add_node(node=node, radius=0.25)
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
    #         test_plotter.add_node(node=node, radius=0.25)

    #     for node in nodes:
    #         node.rotate(origin=Node(position=[0, 0, 0]), angle_x=np.pi/4, angle_y=np.pi/4)

    #     for node in nodes:
    #         test_plotter.add_node(node=node, radius=0.25, color="black")
    #     # test_plotter.show()