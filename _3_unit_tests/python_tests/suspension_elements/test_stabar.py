from vehicle_model.suspension_model.suspension_elements._2_elements.stabar import Stabar
from vehicle_model.suspension_model.suspension_elements._1_elements.node import Node

from unittest import TestCase
import numpy as np


class TestStabar(TestCase):
    def test_stabar_init(self):
        left_arm_end = Node(position=[1, 2, 0])
        right_arm_end = Node(position=[1, -2, 0])
        left_droplink_end = Node(position=[1, 2, -0.5])
        right_droplink_end = Node(position=[1, -2, -0.5])
        bar_left_end = Node(position=[0, 2, 0])
        bar_right_end = Node(position=[0, -2, 0])
        torsional_stiffness = 1 # Nm/rad

        stabar = Stabar(left_arm_end=left_arm_end, 
                        right_arm_end=right_arm_end, 
                        left_droplink_end=left_droplink_end, 
                        right_droplink_end=right_droplink_end, 
                        bar_left_end=bar_left_end, 
                        bar_right_end=bar_right_end, 
                        torsional_stiffness=torsional_stiffness)
        
        left_droplink_end.add_listener(stabar)
        right_droplink_end.add_listener(stabar)
        
        left_droplink_end.translate(translation=[-1 * (1 - 1/2 * np.sqrt(3)), 0, 0.5])

        self.assertEqual(left_droplink_end.position[0], 1/2 * np.sqrt(3))
    
    def test_stabar_residual_eqn(self):
        left_arm_end = Node(position=[1, 2, 0])
        right_arm_end = Node(position=[1, -2, 0])
        left_droplink_end = Node(position=[1, 2, -0.5])
        right_droplink_end = Node(position=[1, -2, -0.5])
        bar_left_end = Node(position=[0, 2, 0])
        bar_right_end = Node(position=[0, -2, 0])
        torsional_stiffness = 1 # Nm/rad

        stabar = Stabar(left_arm_end=left_arm_end, 
                        right_arm_end=right_arm_end, 
                        left_droplink_end=left_droplink_end, 
                        right_droplink_end=right_droplink_end, 
                        bar_left_end=bar_left_end, 
                        bar_right_end=bar_right_end, 
                        torsional_stiffness=torsional_stiffness)
        
        left_droplink_end.add_listener(stabar)
        right_droplink_end.add_listener(stabar)
        
        stabar._droplink_eqn(x=np.pi/6, args=[stabar.left_droplink])
        
        self.assertEqual([round(x, 7) for x in stabar.left_droplink.outboard_node.position], [round(1/2 * np.sqrt(3), 7), 2, 0.5])
    
    def test_stabar_residual_eqn_length(self):
        left_arm_end = Node(position=[1, 2, 0])
        right_arm_end = Node(position=[1, -2, 0])
        left_droplink_end = Node(position=[1, 2, -0.5])
        right_droplink_end = Node(position=[1, -2, -0.5])
        bar_left_end = Node(position=[0, 2, 0])
        bar_right_end = Node(position=[0, -2, 0])
        torsional_stiffness = 1 # Nm/rad

        stabar = Stabar(left_arm_end=left_arm_end, 
                        right_arm_end=right_arm_end, 
                        left_droplink_end=left_droplink_end, 
                        right_droplink_end=right_droplink_end, 
                        bar_left_end=bar_left_end, 
                        bar_right_end=bar_right_end, 
                        torsional_stiffness=torsional_stiffness)
        
        left_droplink_end.add_listener(stabar)
        right_droplink_end.add_listener(stabar)

        stabar._droplink_eqn(x=np.pi/6, args=[stabar.left_droplink])
        left_droplink_end.position[0] += -1 * (1 - 1/2 * np.sqrt(3))
        left_droplink_end.position[1] += 0
        left_droplink_end.position[2] += 0.5

        self.assertEqual(round(stabar.left_droplink.length, 7), stabar.left_droplink.initial_length)

    def test_stabar_residual_eqn_repeated(self):
        left_arm_end = Node(position=[1, 2, 0])
        right_arm_end = Node(position=[1, -2, 0])
        left_droplink_end = Node(position=[1, 2, -0.5])
        right_droplink_end = Node(position=[1, -2, -0.5])
        bar_left_end = Node(position=[0, 2, 0])
        bar_right_end = Node(position=[0, -2, 0])
        torsional_stiffness = 1 # Nm/rad

        stabar = Stabar(left_arm_end=left_arm_end, 
                        right_arm_end=right_arm_end, 
                        left_droplink_end=left_droplink_end, 
                        right_droplink_end=right_droplink_end, 
                        bar_left_end=bar_left_end, 
                        bar_right_end=bar_right_end, 
                        torsional_stiffness=torsional_stiffness)
        
        left_droplink_end.add_listener(stabar)
        right_droplink_end.add_listener(stabar)
        
        stabar._droplink_eqn(x=np.pi/6, args=[stabar.left_droplink])
        stabar._droplink_eqn(x=np.pi/6, args=[stabar.left_droplink])
        
        self.assertEqual([round(x, 7) for x in stabar.left_droplink.outboard_node.position], [round(1/2 * np.sqrt(3), 7), 2, 0.5])

    def test_stabar_update_one(self):
        left_arm_end = Node(position=[1, 2, 0])
        right_arm_end = Node(position=[1, -2, 0])
        left_droplink_end = Node(position=[1, 2, -0.5])
        right_droplink_end = Node(position=[1, -2, -0.5])
        bar_left_end = Node(position=[0, 2, 0])
        bar_right_end = Node(position=[0, -2, 0])
        torsional_stiffness = 1 # Nm/rad

        stabar = Stabar(left_arm_end=left_arm_end, 
                        right_arm_end=right_arm_end, 
                        left_droplink_end=left_droplink_end, 
                        right_droplink_end=right_droplink_end, 
                        bar_left_end=bar_left_end, 
                        bar_right_end=bar_right_end, 
                        torsional_stiffness=torsional_stiffness)
        
        left_droplink_end.add_listener(stabar)
        right_droplink_end.add_listener(stabar)
        
        left_droplink_end.translate(translation=[-1 * (1 - 1/2 * np.sqrt(3)), 0, 0.5])
        
        self.assertEqual(round(stabar.left_rotation, 7), round(np.pi/6, 7))
    
    def test_stabar_update_two(self):
        left_arm_end = Node(position=[1, 2, 0])
        right_arm_end = Node(position=[1, -2, 0])
        left_droplink_end = Node(position=[1, 2, -0.5])
        right_droplink_end = Node(position=[1, -2, -0.5])
        bar_left_end = Node(position=[0, 2, 0])
        bar_right_end = Node(position=[0, -2, 0])
        torsional_stiffness = 1 # Nm/rad

        stabar = Stabar(left_arm_end=left_arm_end, 
                        right_arm_end=right_arm_end, 
                        left_droplink_end=left_droplink_end, 
                        right_droplink_end=right_droplink_end, 
                        bar_left_end=bar_left_end, 
                        bar_right_end=bar_right_end, 
                        torsional_stiffness=torsional_stiffness)
        
        left_droplink_end.add_listener(stabar)
        right_droplink_end.add_listener(stabar)
        
        left_droplink_end.translate(translation=[-1 * (1 - 1/2 * np.sqrt(3)), 0, 0.5])
        
        self.assertEqual(round(stabar.right_rotation, 7), 0)
    
    def test_stabar_update_three(self):
        left_arm_end = Node(position=[1, 2, 0])
        right_arm_end = Node(position=[1, -2, 0])
        left_droplink_end = Node(position=[1, 2, -0.5])
        right_droplink_end = Node(position=[1, -2, -0.5])
        bar_left_end = Node(position=[0, 2, 0])
        bar_right_end = Node(position=[0, -2, 0])
        torsional_stiffness = 1 # Nm/rad

        stabar = Stabar(left_arm_end=left_arm_end, 
                        right_arm_end=right_arm_end, 
                        left_droplink_end=left_droplink_end, 
                        right_droplink_end=right_droplink_end, 
                        bar_left_end=bar_left_end, 
                        bar_right_end=bar_right_end, 
                        torsional_stiffness=torsional_stiffness)
        
        left_droplink_end.add_listener(stabar)
        right_droplink_end.add_listener(stabar)
        
        right_droplink_end.translate(translation=[-1 * (1 - 1/2 * np.sqrt(3)), 0, 0.5])
        
        self.assertEqual(round(stabar.right_rotation, 7), round(np.pi/6, 7))
    
    def test_stabar_update_four(self):
        left_arm_end = Node(position=[1, 2, 0])
        right_arm_end = Node(position=[1, -2, 0])
        left_droplink_end = Node(position=[1, 2, -0.5])
        right_droplink_end = Node(position=[1, -2, -0.5])
        bar_left_end = Node(position=[0, 2, 0])
        bar_right_end = Node(position=[0, -2, 0])
        torsional_stiffness = 1 # Nm/rad

        stabar = Stabar(left_arm_end=left_arm_end, 
                        right_arm_end=right_arm_end, 
                        left_droplink_end=left_droplink_end, 
                        right_droplink_end=right_droplink_end, 
                        bar_left_end=bar_left_end, 
                        bar_right_end=bar_right_end, 
                        torsional_stiffness=torsional_stiffness)
        
        left_droplink_end.add_listener(stabar)
        right_droplink_end.add_listener(stabar)
        
        left_droplink_end.translate(translation=[-1 * (1 - 1/2 * np.sqrt(3)), 0, -0.5])
        
        self.assertEqual(round(stabar.left_rotation, 7), round(-np.pi/6, 7))
    
    def test_stabar_update_five(self):
        left_arm_end = Node(position=[1, 2, 0])
        right_arm_end = Node(position=[1, -2, 0])
        left_droplink_end = Node(position=[1, 2, -0.5])
        right_droplink_end = Node(position=[1, -2, -0.5])
        bar_left_end = Node(position=[0, 2, 0])
        bar_right_end = Node(position=[0, -2, 0])
        torsional_stiffness = 1 # Nm/rad

        stabar = Stabar(left_arm_end=left_arm_end, 
                        right_arm_end=right_arm_end, 
                        left_droplink_end=left_droplink_end, 
                        right_droplink_end=right_droplink_end, 
                        bar_left_end=bar_left_end, 
                        bar_right_end=bar_right_end, 
                        torsional_stiffness=torsional_stiffness)
        
        left_droplink_end.add_listener(stabar)
        right_droplink_end.add_listener(stabar)
        
        left_droplink_end.translate(translation=[-1 * (1 - 1/2 * np.sqrt(3)), 0, -0.5])
        
        self.assertEqual(round(stabar.right_rotation, 7), 0)
    
    def test_stabar_update_six(self):
        left_arm_end = Node(position=[1, 2, 0])
        right_arm_end = Node(position=[1, -2, 0])
        left_droplink_end = Node(position=[1, 2, -0.5])
        right_droplink_end = Node(position=[1, -2, -0.5])
        bar_left_end = Node(position=[0, 2, 0])
        bar_right_end = Node(position=[0, -2, 0])
        torsional_stiffness = 1 # Nm/rad

        stabar = Stabar(left_arm_end=left_arm_end, 
                        right_arm_end=right_arm_end, 
                        left_droplink_end=left_droplink_end, 
                        right_droplink_end=right_droplink_end, 
                        bar_left_end=bar_left_end, 
                        bar_right_end=bar_right_end, 
                        torsional_stiffness=torsional_stiffness)
        
        left_droplink_end.add_listener(stabar)
        right_droplink_end.add_listener(stabar)
        
        right_droplink_end.translate(translation=[-1 * (1 - 1/2 * np.sqrt(3)), 0, -0.5])
        
        self.assertEqual(round(stabar.right_rotation, 7), round(-np.pi/6, 7))
    
    def test_stabar_update_rotation_one(self):
        left_arm_end = Node(position=[1, 2, 0])
        right_arm_end = Node(position=[1, -2, 0])
        left_droplink_end = Node(position=[1, 2, -0.5])
        right_droplink_end = Node(position=[1, -2, -0.5])
        bar_left_end = Node(position=[0, 2, 0])
        bar_right_end = Node(position=[0, -2, 0])
        torsional_stiffness = 1 # Nm/rad

        stabar = Stabar(left_arm_end=left_arm_end, 
                        right_arm_end=right_arm_end, 
                        left_droplink_end=left_droplink_end, 
                        right_droplink_end=right_droplink_end, 
                        bar_left_end=bar_left_end, 
                        bar_right_end=bar_right_end, 
                        torsional_stiffness=torsional_stiffness)
        
        left_droplink_end.add_listener(stabar)
        right_droplink_end.add_listener(stabar)
        
        left_droplink_end.translate(translation=[-1 * (1 - 1/2 * np.sqrt(3)), 0, 0.5])
        right_droplink_end.translate(translation=[-1 * (1 - 1/2 * np.sqrt(3)), 0, -0.5])
        
        self.assertEqual(round(stabar.rotation, 7), round(np.pi/3, 7))
    
    def test_stabar_update_rotation_two(self):
        left_arm_end = Node(position=[1, 2, 0])
        right_arm_end = Node(position=[1, -2, 0])
        left_droplink_end = Node(position=[1, 2, -0.5])
        right_droplink_end = Node(position=[1, -2, -0.5])
        bar_left_end = Node(position=[0, 2, 0])
        bar_right_end = Node(position=[0, -2, 0])
        torsional_stiffness = 1 # Nm/rad

        stabar = Stabar(left_arm_end=left_arm_end, 
                        right_arm_end=right_arm_end, 
                        left_droplink_end=left_droplink_end, 
                        right_droplink_end=right_droplink_end, 
                        bar_left_end=bar_left_end, 
                        bar_right_end=bar_right_end, 
                        torsional_stiffness=torsional_stiffness)
        
        left_droplink_end.add_listener(stabar)
        right_droplink_end.add_listener(stabar)
        
        left_droplink_end.translate(translation=[-1 * (1 - 1/2 * np.sqrt(3)), 0, -0.5])
        right_droplink_end.translate(translation=[-1 * (1 - 1/2 * np.sqrt(3)), 0, 0.5])
        
        self.assertEqual(round(stabar.rotation, 7), round(-np.pi/3, 7))
    
    def test_stabar_update_rotation_three(self):
        left_arm_end = Node(position=[1, 2, 0])
        right_arm_end = Node(position=[1, -2, 0])
        left_droplink_end = Node(position=[1, 2, -0.5])
        right_droplink_end = Node(position=[1, -2, -0.5])
        bar_left_end = Node(position=[0, 2, 0])
        bar_right_end = Node(position=[0, -2, 0])
        torsional_stiffness = 1 # Nm/rad

        stabar = Stabar(left_arm_end=left_arm_end, 
                        right_arm_end=right_arm_end, 
                        left_droplink_end=left_droplink_end, 
                        right_droplink_end=right_droplink_end, 
                        bar_left_end=bar_left_end, 
                        bar_right_end=bar_right_end, 
                        torsional_stiffness=torsional_stiffness)
        
        left_droplink_end.add_listener(stabar)
        right_droplink_end.add_listener(stabar)
        
        left_droplink_end.translate(translation=[-1 * (1 - 1/2 * np.sqrt(3)), 0, 0.5])
        right_droplink_end.translate(translation=[-1 * (1 - 1/2 * np.sqrt(3)), 0, 0.5])
        
        self.assertEqual(round(stabar.rotation, 7), 0)
    
    def test_stabar_torque_one(self):
        left_arm_end = Node(position=[1, 2, 0])
        right_arm_end = Node(position=[1, -2, 0])
        left_droplink_end = Node(position=[1, 2, -0.5])
        right_droplink_end = Node(position=[1, -2, -0.5])
        bar_left_end = Node(position=[0, 2, 0])
        bar_right_end = Node(position=[0, -2, 0])
        torsional_stiffness = 1 # Nm/rad

        stabar = Stabar(left_arm_end=left_arm_end, 
                        right_arm_end=right_arm_end, 
                        left_droplink_end=left_droplink_end, 
                        right_droplink_end=right_droplink_end, 
                        bar_left_end=bar_left_end, 
                        bar_right_end=bar_right_end, 
                        torsional_stiffness=torsional_stiffness)
        
        left_droplink_end.add_listener(stabar)
        right_droplink_end.add_listener(stabar)
        
        left_droplink_end.translate(translation=[-1 * (1 - 1/2 * np.sqrt(3)), 0, 0.5])
        right_droplink_end.translate(translation=[-1 * (1 - 1/2 * np.sqrt(3)), 0, 0.5])
        
        self.assertEqual(round(stabar.torque, 7), 0)

    def test_stabar_torque_two(self):
        left_arm_end = Node(position=[1, 2, 0])
        right_arm_end = Node(position=[1, -2, 0])
        left_droplink_end = Node(position=[1, 2, -0.5])
        right_droplink_end = Node(position=[1, -2, -0.5])
        bar_left_end = Node(position=[0, 2, 0])
        bar_right_end = Node(position=[0, -2, 0])
        torsional_stiffness = 1 # Nm/rad

        stabar = Stabar(left_arm_end=left_arm_end, 
                        right_arm_end=right_arm_end, 
                        left_droplink_end=left_droplink_end, 
                        right_droplink_end=right_droplink_end, 
                        bar_left_end=bar_left_end, 
                        bar_right_end=bar_right_end, 
                        torsional_stiffness=torsional_stiffness)
        
        left_droplink_end.add_listener(stabar)
        right_droplink_end.add_listener(stabar)
        
        left_droplink_end.translate(translation=[-1 * (1 - 1/2 * np.sqrt(3)), 0, 0.5])
        right_droplink_end.translate(translation=[-1 * (1 - 1/2 * np.sqrt(3)), 0, -0.5])
        
        self.assertEqual(round(stabar.torque, 7), round(1 * np.pi/3, 7))
    
    def test_stabar_torque_three(self):
        left_arm_end = Node(position=[1, 2, 0])
        right_arm_end = Node(position=[1, -2, 0])
        left_droplink_end = Node(position=[1, 2, -0.5])
        right_droplink_end = Node(position=[1, -2, -0.5])
        bar_left_end = Node(position=[0, 2, 0])
        bar_right_end = Node(position=[0, -2, 0])
        torsional_stiffness = 1 # Nm/rad

        stabar = Stabar(left_arm_end=left_arm_end, 
                        right_arm_end=right_arm_end, 
                        left_droplink_end=left_droplink_end, 
                        right_droplink_end=right_droplink_end, 
                        bar_left_end=bar_left_end, 
                        bar_right_end=bar_right_end, 
                        torsional_stiffness=torsional_stiffness)
        
        left_droplink_end.add_listener(stabar)
        right_droplink_end.add_listener(stabar)
        
        left_droplink_end.translate(translation=[-1 * (1 - 1/2 * np.sqrt(3)), 0, -0.5])
        right_droplink_end.translate(translation=[-1 * (1 - 1/2 * np.sqrt(3)), 0, 0.5])
        
        self.assertEqual(round(stabar.torque, 7), round(1 * np.pi/3, 7))