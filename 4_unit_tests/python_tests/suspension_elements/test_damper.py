from vehicle_model.suspension_model.suspension_elements.secondary_elements.damper import Damper
from vehicle_model.suspension_model.suspension_elements.primary_elements.node import Node

from unittest import main, TestCase
import numpy as np


class TestDamper(TestCase):
    def test_damper_init(self):
        inboard = Node(position=[0, 0, 0])
        outboard = Node(position=[0, 0.5, 0])
        damping_curve = [
            (0, 0),
            (1, 1),
            (2, np.sqrt(2)),
            (3, np.sqrt(3)),
            (4, 2)
        ]

        damper = Damper(inboard_node=inboard, outboard_node=outboard, damping_curve=damping_curve)
        
        self.assertListEqual(damper.damping_curve, [(0, 1, 2, 3, 4), (0, 1, np.sqrt(2), np.sqrt(3), 2)])
    
    def test_velocity_reference(self):
        inboard = Node(position=[0, 0, 0])
        outboard = Node(position=[0, 0.5, 0])
        damping_curve = [
            (0, 0),
            (1, 1),
            (2, np.sqrt(2)),
            (3, np.sqrt(3)),
            (4, 2)
        ]

        damper = Damper(inboard_node=inboard, outboard_node=outboard, damping_curve=damping_curve)
        
        self.assertTupleEqual(damper.velocity_reference, (0, 1, 2, 3, 4))
    
    def test_force_reference(self):
        inboard = Node(position=[0, 0, 0])
        outboard = Node(position=[0, 0.5, 0])
        damping_curve = [
            (0, 0),
            (1, 1),
            (2, np.sqrt(2)),
            (3, np.sqrt(3)),
            (4, 2)
        ]

        damper = Damper(inboard_node=inboard, outboard_node=outboard, damping_curve=damping_curve)
        
        self.assertTupleEqual(damper.force_reference, (0, 1, np.sqrt(2), np.sqrt(3), 2))

    def test_force_one(self):
        inboard = Node(position=[0, 0, 0])
        outboard = Node(position=[0, 0.5, 0])
        damping_curve = [
            (0, 0),
            (1, 1),
            (2, np.sqrt(2)),
            (3, np.sqrt(3)),
            (4, 2)
        ]

        damper = Damper(inboard_node=inboard, outboard_node=outboard, damping_curve=damping_curve)

        self.assertEqual(damper.force, 0)

    def test_force_two(self):
        inboard = Node(position=[0, 0, 0])
        outboard = Node(position=[0, 0.5, 0])
        damping_curve = [
            (0, 0),
            (1, 1),
            (2, np.sqrt(2)),
            (3, np.sqrt(3)),
            (4, 2)
        ]

        damper = Damper(inboard_node=inboard, outboard_node=outboard, damping_curve=damping_curve)
        damper.velocity = 1

        self.assertEqual(damper.force, 1)
    
    def test_force_three(self):
        inboard = Node(position=[0, 0, 0])
        outboard = Node(position=[0, 0.5, 0])
        damping_curve = [
            (0, 0),
            (1, 1),
            (2, np.sqrt(2)),
            (3, np.sqrt(3)),
            (4, 2)
        ]

        damper = Damper(inboard_node=inboard, outboard_node=outboard, damping_curve=damping_curve)
        damper.velocity = 2.5

        self.assertEqual(damper.force, (np.sqrt(3) + np.sqrt(2)) / 2)