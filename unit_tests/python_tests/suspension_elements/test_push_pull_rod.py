from src.vehicle_model.suspension_model.suspension_elements._4_elements.push_pull_rod import PushPullRod
from src.vehicle_model.suspension_model.suspension_elements._2_elements.bellcrank import Bellcrank
from src.vehicle_model.suspension_model.suspension_elements._2_elements.spring import Spring
from src.vehicle_model.suspension_model.suspension_elements._1_elements.link import Link
from src.vehicle_model.suspension_model.suspension_elements._1_elements.node import Node

from unittest import TestCase
import numpy as np


class TestPushPullRod(TestCase):
    def test_push_pull_rod_partial_one(self):
        rigid_outboard = Node(position=[0, 2, 0])
        shared_node = Node(position=[0, 1, 0])
        compliant_inboard = Node(position=[0, 0, 0])

        outboard_rod = Link(inboard_node=shared_node, outboard_node=rigid_outboard)
        spring = Spring(inboard_node=compliant_inboard, outboard_node=shared_node, free_length=1, rate=1)

        push_pull_rod = PushPullRod(outboard_rod=outboard_rod, spring=spring, inboard_rod=None, bellcrank=None)

        rigid_outboard.translate(translation=[0, -0.5, 0])

        self.assertEqual(shared_node[1], 0.5)

    def test_push_pull_rod_partial_two(self):
        rigid_outboard = Node(position=[0, 2, 0])
        shared_node = Node(position=[0, 1, 0])
        compliant_inboard = Node(position=[0, 0, 0])

        outboard_rod = Link(inboard_node=shared_node, outboard_node=rigid_outboard)
        spring = Spring(inboard_node=compliant_inboard, outboard_node=shared_node, free_length=1, rate=1)

        push_pull_rod = PushPullRod(outboard_rod=outboard_rod, spring=spring, inboard_rod=None, bellcrank=None)

        rigid_outboard.translate(translation=[0, 0, 1])

        angle = np.atan(1/2)
        compliant_length = np.sqrt(5) - 1

        self.assertListEqual(shared_node.position, [0, compliant_length * np.cos(angle), compliant_length * np.sin(angle)])

    def test_push_pull_rod_partial_three(self):
        rigid_outboard = Node(position=[0, 2, 0])
        shared_node = Node(position=[0, 1, 0])
        compliant_inboard = Node(position=[0, 0, 0])

        outboard_rod = Link(inboard_node=shared_node, outboard_node=rigid_outboard)
        spring = Spring(inboard_node=compliant_inboard, outboard_node=shared_node, free_length=1, rate=1)

        push_pull_rod = PushPullRod(outboard_rod=outboard_rod, spring=spring, inboard_rod=None, bellcrank=None)

        rigid_outboard.translate(translation=[0, -0.5, 0])

        self.assertEqual(push_pull_rod.outboard_rod.length, push_pull_rod.outboard_rod.initial_length)
    
    def test_push_pull_rod_partial_four(self):
        rigid_outboard = Node(position=[0, 2, 0])
        shared_node = Node(position=[0, 1, 0])
        compliant_inboard = Node(position=[0, 0, 0])

        outboard_rod = Link(inboard_node=shared_node, outboard_node=rigid_outboard)
        spring = Spring(inboard_node=compliant_inboard, outboard_node=shared_node, free_length=1, rate=1)

        push_pull_rod = PushPullRod(outboard_rod=outboard_rod, spring=spring, inboard_rod=None, bellcrank=None)

        rigid_outboard.translate(translation=[0, -0.5, 0])

        self.assertNotEqual(push_pull_rod.spring.length, push_pull_rod.spring.initial_length)
    
    def test_push_pull_rod_partial_five(self):
        rigid_outboard = Node(position=[0, 2, 0])
        shared_node = Node(position=[0, 1, 0])
        compliant_inboard = Node(position=[0, 0, 0])

        outboard_rod = Link(inboard_node=shared_node, outboard_node=rigid_outboard)
        spring = Spring(inboard_node=compliant_inboard, outboard_node=shared_node, free_length=1, rate=1)

        push_pull_rod = PushPullRod(outboard_rod=outboard_rod, spring=spring, inboard_rod=None, bellcrank=None)

        rigid_outboard.translate(translation=[0, 0, 1])

        self.assertEqual(round(push_pull_rod.outboard_rod.length, 7), round(push_pull_rod.outboard_rod.initial_length, 7))
    
    def test_push_pull_rod_partial_six(self):
        rigid_outboard = Node(position=[0, 2, 0])
        shared_node = Node(position=[0, 1, 0])
        compliant_inboard = Node(position=[0, 0, 0])

        outboard_rod = Link(inboard_node=shared_node, outboard_node=rigid_outboard)
        spring = Spring(inboard_node=compliant_inboard, outboard_node=shared_node, free_length=1, rate=1)

        push_pull_rod = PushPullRod(outboard_rod=outboard_rod, spring=spring, inboard_rod=None, bellcrank=None)

        rigid_outboard.translate(translation=[0, 0, 1])

        self.assertEqual(round(push_pull_rod.spring.length, 7), round(np.sqrt(5) - 1, 7))
    
    def test_push_pull_rod_full_outboard_translation_positive(self):
        # Nodes
        outboard_rod_outboard = Node(position=[0, 0, 0])
        outboard_rod_inboard = Node(position=[0, 1, 1])
        inboard_rod_outboard = Node(position=[0, 2, 2])
        inboard_rod_inboard = Node(position=[0, 3, 2])
        
        spring_inboard = Node(position=[0, 4, 2])

        bellcrank_pivot = Node(position=[0, 2, 1])
        bellcrank_direction = [1, 0, 0]

        # Links
        bellcrank = Bellcrank(outboard_rod_inboard, inboard_rod_outboard, pivot=bellcrank_pivot, pivot_direction=bellcrank_direction)
        outboard_rod = Link(inboard_node=outboard_rod_inboard, outboard_node=outboard_rod_outboard)
        inboard_rod = Link(inboard_node=inboard_rod_inboard, outboard_node=inboard_rod_outboard)
        spring = Spring(inboard_node=spring_inboard, outboard_node=inboard_rod_inboard, free_length=1, rate=1)
        
        # Full PushPullRod
        push_pull_rod = PushPullRod(outboard_rod=outboard_rod, spring=spring, inboard_rod=inboard_rod, bellcrank=bellcrank)

        outboard_rod_outboard.translate(translation=[0, 0, 1])

        bellcrank_angle = -41.41 # from Solidworks
        self.assertEqual(round(push_pull_rod.bellcrank_angle * 180 / np.pi, 2), bellcrank_angle)
    
    def test_push_pull_rod_full_outboard_translation_negative(self):
        # Nodes
        outboard_rod_outboard = Node(position=[0, 0, 0])
        outboard_rod_inboard = Node(position=[0, 1, 1])
        inboard_rod_outboard = Node(position=[0, 2, 2])
        inboard_rod_inboard = Node(position=[0, 3, 2])
        
        spring_inboard = Node(position=[0, 4, 2])

        bellcrank_pivot = Node(position=[0, 2, 1])
        bellcrank_direction = [1, 0, 0]

        # Links
        bellcrank = Bellcrank(outboard_rod_inboard, inboard_rod_outboard, pivot=bellcrank_pivot, pivot_direction=bellcrank_direction)
        outboard_rod = Link(inboard_node=outboard_rod_inboard, outboard_node=outboard_rod_outboard)
        inboard_rod = Link(inboard_node=inboard_rod_inboard, outboard_node=inboard_rod_outboard)
        spring = Spring(inboard_node=spring_inboard, outboard_node=inboard_rod_inboard, free_length=1, rate=1)
        
        # Full PushPullRod
        push_pull_rod = PushPullRod(outboard_rod=outboard_rod, spring=spring, inboard_rod=inboard_rod, bellcrank=bellcrank)

        outboard_rod_outboard.translate(translation=[0, 0, -0.25])

        bellcrank_angle = 17.30 # from Solidworks
        self.assertEqual(round(push_pull_rod.bellcrank_angle * 180 / np.pi, 2), bellcrank_angle)
    
    def test_push_pull_rod_full_outboard_rod_positive(self):
        # Nodes
        outboard_rod_outboard = Node(position=[0, 0, 0])
        outboard_rod_inboard = Node(position=[0, 1, 1])
        inboard_rod_outboard = Node(position=[0, 2, 2])
        inboard_rod_inboard = Node(position=[0, 3, 2])
        
        spring_inboard = Node(position=[0, 4, 2])

        bellcrank_pivot = Node(position=[0, 2, 1])
        bellcrank_direction = [1, 0, 0]

        # Links
        bellcrank = Bellcrank(outboard_rod_inboard, inboard_rod_outboard, pivot=bellcrank_pivot, pivot_direction=bellcrank_direction)
        outboard_rod = Link(inboard_node=outboard_rod_inboard, outboard_node=outboard_rod_outboard)
        inboard_rod = Link(inboard_node=inboard_rod_inboard, outboard_node=inboard_rod_outboard)
        spring = Spring(inboard_node=spring_inboard, outboard_node=inboard_rod_inboard, free_length=1, rate=1)
        
        # Full PushPullRod
        push_pull_rod = PushPullRod(outboard_rod=outboard_rod, spring=spring, inboard_rod=inboard_rod, bellcrank=bellcrank)

        outboard_rod_outboard.translate(translation=[0, 0, 1])

        self.assertEqual(round(outboard_rod.length, 5), round(outboard_rod.initial_length, 5))
    
    def test_push_pull_rod_full_outboard_rod_negative(self):
        # Nodes
        outboard_rod_outboard = Node(position=[0, 0, 0])
        outboard_rod_inboard = Node(position=[0, 1, 1])
        inboard_rod_outboard = Node(position=[0, 2, 2])
        inboard_rod_inboard = Node(position=[0, 3, 2])
        
        spring_inboard = Node(position=[0, 4, 2])

        bellcrank_pivot = Node(position=[0, 2, 1])
        bellcrank_direction = [1, 0, 0]

        # Links
        bellcrank = Bellcrank(outboard_rod_inboard, inboard_rod_outboard, pivot=bellcrank_pivot, pivot_direction=bellcrank_direction)
        outboard_rod = Link(inboard_node=outboard_rod_inboard, outboard_node=outboard_rod_outboard)
        inboard_rod = Link(inboard_node=inboard_rod_inboard, outboard_node=inboard_rod_outboard)
        spring = Spring(inboard_node=spring_inboard, outboard_node=inboard_rod_inboard, free_length=1, rate=1)
        
        # Full PushPullRod
        push_pull_rod = PushPullRod(outboard_rod=outboard_rod, spring=spring, inboard_rod=inboard_rod, bellcrank=bellcrank)

        outboard_rod_outboard.translate(translation=[0, 0, -0.25])

        self.assertEqual(round(outboard_rod.length, 5), round(outboard_rod.initial_length, 5))
    
    def test_push_pull_rod_full_outboard_rod_position(self):
        # Nodes
        outboard_rod_outboard = Node(position=[0, 0, 0])
        outboard_rod_inboard = Node(position=[0, 1, 1])
        inboard_rod_outboard = Node(position=[0, 2, 2])
        inboard_rod_inboard = Node(position=[0, 3, 2])
        
        spring_inboard = Node(position=[0, 4, 2])

        bellcrank_pivot = Node(position=[0, 2, 1])
        bellcrank_direction = [1, 0, 0]

        # Links
        bellcrank = Bellcrank(outboard_rod_inboard, inboard_rod_outboard, pivot=bellcrank_pivot, pivot_direction=bellcrank_direction)
        outboard_rod = Link(inboard_node=outboard_rod_inboard, outboard_node=outboard_rod_outboard)
        inboard_rod = Link(inboard_node=inboard_rod_inboard, outboard_node=inboard_rod_outboard)
        spring = Spring(inboard_node=spring_inboard, outboard_node=inboard_rod_inboard, free_length=1, rate=1)
        
        # Full PushPullRod
        push_pull_rod = PushPullRod(outboard_rod=outboard_rod, spring=spring, inboard_rod=inboard_rod, bellcrank=bellcrank)

        outboard_rod_outboard.translate(translation=[0, 0, 1])

        position = [0, 1.25, 1.66144] # from Solidworks
        self.assertEqual([round(x, 5) for x in push_pull_rod.outboard_rod.inboard_node], position)
    
    def test_push_pull_rod_full_outboard_node_rotation_positive(self):
        # Nodes
        outboard_rod_outboard = Node(position=[0, 0, 0])
        outboard_rod_inboard = Node(position=[0, 1, 1])
        inboard_rod_outboard = Node(position=[0, 2, 2])
        inboard_rod_inboard = Node(position=[0, 3, 2])
        
        spring_inboard = Node(position=[0, 4, 2])

        bellcrank_pivot = Node(position=[0, 2, 1])
        bellcrank_direction = [1, 0, 0]

        # Pickup rotation origin
        pickup_rot_origin = Node(position=[0, 1, 0])

        # Links
        bellcrank = Bellcrank(outboard_rod_inboard, inboard_rod_outboard, pivot=bellcrank_pivot, pivot_direction=bellcrank_direction)
        outboard_rod = Link(inboard_node=outboard_rod_inboard, outboard_node=outboard_rod_outboard)
        inboard_rod = Link(inboard_node=inboard_rod_inboard, outboard_node=inboard_rod_outboard)
        spring = Spring(inboard_node=spring_inboard, outboard_node=inboard_rod_inboard, free_length=1, rate=1)
        
        # Full PushPullRod
        push_pull_rod = PushPullRod(outboard_rod=outboard_rod, spring=spring, inboard_rod=inboard_rod, bellcrank=bellcrank)

        outboard_rod_outboard.rotate(origin=pickup_rot_origin, direction=[1, 0, 0], angle=15 * np.pi / 180)

        bellcrank_angle = 15 # from Solidworks
        self.assertEqual(round(push_pull_rod.bellcrank_angle * 180 / np.pi, 2), bellcrank_angle)
    
    def test_push_pull_rod_full_outboard_node_rotation_negative(self):
        # Nodes
        outboard_rod_outboard = Node(position=[0, 0, 0])
        outboard_rod_inboard = Node(position=[0, 1, 1])
        inboard_rod_outboard = Node(position=[0, 2, 2])
        inboard_rod_inboard = Node(position=[0, 3, 2])
        
        spring_inboard = Node(position=[0, 4, 2])

        bellcrank_pivot = Node(position=[0, 2, 1])
        bellcrank_direction = [1, 0, 0]

        # Pickup rotation origin
        pickup_rot_origin = Node(position=[0, 1, 0])

        # Links
        bellcrank = Bellcrank(outboard_rod_inboard, inboard_rod_outboard, pivot=bellcrank_pivot, pivot_direction=bellcrank_direction)
        outboard_rod = Link(inboard_node=outboard_rod_inboard, outboard_node=outboard_rod_outboard)
        inboard_rod = Link(inboard_node=inboard_rod_inboard, outboard_node=inboard_rod_outboard)
        spring = Spring(inboard_node=spring_inboard, outboard_node=inboard_rod_inboard, free_length=1, rate=1)
        
        # Full PushPullRod
        push_pull_rod = PushPullRod(outboard_rod=outboard_rod, spring=spring, inboard_rod=inboard_rod, bellcrank=bellcrank)

        outboard_rod_outboard.rotate(origin=pickup_rot_origin, direction=[1, 0, 0], angle=-15 * np.pi / 180)

        bellcrank_angle = -15 # from Solidworks
        self.assertEqual(round(push_pull_rod.bellcrank_angle * 180 / np.pi, 2), bellcrank_angle)