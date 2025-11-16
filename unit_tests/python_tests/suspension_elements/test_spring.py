from src.vehicle_model.suspension_model.suspension_elements._2_elements.spring import Spring
from src.vehicle_model.suspension_model.suspension_elements._1_elements.node import Node

from unittest import TestCase


class TestSpring(TestCase):
    def test_spring_init(self):
        inboard = Node(position=[0, 0, 0])
        outboard = Node(position=[0, 0.5, 0])
        free_length = 1
        rate = 0.5

        spring = Spring(inboard_node=inboard, outboard_node=outboard, free_length=free_length, rate=rate)
        
        self.assertEqual(spring.free_length, 1)
    
    def test_spring_compression_one(self):
        inboard = Node(position=[0, 0, 0])
        outboard = Node(position=[0, 0.5, 0])
        free_length = 1
        rate = 0.5

        spring = Spring(inboard_node=inboard, outboard_node=outboard, free_length=free_length, rate=rate)
        
        self.assertEqual(spring.compression, 0.5)
    
    def test_spring_compression_two(self):
        inboard = Node(position=[0, 0, 0])
        outboard = Node(position=[0, 0.5, 0])
        free_length = 1
        rate = 0.5

        spring = Spring(inboard_node=inboard, outboard_node=outboard, free_length=free_length, rate=rate)
        
        spring.outboard_node[1] = 0.25

        self.assertEqual(spring.compression, 0.75)
    
    def test_spring_force_one(self):
        inboard = Node(position=[0, 0, 0])
        outboard = Node(position=[0, 0.5, 0])
        free_length = 1
        rate = 0.5

        spring = Spring(inboard_node=inboard, outboard_node=outboard, free_length=free_length, rate=rate)
        
        self.assertEqual(spring.force, 0.25)
    
    def test_spring_force_two(self):
        inboard = Node(position=[0, 0, 0])
        outboard = Node(position=[0, 0.5, 0])
        free_length = 1
        rate = 0.5

        spring = Spring(inboard_node=inboard, outboard_node=outboard, free_length=free_length, rate=rate)
        
        spring.outboard_node[1] = 1

        self.assertEqual(spring.force, 0)